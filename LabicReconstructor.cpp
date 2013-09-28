#include <algorithm>
#include <cassert>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/common/eigen.h>
#include <pcl/io/ply_io.h>
#include "LabicKinect.h"
#include "LabicReconstructor.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

LabicReconstructor::LabicReconstructor(bool* _stop) : stop(_stop) {
	
	ID = 0;
	
	minFeatures     = 150;
	maxFeatures     = 500;
	maxDetectionIte = 100;
	minMatches      = 25;
	maxMatchDistance = 10;
	
	adjuster  = new FastAdjuster(100, true);
	extractor = new BriefDescriptorExtractor();
	matcher   = new BFMatcher(NORM_HAMMING, true); // gives infinite distance between matchings
	matcher2  = new BFMatcher(NORM_HAMMING, false);
	depthPrevious = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
	
	ransac = new RANSACAligner();
	ransac->setDistanceThreshold(0.5); // 1.0
	ransac->setMaxIterations(100);
	ransac->setMinInliers(20);
	ransac->setNumSamples(3);

	minInliersToValidateTransformation = 10;
	reconstructionsGenerated = 0;
	reconstructionsAccepted = 0;

}

void LabicReconstructor::reconstruct() {
    cout << "[LabicReconstructor] Reconstructor initialized" << endl;

    while (!*stop) {
    	if (cv->isReady()) {
    		// If this is the first frame received, just save it
    		if (world.empty()) {
    		    cout << "[LabicReconstructor] Preparing first frame" << endl;
    			cv->rgbCurrent.copyTo(rgbPrevious);
    			copy(cv->depthCurrent, cv->depthCurrent + width*height, depthPrevious);

    		    frameToPointCloud(rgbPrevious, depthPrevious, alignedCloudPrevious);
    			extractRGBFeatures(rgbPrevious, depthPrevious, featuresPrevious, descriptorsPrevious);

    			world += alignedCloudPrevious;
    			transformPrevious.setIdentity();

    			pcl::io::savePLYFileASCII("world0.ply", world);

				cout << endl << "[LabicReconstructor] Initial frame saved" << endl;

    		} else {
				cout << endl << "[LabicReconstructor] Reconstructor got frames. Reconstructing..." << endl;
				imwrite("rgbPrevious.jpg", rgbPrevious);
				imwrite("rgbCurrent.jpg", cv->rgbCurrent);
				performLoop(cv->rgbCurrent, cv->depthCurrent);

				cout << "[LabicReconstructor] Finished reconstruction loop" << endl << endl;
    		}
			cv->restartState();
		}
    	else {
    		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    	}
    }
    
    cout << "[LabicReconstructor] Reconstructor finished" << endl;
}

void LabicReconstructor::performLoop(const Mat& rgbCurrent,
								 	 const uint16_t* depthCurrent) {
	
	vector<KeyPoint> featuresCurrent;
	Mat descriptorsCurrent, matchesMat;
	vector<DMatch> relatedFeatures;
    vector<Point2f> selectedFeaturePointsCurrent, selectedFeaturePointsPrevious;
	PointCloud<PointXYZRGB> cloudCurrent, featureCloudCurrent, featureCloudPrevious, alignedCloudCurrent;
	Eigen::Matrix4d transform = Eigen::Matrix4d::Zero();
    vector<int> transformationInliersIndexes;

    // 0. Get PointCloud from previous and current states
    frameToPointCloud(rgbCurrent, depthCurrent, cloudCurrent);

    pcl::io::savePLYFileASCII("cloudCurrent.ply", cloudCurrent);

	// 1. Extract features from both images
	extractRGBFeatures(rgbCurrent, depthCurrent, featuresCurrent, descriptorsCurrent);
	
	// 2. Get related features (matches) between features from both images
	matchFeatures(featuresPrevious, descriptorsPrevious, featuresCurrent, descriptorsCurrent, relatedFeatures);
	drawMatches(rgbPrevious, featuresPrevious, rgbCurrent, featuresCurrent, relatedFeatures, matchesMat);
	if (relatedFeatures.size() < minMatches) {
		cerr << "[LabicReconstructor::performLoop] IMAGES DO NOT MATCH! ABORTING RECONSTRUCTION" << endl;
		imwrite("matchfailed.jpg", matchesMat);
		return;
	}

    for (int i=0; i<relatedFeatures.size(); i++) {
        int previousIndex = relatedFeatures[i].trainIdx;
        int currentIndex = relatedFeatures[i].queryIdx;
        Point2f previousPoint = featuresPrevious[previousIndex].pt;
        Point2f currentPoint = featuresCurrent[currentIndex].pt;
        // Discard matches that do not have depth information
        if (depthPrevious[(int)(width*previousPoint.y + previousPoint.x)] > 0 &&
        	depthCurrent[(int)(width*currentPoint.y + currentPoint.x)] > 0) {
        	selectedFeaturePointsPrevious.push_back(previousPoint);
        	selectedFeaturePointsCurrent.push_back(currentPoint);
        }
    }
	
    cout << "[LabicReconstructor::performLoop] Matches after depth filter: " << selectedFeaturePointsPrevious.size() << " points" << endl;
    
	// 3. Generate PointClouds of related features
    frameToPointCloud(rgbPrevious, depthPrevious, featureCloudPrevious, selectedFeaturePointsPrevious);
    frameToPointCloud(rgbCurrent, depthCurrent, featureCloudCurrent, selectedFeaturePointsCurrent);
    // As the previous frame already had a transformation, apply it to the featureCloud so it matches the previous alignment
    transformPointCloud(featureCloudPrevious, featureCloudPrevious, transformPrevious);
    
    cout << "[LabicReconstructor::performLoop] Feature cloud being transformed with " << featureCloudPrevious.size() << " points" << endl;
	// 4. Alignment detection
//    cout << "[LabicReconstructor::performLoop] Transformation matrix before RANSAC:" << endl << transform << endl;

    ransac->estimate(featureCloudPrevious, featureCloudCurrent);
    transform = ransac->getFinalTransform();
    transformationInliersIndexes = ransac->getFinalInliers();
	
    cout << "[LabicReconstructor::performLoop] Final transformation matrix resulted in " << transformationInliersIndexes.size() << " inliers: " << endl << transform << endl;

    reconstructionsGenerated++;

    // Check if transformation generated the correct set of inliers
    if (transformationInliersIndexes.size() < minInliersToValidateTransformation) {
    	cout << "[LabicReconstructor::performLoop] Transformation NOT accepted (did not generate the mininum of inliers). Using previous transformation:" << endl
    		 << transformPrevious << endl;
    	transform = transformPrevious;
    } else {
        cout << "[LabicReconstructor::performLoop] Transformation accepted" << endl;
        reconstructionsAccepted++;
    }

	
	// 5. Apply transformation to all frame points
    cout << "[LabicReconstructor::performLoop] Transforming cloud to world " << endl;
    transformPointCloud(cloudCurrent, alignedCloudCurrent, transform);
    
    alignedCloudPrevious = PointCloud<PointXYZRGB>(alignedCloudCurrent);
    rgbCurrent.copyTo(rgbPrevious);
    copy(depthCurrent, depthCurrent + width*height, depthPrevious);
    featuresPrevious = featuresCurrent;
    descriptorsCurrent.copyTo(descriptorsPrevious);
    world += alignedCloudCurrent;
    cout << "[LabicReconstructor::performLoop] WORLD UPDATED - " << world.size() << " points (+" << alignedCloudCurrent.size() << " added)" << endl;

    char filenamejpg[15], filenameply[15];
    sprintf(filenamejpg, "matches%d.jpg", reconstructionsAccepted);
    sprintf(filenameply, "world%d.ply", reconstructionsAccepted);
    imwrite(filenamejpg, matchesMat);
    pcl::io::savePLYFileASCII(filenameply, world);
	
}

void LabicReconstructor::extractRGBFeatures(const Mat& img, const uint16_t* depth, vector<KeyPoint>& keypoints, Mat& descriptors) {
	cout << "[LabicReconstructor::extractRGBFeatures] Extracting RGB features and descriptors" << endl;
	
    Mat imgBlackWhite;

    int pointsDropped = 0;
    int i, j;
    cvtColor(img, imgBlackWhite, CV_RGB2GRAY);
    
	for (i=0; i<maxDetectionIte; i++) {
		adjuster->detect(imgBlackWhite, keypoints);
		extractor->compute(imgBlackWhite, keypoints, descriptors);
        
		// Filter features to garantee depth information
        pointsDropped = 0;
		for (j=0; j<keypoints.size(); j++) {
			int keypointIndex = width*keypoints[j].pt.y + keypoints[j].pt.x;
			float keypointDepth = depth[keypointIndex];
			// If point is in origin, it does not have depth information available
			// Therefore, it should not be considered a keypoint
			if (keypointDepth == 0) {
				/*cout << "[LabicReconstructor::extractRGBFeatures] Dropping point (" << keypoints[j].pt.x
					<< ", " << keypoints[j].pt.y << ") with depth " << keypointDepth << "." << endl;*/
				pointsDropped++;
			}
		}
		
//        cout << "[LabicReconstructor::extractRGBFeatures] Iteration " << i << " found " << keypoints.size() << " points and dropped " << pointsDropped << " points" << endl;
        
		if (keypoints.size()-pointsDropped < minFeatures){
			adjuster->tooFew(minFeatures, keypoints.size());
		} else if (keypoints.size()-pointsDropped > maxFeatures) {
			adjuster->tooMany(maxFeatures, keypoints.size());
		} else {
			break;
		}
	}

	cout << "[LabicReconstructor::extractRGBFeatures] Extracted " << keypoints.size()
	<< " features (target range: " << minFeatures << " to " << maxFeatures
	<< ", iteration: " << i << ")" << " (dropped " << pointsDropped << " points)" << endl;
};

void LabicReconstructor::matchFeatures(vector<KeyPoint>&   _keypoints_q,
									   const Mat&               _descriptors_q,
									   vector<KeyPoint>&   _keypoints_t,
									   const Mat&               _descriptors_t,
									   vector<DMatch>&     _matches) const {
	
	cout << "[LabicReconstructor::matchFeatures] Matching features\n";
	vector<DMatch> matches;
	
	matcher2->match(_descriptors_q, _descriptors_t, matches);
	cout << "[LabicReconstructor::matchFeatures] Inital matched features: " << matches.size() << endl;
		
	_matches.clear();
	
	// Distance filter
	for (int i=0; i<matches.size(); i++) {
		if (matches[i].distance < maxMatchDistance) {
			_matches.push_back(matches[i]);
		}
	}
	
	cout << "[LabicReconstructor::matchFeatures] Final matches after matching threshold: " << _matches.size() << endl;
	
}

void LabicReconstructor::start() {
    m_Thread = boost::thread(&LabicReconstructor::reconstruct, this);
}

void LabicReconstructor::join() {
    m_Thread.join();
}

void LabicReconstructor::close() {
    // Extra code if need to 
    join();
}
