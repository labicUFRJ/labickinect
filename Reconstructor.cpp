#include <algorithm>
#include <cassert>
#include <ctime>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/common/eigen.h>
#include <pcl/io/ply_io.h>
#include "KinectController.h"
#include "Reconstructor.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

Reconstructor::Reconstructor(bool* _stop, FrameQueue& q) : stop(_stop), autoSave(false), queue(q) {
	// Reconstructor parameters
	minFeatures      = 200;
	maxFeatures      = 500;
	maxDetectionIte  = 100;
	minMatches       = 25; // min number of visual matches to start ransac
	maxMatchDistance = 10; // max distance of visual match to be valid
	minInliersToValidateTransformation = 10; // gamma - min inliers to accept ransac
	
	adjuster  = new FastAdjuster(100, true);
	extractor = new BriefDescriptorExtractor();
	matcher   = new BFMatcher(NORM_HAMMING, true); // gives infinite distance between matchings
	matcher2  = new BFMatcher(NORM_HAMMING, false);
	
	ransac = new RANSACAligner();
	ransac->setDistanceThreshold(1.0); // paper: 2.0 pixels
	ransac->setMaxIterations(150);
	ransac->setMinInliers(50);
	ransac->setNumSamples(3);

	framesAnalyzed = 0;
	reconstructionsGenerated = 0;
	reconstructionsAccepted = 0;
	featuresExtracted = 0;
	featuresMatched = 0;
	matchesDiscarded = 0;
	pointsDetected = 0;
	totalTime = 0;

	transformPrevious.setIdentity();
}

void Reconstructor::threadFunc() {
	clock_t t;
    cout << "[LabicReconstructor] Reconstructor initialized" << endl;

    while (!*stop || queue.size() > 0) {
    	if (queue.size() > 0) {
    		// If this is the first frame received, just save it
    		if (world.empty()) {
    		    cout << "[LabicReconstructor] Preparing first frame" << endl;
    			rgbdPrevious = queue.pop();

    			extractRGBFeatures(rgbdPrevious, featuresPrevious, descriptorsPrevious);

    			world = rgbdPrevious.pointCloud();

    			pointsDetected = world.size();

				cout << "[LabicReconstructor] Initial frame saved" << endl;

    		} else {
				cout << "[LabicReconstructor] Reconstructor got frames. Reconstructing..." << endl;

				rgbdCurrent = queue.pop();

				if (rgbdCurrent == rgbdPrevious) {
					cerr << "ERROR - LOOP WITH EQUAL FRAMES" << endl;
					continue;
				}

				//imwrite("rgbPrevious.jpg", rgbdPrevious.rgb());
				//imwrite("rgbCurrent.jpg", rgbdCurrent.rgb());

	        	t = clock();
	        	performLoop();
	    		totalTime += t = clock() - t;

				cout << "[LabicReconstructor] Finished reconstruction loop (" << ((float)t)/CLOCKS_PER_SEC << " secs)" << endl;
				queue.printStatus();
    		}

			framesAnalyzed++;
		}
    	else {
    		//boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    	}
    }
    
    printStats();

    cout << "[LabicReconstructor] Exporting final world" << endl;
    if (world.size() > 0) pcl::io::savePLYFileASCII("world.ply", world);
    else cout << "[LabicReconstructor] World empty" << endl;

    cout << "[LabicReconstructor] Reconstructor finished" << endl;
}

void Reconstructor::performLoop() {
	vector<KeyPoint> featuresCurrent;
	Mat descriptorsCurrent, matchesMat;
	vector<DMatch> relatedFeatures;
    vector<Point2f> selectedFeaturePointsCurrent, selectedFeaturePointsPrevious;
	PointCloud<PointXYZRGB> cloudCurrent, featureCloudCurrent, featureCloudPrevious, alignedCloudCurrent;
	Eigen::Matrix4d transform, transformFinal;
    vector<int> transformationInliersIndexes;

    transform = transformFinal = Eigen::Matrix4d::Zero();

    // 0. Get PointCloud from previous and current states
    cloudCurrent = rgbdCurrent.pointCloud();
    pointsDetected += cloudCurrent.size();

	// 1. Extract features from both images
	extractRGBFeatures(rgbdCurrent, featuresCurrent, descriptorsCurrent);

	// 2. Get related features (matches) between features from both images
	matchFeatures(featuresCurrent, descriptorsCurrent, featuresPrevious, descriptorsPrevious, relatedFeatures);
	drawMatches(rgbdCurrent.rgb(), featuresCurrent, rgbdPrevious.rgb(), featuresPrevious, relatedFeatures, matchesMat);

	if (relatedFeatures.size() < minMatches) {
		cerr << "[LabicReconstructor::performLoop] IMAGES DO NOT MATCH! ABORTING RECONSTRUCTION" << endl;
		imwrite("matchfailed.jpg", matchesMat);
		return;
	}

    // Filter matches and discard matches that do not have depth information on both images
    for (unsigned int i=0; i<relatedFeatures.size(); i++) {
        int previousIndex = relatedFeatures[i].trainIdx;
        int currentIndex = relatedFeatures[i].queryIdx;
        Point2f previousPoint = featuresPrevious[previousIndex].pt;
        Point2f currentPoint = featuresCurrent[currentIndex].pt;

        if (rgbdPrevious.rgbPixelHasDepth(previousPoint.y, previousPoint.x) &&
        	rgbdCurrent.rgbPixelHasDepth(currentPoint.y, currentPoint.x)) {
        	selectedFeaturePointsPrevious.push_back(previousPoint);
        	selectedFeaturePointsCurrent.push_back(currentPoint);
        } else {
        	matchesDiscarded++;
        }
    }

    cout << "[LabicReconstructor::performLoop] Matches after depth filter: " << selectedFeaturePointsPrevious.size() << " points" << endl;

	// 3. Generate PointClouds of related features
    featureCloudPrevious = rgbdPrevious.pointCloudOfSelection(selectedFeaturePointsPrevious);
    featureCloudCurrent = rgbdCurrent.pointCloudOfSelection(selectedFeaturePointsCurrent);

    //cout << "[LabicReconstructor::performLoop] Feature cloud being transformed with " << featureCloudPrevious.size() << " points" << endl;

	// 4. Alignment detection
    if (featureCloudPrevious.size() < 10) {
		cerr << "[LabicReconstructor::performLoop] Too few points to call RANSAC! Aborting reconstruction!" << endl;
		return;
    }

    ransac->estimate(featureCloudPrevious, featureCloudCurrent);
    transform = ransac->getFinalTransform();
    transformationInliersIndexes = ransac->getFinalInliers();

    cout << "[LabicReconstructor::performLoop] Final transformation matrix resulted in " << transformationInliersIndexes.size() << " inliers: " << endl << transform << endl;

    reconstructionsGenerated++;

    // Check if transformation generated the correct set of inliers
    if (transformationInliersIndexes.size() < minInliersToValidateTransformation) {
    	cerr << "[LabicReconstructor::performLoop] RANSAC transformation NOT accepted (did not generate the mininum of inliers). ";

    	// Use same transformation as previous reconstruction
		transformFinal = transformPrevious;
		cerr << "Using previous transformation:" << endl << transformFinal << endl;
		transformationInliersIndexes.clear();

    } else {
        cout << "[LabicReconstructor::performLoop] Transformation accepted" << endl;
        // As the previous frame already had a transformation, multiply it so it matches the previous alignment
        transformFinal = transformPrevious * transform; // TODO or the opposite??????????
        cout << "			transformFinal:" << endl << transformFinal << endl;
        transformPrevious = transformFinal;
        reconstructionsAccepted++;
    }

	// 5. Apply transformation to all frame points
    transformPointCloud(cloudCurrent, alignedCloudCurrent, transformFinal);

    // 6. Update 'previous' variables
    rgbdCurrent.copyTo(rgbdPrevious);
    featuresPrevious = featuresCurrent;
    descriptorsCurrent.copyTo(descriptorsPrevious);
    world += alignedCloudCurrent;
    cout << "[LabicReconstructor::performLoop] World updated. Total of " << world.size() << " points (approximately " << ((world.size()*sizeof(PointXYZRGB))>>20)*1.35 << " MB)" << endl;


    if (autoSave) {
        char filenamejpg[25], filenameply[25];
        sprintf(filenamejpg, "matches%d.jpg", reconstructionsAccepted);
        sprintf(filenameply, "world%d.ply", reconstructionsAccepted);
        imwrite(filenamejpg, matchesMat);
    	pcl::io::savePLYFileASCII(filenameply, world);

    	cout << "[LabicReconstructor] World saved with filename '" << filenameply << "'" << endl;
    }

}

void Reconstructor::extractRGBFeatures(const RGBDImage& rgbd, vector<KeyPoint>& keypoints, Mat& descriptors) {
//	cout << "[LabicReconstructor::extractRGBFeatures] Extracting RGB features and descriptors" << endl;
	
    Mat imgBlackWhite;

    int pointsDropped = 0;
    unsigned int i, j;
    cvtColor(rgbd.rgb(), imgBlackWhite, CV_RGB2GRAY);
    
	for (i=0; i<maxDetectionIte; i++) {
		adjuster->detect(imgBlackWhite, keypoints);
		extractor->compute(imgBlackWhite, keypoints, descriptors);
        
		// Filter features to garantee depth information
        pointsDropped = 0;
		for (j=0; j<keypoints.size(); j++) {
			if (!rgbd.rgbPixelHasDepth(keypoints[j].pt.y, keypoints[j].pt.x)) {
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

	featuresExtracted += keypoints.size();

	cout << "[LabicReconstructor::extractRGBFeatures] Extracted " << keypoints.size()
	<< " features (target range: " << minFeatures << " to " << maxFeatures
	<< ", iteration: " << i << ")" << " (dropped " << pointsDropped << " points)" << endl;
}

/**
 * q -> query (current / source)
 * t -> train (previous / target)
 */
void Reconstructor::matchFeatures(vector<KeyPoint>& _keypoints_q, const Mat& _descriptors_q, vector<KeyPoint>& _keypoints_t, const Mat& _descriptors_t, vector<DMatch>& _matches) {
//	cout << "[LabicReconstructor::matchFeatures] Matching features\n";
	vector<DMatch> matches;
	
	matcher->match(_descriptors_q, _descriptors_t, matches);

	_matches.clear();
	
	// Distance filter
	for (unsigned int i=0; i<matches.size(); i++) {
		if (matches[i].distance < maxMatchDistance) {
			_matches.push_back(matches[i]);
		}
	}
	
	featuresMatched += _matches.size();

	cout << "[LabicReconstructor::matchFeatures] RGB matches after threshold: " << _matches.size() << " (initial: " << matches.size() << ")" << endl;
}

void Reconstructor::printStats() const {
	cout << "[LabicReconstructor] STATS" << endl
		 << "	Frames analyzed: " << framesAnalyzed << endl;
	if (framesAnalyzed > 1)
	cout << "	Features extracted: " << featuresExtracted << " (avg. " << featuresExtracted/framesAnalyzed << ")" << endl
		 << "	Features matched: " << featuresMatched << " (avg. " << featuresMatched/(framesAnalyzed-1) << ")" << endl
		 << "	Matches discarded: " << matchesDiscarded << " (avg. " << matchesDiscarded/(framesAnalyzed-1) << ")" << endl
		 << "	Points detected: " << pointsDetected << " (avg. " << pointsDetected/framesAnalyzed << ")" << endl
		 << "	Reconstructions generated: " << reconstructionsGenerated << endl
		 << "	Reconstructions accepted: " << reconstructionsAccepted << endl
		 << "	Reconstruction time: " << ((float)totalTime)/CLOCKS_PER_SEC << " secs (avg. " << (((float)totalTime)/CLOCKS_PER_SEC)/reconstructionsAccepted << " secs)" << endl
		 ;
}
