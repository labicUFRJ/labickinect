#include <algorithm>
#include <cassert>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/common/eigen.h>
#include <pcl/io/ply_io.h>
#include "../kinect/kinect_controller.h"
#include "reconstructor.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

Reconstructor::Reconstructor(bool* _stop, Queue<RGBDImage>& q) : stop(_stop), autoSave(false), queue(q) {
	// Reconstructor parameters
	minFeatures      = 150;
	maxFeatures      = 500;
	maxDetectionIte  = 100;
	minMatches       = 25; // min number of visual matches to start ransac
	maxMatchDistance = 10; // max distance of visual match to be valid (pixels)
	minInliersToValidateTransformation = 10; // gamma - min inliers to accept ransac
	
	adjuster  = new FastAdjuster(50, true);
	extractor = new BriefDescriptorExtractor();
	matcher   = new BFMatcher(NORM_HAMMING, true);
	
	ransac = new RANSACAligner();
	ransac->setDistanceThreshold(0.8); // paper: 2.0 pixels
	ransac->setMaxIterations(150);
	ransac->setMinInliers(30);
	ransac->setNumSamples(3);

	framesAnalyzed = 0;
	reconstructionsGenerated = 0;
	reconstructionsAccepted = 0;
	featuresExtracted = 0;
	featuresMatched = 0;
	matchesDiscarded = 0;
	pointsDetected = 0;
	totalTime = 0;
	totalError = 0;
	lastError = 0;

	transformFinal.setIdentity();
	transformPrevious.setIdentity();
}

void Reconstructor::threadFunc() {
	hrclock::time_point t;
	double timeReconstruction;
    cout << "[LabicReconstructor] Reconstructor initialized" << endl;

    while (!*stop || queue.size() > 0) {
    	if (queue.size() > 0) {
    		// If this is the first frame received, just save it
    		if (world.empty()) {
    		    dinfo << "[LabicReconstructor] Preparing first frame" << endl;
    			rgbdPrevious = queue.pop();

    			extractRGBFeatures(rgbdPrevious, featuresPrevious, descriptorsPrevious);

    			world = rgbdPrevious.pointCloud();

    			pointsDetected = world.size();

    			dinfo << "[LabicReconstructor] Initial frame saved" << endl;

    		} else {
				cout << "[LabicReconstructor] Reconstructing frame " << reconstructionsGenerated+1 << "..." << endl;

				rgbdCurrent = queue.pop();

				if (rgbdCurrent == rgbdPrevious) {
					derr << "ERROR - LOOP WITH EQUAL FRAMES" << endl;
					continue;
				}

	        	t = hrclock::now();
	        	performVisualAlignment();
	    		totalTime += (timeReconstruction = diffTime(hrclock::now(), t));

	    		dinfo << "[LabicReconstructor] Finished reconstruction loop (" << timeReconstruction << " secs)" << endl;
				ddebug << queue << endl;
    		}

			framesAnalyzed++;
		}
    	else {
    		//boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    	}
    }

    cout << "[LabicReconstructor] Exporting final world. Please wait... " << endl;
    if (world.size() > 0) {
    	pcl::io::savePLYFileASCII("world.ply", world);
    	cout << "Done." << endl;
    }
    else cout << "Nothing to be done." << endl;

    cout << "[LabicReconstructor] Reconstructor finished" << endl;
}

void Reconstructor::performVisualAlignment() {
	vector<KeyPoint> featuresCurrent;
	Mat descriptorsCurrent;
	vector<DMatch> relatedFeatures;
    vector<Point2f> selectedFeaturePointsCurrent, selectedFeaturePointsPrevious;
    vector<int> transformationInliersIndexes;
	PointCloud<PointXYZRGB> cloudCurrent, featureCloudCurrent, featureCloudPrevious, alignedCloudCurrent;
	Eigen::Matrix4d transform;
	double ransacError;

    transform = Eigen::Matrix4d::Zero();

    // 0. Get PointCloud from previous and current states
    cloudCurrent = rgbdCurrent.pointCloud();
    pointsDetected += cloudCurrent.size();

	// 1. Extract features from both images
	extractRGBFeatures(rgbdCurrent, featuresCurrent, descriptorsCurrent);

	// 2. Get related features (matches) between features from both images
	matchFeatures(featuresCurrent, descriptorsCurrent, featuresPrevious, descriptorsPrevious, relatedFeatures);

	// 2.1. Verify if visual match resulted in the minimal number of associations
	if (relatedFeatures.size() < minMatches) {
		derr << "[LabicReconstructor] IMAGES DO NOT MATCH! ABORTING RECONSTRUCTION. MATCH SAVED TO 'matchfailed.jpg'" << endl;

		Mat matchesMat;
		drawMatches(rgbdCurrent.rgb(), featuresCurrent, rgbdPrevious.rgb(), featuresPrevious, relatedFeatures, matchesMat);
		imwrite("matchfailed.jpg", matchesMat);

		return;
	}

    // 2.2. Apply depth-filter to matches
    for (unsigned int i=0; i<relatedFeatures.size(); i++) {
        int previousIndex = relatedFeatures[i].trainIdx;
        int currentIndex = relatedFeatures[i].queryIdx;
        Point2f previousPoint = featuresPrevious[previousIndex].pt;
        Point2f currentPoint = featuresCurrent[currentIndex].pt;

        if (rgbdPrevious.rgbPixelHasDepth(previousPoint.y, previousPoint.x) &&
        	rgbdCurrent.rgbPixelHasDepth(currentPoint.y, currentPoint.x)) {
        	selectedFeaturePointsPrevious.push_back(previousPoint);
        	selectedFeaturePointsCurrent.push_back(currentPoint);
        } else matchesDiscarded++;
    }

    ddebug << "[LabicReconstructor::performAlignment] Matches after depth filter: " << selectedFeaturePointsPrevious.size() << " points" << endl;

	// 3. Generate PointClouds of related features
    featureCloudPrevious = rgbdPrevious.pointCloudOfSelection(selectedFeaturePointsPrevious);
    featureCloudCurrent = rgbdCurrent.pointCloudOfSelection(selectedFeaturePointsCurrent);

	// 4. Alignment detection
    if (featureCloudPrevious.size() < minInliersToValidateTransformation) {
		derr << "[LabicReconstructor] Too few points to call RANSAC! Aborting reconstruction!" << endl;
		return;
    }

    ransac->estimate(featureCloudPrevious, featureCloudCurrent);
    transform = ransac->getFinalTransform();
    ransacError = ransac->getFinalError();
    transformationInliersIndexes = ransac->getFinalInliers();

    dinfo << "[LabicReconstructor] Final transformation matrix that resulted in " << transformationInliersIndexes.size() << " inliers: " << endl << transform << endl;

    reconstructionsGenerated++;

    // Check if transformation generated the correct set of inliers
    if (transformationInliersIndexes.size() < minInliersToValidateTransformation) {
    	dwarn << "[LabicReconstructor] RANSAC transformation NOT accepted (did not generate the mininum of inliers)" << endl;

    	// Use same transformation as previous reconstruction
		transformFinal = transformPrevious * transformFinal;
		ddebug << "Repeating previous transformation:\n" << transformPrevious << endl;

		transformationInliersIndexes.clear();
		ransacError = lastError;
    } else {
        dinfo << "[LabicReconstructor] Transformation accepted" << endl;
        transformPrevious = transform;
        transformFinal = transform * transformFinal; // TODO or the opposite??????????
        lastError = ransacError;
        reconstructionsAccepted++;
    }

    totalError += ransacError;

	// 5. Apply transformation to all frame points
    ddebug << "Final accumulated transformation:\n" << transformFinal << endl;
    transformPointCloud(cloudCurrent, alignedCloudCurrent, transformFinal);

    // 6. Update 'previous' variables
    rgbdCurrent.copyTo(rgbdPrevious);
    featuresPrevious = featuresCurrent;
    descriptorsCurrent.copyTo(descriptorsPrevious);

    // 7. Update world with new frame
    world += alignedCloudCurrent;
    cout << "[LabicReconstructor] World updated. Total of " << world.size() << " points (estimated size: " << ((world.size()*sizeof(PointXYZRGB))>>20)*1.35 << " MB)" << endl;

    if (autoSave) {
        char filenameply[25];
        sprintf(filenameply, "world%d.ply", reconstructionsGenerated);
    	pcl::io::savePLYFileASCII(filenameply, world);
    	cout << "[LabicReconstructor] World exported with filename '" << filenameply << "'" << endl;
    }

}

void Reconstructor::extractRGBFeatures(const RGBDImage& rgbd, vector<KeyPoint>& keypoints, Mat& descriptors) {
	ddebug << "[LabicReconstructor::extractRGBFeatures] Extracting RGB features and descriptors" << endl;
	
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
		
//        ddebug << "[LabicReconstructor::extractRGBFeatures] Iteration " << i << " found " << keypoints.size() << " points and dropped " << pointsDropped << " points" << endl;
        
		if (keypoints.size()-pointsDropped < minFeatures){
			adjuster->tooFew(minFeatures, keypoints.size());
		} else if (keypoints.size()-pointsDropped > maxFeatures) {
			adjuster->tooMany(maxFeatures, keypoints.size());
		} else {
			break;
		}
	}

	featuresExtracted += keypoints.size();

	ddebug << "[LabicReconstructor::extractRGBFeatures] Extracted " << keypoints.size()
				<< " features (target range: " << minFeatures << " to " << maxFeatures
				<< ", iteration: " << i << ")" << " (dropped " << pointsDropped << " points)" << endl;
}

/**
 * q -> query (current / source)
 * t -> train (previous / target)
 */
void Reconstructor::matchFeatures(vector<KeyPoint>& _keypoints_q, const Mat& _descriptors_q, vector<KeyPoint>& _keypoints_t, const Mat& _descriptors_t, vector<DMatch>& _matches) {
	ddebug << "[LabicReconstructor::matchFeatures] Matching features\n";
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

	ddebug << "[LabicReconstructor::matchFeatures] RGB matches after threshold: " << _matches.size() << " (initial: " << matches.size() << ")" << endl;
}

void Reconstructor::printStats() const {
	cout << "[LabicReconstructor] STATS" << endl
		 << "	Frames analyzed: " << framesAnalyzed << endl;
	if (framesAnalyzed > 1)
	cout << "	Total error: " << totalError << " (avg. " << totalError/reconstructionsGenerated << ")" << endl
		<< "	Features extracted: " << featuresExtracted << " (avg. " << featuresExtracted/framesAnalyzed << ")" << endl
		 << "	Features matched: " << featuresMatched << " (avg. " << featuresMatched/(framesAnalyzed-1) << ")" << endl
		 << "	Matches discarded: " << matchesDiscarded << " (avg. " << matchesDiscarded/(framesAnalyzed-1) << ")" << endl
		 << "	Points detected: " << pointsDetected << " (avg. " << pointsDetected/framesAnalyzed << ")" << endl
		 << "	Reconstructions generated: " << reconstructionsGenerated << " (" << reconstructionsGenerated-reconstructionsAccepted << " NOT accepted)" << endl
		 << "	Reconstruction time: " << totalTime << " secs (avg. " << totalTime/reconstructionsGenerated << " secs)" << endl
	;
}
