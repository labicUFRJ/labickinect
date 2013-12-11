#include <cassert>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/common/eigen.h>
#include "visual_reconstructor.h"
#include "reconstruction_exception.h"


using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

VisualReconstructor::VisualReconstructor(): ransacError(0.0), cachedPrevious(0) {
	// Reconstructor parameters
	minFeatures      = 150;
	maxFeatures      = 500;
	maxDetectionIte  = 100;
	minInliers       = 30; // min inliers necessary to match and ransac
	maxMatchDistance = 10; // max distance of visual match to be valid (pixels)

	adjuster = new FastAdjuster(50, true);
	extractor = new BriefDescriptorExtractor();
	matcher = new BFMatcher(NORM_HAMMING, true);

	ransac.setDistanceThreshold(0.8); // paper: 2.0 pixels
	ransac.setMaxIterations(150);
	ransac.setMinInliers(minInliers);
	ransac.setNumSamples(3);
}

void VisualReconstructor::operator () (RGBDImage& rgbdPrevious, RGBDImage& rgbdCurrent) {
	vector<KeyPoint> featuresCurrent;
	Mat descriptorsCurrent;
	vector<DMatch> relatedFeatures;
    vector<Point2f> selectedFeaturePointsCurrent, selectedFeaturePointsPrevious;
    Cloud featureCloudCurrent, featureCloudPrevious, alignedCloudCurrent;

	// Clear results from last alignment
    transform = Eigen::Matrix4d::Zero();
    transformationInliersIndexes.clear();
	ransacError = 0.0;

	// 1. Extract features from both images // TODO test
	// Test if previous features/descriptors are cached
	if (cachedPrevious != rgbdPrevious.timestamp()) {
		if (cachedPrevious) dwarn << "[VisualReconstructor] Previous cache does not match parameter" << endl;
		extractRGBFeatures(rgbdPrevious, featuresPrevious, descriptorsPrevious);
	}
	extractRGBFeatures(rgbdCurrent, featuresCurrent, descriptorsCurrent);

	// 2. Get related features (matches) between features from both images
	matchFeatures(featuresCurrent, descriptorsCurrent, featuresPrevious, descriptorsPrevious, relatedFeatures);

	// 2.1. Verify if visual match resulted in the minimal number of associations
	if (relatedFeatures.size() < minInliers) {
		Mat matchesMat;
		drawMatches(rgbdCurrent.rgb(), featuresCurrent, rgbdPrevious.rgb(), featuresPrevious, relatedFeatures, matchesMat);
		imwrite("matchfailed.jpg", matchesMat);

		throw ReconstructionException("IMAGES DO NOT MATCH! ABORTING RECONSTRUCTION. MATCH SAVED TO 'matchfailed.jpg'");
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
        }
    }

    ddebug << "[VisualReconstructor] Matches after depth filter: " << selectedFeaturePointsPrevious.size() << " points" << endl;

	// 3. Generate PointClouds of related features
    featureCloudPrevious = rgbdPrevious.pointCloudOfSelection(selectedFeaturePointsPrevious);
    featureCloudCurrent = rgbdCurrent.pointCloudOfSelection(selectedFeaturePointsCurrent);

	// 4. Alignment detection
    if (featureCloudPrevious.size() < minInliers) {
		throw ReconstructionException("Too few points to call RANSAC");
		return;
    }

    ransac.estimate(featureCloudPrevious, featureCloudCurrent);
    transform = ransac.getFinalTransform();
    ransacError = ransac.getFinalError();
    transformationInliersIndexes = ransac.getFinalInliers();

    cachedPrevious = rgbdCurrent.timestamp();
	featuresPrevious = featuresCurrent;
	descriptorsCurrent.copyTo(descriptorsPrevious);
}


void VisualReconstructor::extractRGBFeatures(const RGBDImage& rgbd, vector<KeyPoint>& keypoints, Mat& descriptors) {
	ddebug << "[VisualReconstructor] Extracting RGB features and descriptors" << endl;

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

//	featuresExtracted += keypoints.size();

	ddebug << "[VisualReconstructor] Extracted " << keypoints.size()
				<< " features (target range: " << minFeatures << " to " << maxFeatures
				<< ", iteration: " << i << ")" << " (dropped " << pointsDropped << " points)" << endl;
}

/**
 * q -> query (current / source)
 * t -> train (previous / target)
 */
void VisualReconstructor::matchFeatures(vector<KeyPoint>& _keypoints_q, const Mat& _descriptors_q, vector<KeyPoint>& _keypoints_t, const Mat& _descriptors_t, vector<DMatch>& _matches) {
	ddebug << "[VisualReconstructor] Matching features\n";
	vector<DMatch> matches;

	matcher->match(_descriptors_q, _descriptors_t, matches);

	_matches.clear();

	// Distance filter
	for (unsigned int i=0; i<matches.size(); i++) {
		if (matches[i].distance < maxMatchDistance) {
			_matches.push_back(matches[i]);
		}
	}

//	featuresMatched += _matches.size();

	ddebug << "[LabicReconstructor::matchFeatures] RGB matches after threshold: " << _matches.size() << " (initial: " << matches.size() << ")" << endl;
}
