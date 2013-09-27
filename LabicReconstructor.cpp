#include "LabicReconstructor.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

LabicReconstructor::LabicReconstructor(bool* _stop) : stop(_stop) {
	
	ID = 0;
	
	minFeatures     = 100;
	maxFeatures     = 500;
	maxDetectionIte = 100;
	minMatches      = 10;
	maxMatchDistance = 5;
	
	adjuster  = new FastAdjuster(100, true);
	extractor = new BriefDescriptorExtractor();
	matcher   = new BFMatcher(NORM_HAMMING, true); // gives infinite distance between matchings
	matcher2  = new BFMatcher(NORM_HAMMING, false);
	
	ransac = new RANSACAligner();
	ransac->setDistanceThreshold(1.0);
	ransac->setMaxIterations(100);
	ransac->setMinInliers(10);
	ransac->setNumSamples(3);

	minInliersToValidateTransformation = 10;
	reconstructionsGenerated = 0;
	reconstructionsAccepted = 0;

};

void LabicReconstructor::reconstruct() {
    cout << "[LabicReconstructor] Reconstructor initialized" << endl;

    while (!*stop) {
    	if (cv->isReady()) {
			cout << endl << "[LabicReconstructor] Reconstructor got frames. Reconstructing..." << endl;

			performLoop(cv->rgbCurrent, cv->rgbPrevious, cv->depthCurrent, cv->depthPrevious);

			cout << "[LabicReconstructor] Finished reconstruction loop" << endl << endl;

			cv->restartState();
		}
    	else {
    		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    	}
    }
    
    cout << "[LabicReconstructor] Reconstructor finished" << endl;
}

void LabicReconstructor::performLoop(const Mat& rgbCurrent,
								  const Mat& rgbPrevious,
								  const uint16_t* depthCurrent,
								  const uint16_t* depthPrevious) {
	
	vector<KeyPoint> featuresCurrent, featuresPrevious;
	Mat descriptorsCurrent, descriptorsPrevious;
	vector<DMatch> relatedFeatures;
    vector<Point2f> selectedFeaturePointsCurrent, selectedFeaturePointsPrevious;
	PointCloud<PointXYZRGB> cloudCurrent, cloudPrevious, featureCloudCurrent, featureCloudPrevious, transformedCloudCurrent;
	Eigen::Matrix4d transform = Eigen::Matrix4d::Zero();
    vector<int> transformationInliersIndexes;

    // 0. Get PointCloud from previous and current states
    Kinect::frameToPointCloud(rgbPrevious, depthPrevious, cloudPrevious);
    Kinect::frameToPointCloud(rgbCurrent, depthCurrent, cloudCurrent);

    // Initialize world with first cloud
    if (world.empty()) {
    	world = cloudPrevious;
    	pcl::io::savePLYFileASCII("world0.ply", world);
    }

	// 1. Extract features from both images
	extractRGBFeatures(rgbPrevious, depthPrevious, featuresPrevious, descriptorsPrevious);
	extractRGBFeatures(rgbCurrent, depthCurrent, featuresCurrent, descriptorsCurrent);
	
	// 2. Get related features (matches) between features from both images
	matchFeatures(featuresPrevious, descriptorsPrevious, featuresCurrent, descriptorsCurrent, relatedFeatures);
	if (relatedFeatures.size() < minMatches) {
		cerr << "[LabicReconstructor::performLoop] IMAGES DO NOT MATCH! ABORTING RECONSTRUCTION" << endl;
		return;
	}
    //LabicCV::showMatchesPreview(rgbPrevious, featuresPrevious, rgbCurrent, featuresCurrent, relatedFeatures);

    for (int i=0; i<relatedFeatures.size(); i++) {
        int previousIndex = relatedFeatures[i].trainIdx;
        int currentIndex = relatedFeatures[i].queryIdx;
        Point2f previousPoint = featuresPrevious[previousIndex].pt;
        Point2f currentPoint = featuresCurrent[currentIndex].pt;
        // Discard matches that do not have depth information
        if (depthPrevious[(int)(640*previousPoint.y + previousPoint.x)] > 0 &&
        	depthCurrent[(int)(640*currentPoint.y + currentPoint.x)] > 0) {
        	selectedFeaturePointsPrevious.push_back(previousPoint);
        	selectedFeaturePointsCurrent.push_back(currentPoint);
        }
    }
	
    cout << "[LabicReconstructor::performLoop] selectedFeatureIndexesPrevious: " << selectedFeaturePointsPrevious.size() << " poins" << endl
    << "[LabicReconstructor::performLoop] selectedFeatureIndexesCurrent: " << selectedFeaturePointsCurrent.size() << " points" << endl;
    
	// 3. Generate PointClouds of related features (pointcloudsrc, pointcloudtgt)
    Kinect::frameToPointCloud(rgbPrevious, depthPrevious, featureCloudPrevious, selectedFeaturePointsPrevious);
    Kinect::frameToPointCloud(rgbCurrent, depthCurrent, featureCloudCurrent, selectedFeaturePointsCurrent);
    
    cout << "[LabicReconstructor::performLoop] featureCloudPrevious: " << featureCloudPrevious.size() << " points" << endl
    << "[LabicReconstructor::performLoop] featureCloudCurrent: " << featureCloudCurrent.size() << " points" << endl;
	
	// 4. Alignment detection
    cout << "[LabicReconstructor::performLoop] Transformation matrix before RANSAC:" << endl << transform << endl;

    ransac->estimate(featureCloudPrevious, featureCloudCurrent);
    transform = ransac->getFinalTransform();
    transformationInliersIndexes = ransac->getFinalInliers();
	
    cout << "[LabicReconstructor::performLoop] Final transformation matrix resulted in " << transformationInliersIndexes.size() << " inliers: " << endl << transform << endl;

    reconstructionsGenerated++;

    // Check if transformation generated the correct set of inliers
    if (transformationInliersIndexes.size() < minInliersToValidateTransformation) {
    	cout << "[LabicReconstructor::performLoop] Final transformation did not generate the mininum of inliers." << endl;
    	return;
    }

    reconstructionsAccepted++;

    cout << "[LabicReconstructor::performLoop] Transformation accepted. Transforming cloud to world " << endl;
	
	// 5. Apply transformation to all frame points
    transformPointCloud(cloudCurrent, transformedCloudCurrent, transform);
    
    world += transformedCloudCurrent;
    cout << "[LabicReconstructor::performLoop] WORLD UPDATED - " << world.size() << " points (+" << transformedCloudCurrent.size() << " added)" << endl;
    char filename[15];
    sprintf(filename, "world%d.ply", reconstructionsAccepted);
    pcl::io::savePLYFileASCII(filename, world);
	
}

void LabicReconstructor::extractRGBFeatures(const Mat& img, const uint16_t* depth, vector<KeyPoint>& keypoints, Mat& descriptors) {
	cout << "[LabicReconstructor::extractRGBFeatures] computing features" << endl;
	
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
			int keypointIndex = 640*keypoints[j].pt.y + keypoints[j].pt.x;
			float keypointDepth = depth[keypointIndex];
			// If point is in origin, it does not have depth information available
			// Therefore, it should not be considered a keypoint
			if (keypointDepth == 0) {
				/*cout << "[LabicReconstructor::extractRGBFeatures] Dropping point (" << keypoints[j].pt.x
					<< ", " << keypoints[j].pt.y << ") with depth " << keypointDepth << "." << endl;*/
				pointsDropped++;
			}
		}
		
        cout << "[LabicReconstructor::extractRGBFeatures] Iteration " << i << " found " << keypoints.size() << " points and dropped " << pointsDropped << " points" << endl;
        
		if (keypoints.size()-pointsDropped < minFeatures){
			adjuster->tooFew(minFeatures, keypoints.size());
		} else if (keypoints.size()-pointsDropped > maxFeatures) {
			adjuster->tooMany(maxFeatures, keypoints.size());
		} else {
			break;
		}
	}

	cout << "[LabicReconstructor::extractRGBFeatures] the number of features: " << keypoints.size()
	<< "(target range: " << minFeatures << " to " << maxFeatures
	<< ", iteration: " << i << ")" << " (dropped " << pointsDropped << " points)" << endl;
};

void LabicReconstructor::matchFeatures(vector<KeyPoint>&   _keypoints_q,
									   const Mat&               _descriptors_q,
									   vector<KeyPoint>&   _keypoints_t,
									   const Mat&               _descriptors_t,
									   vector<DMatch>&     _matches) const {
	
	cout << "[LabicReconstructor::matchFeatures] matching features\n";
	vector<DMatch> matches;
	
	matcher->match(_descriptors_q, _descriptors_t, matches);
	cout << "[LabicReconstructor::matchFeatures] Inital matched features: " << matches.size() << endl;
		
	_matches.clear();
	
	// Distance filter
	for (int i=0; i<matches.size(); i++) {
		if (matches[i].distance < maxMatchDistance) {
			_matches.push_back(matches[i]);
		}
	}
	
	cout << "[LabicReconstructor::matchFeatures] Final matched features after threshold: " << _matches.size() << endl;
	
}

Mat LabicReconstructor::filterMatches(vector<KeyPoint>&   _keypoints_q,
									  vector<KeyPoint>&   _keypoints_t,
									  vector<DMatch>&     _matches,
									  bool                         _giveID) {

	double RANSACDist = 2.0;
	double RANSACConf = .99;
	vector<Point2d>    imgPoints_q, imgPoints_t;
	
	for (vector<DMatch>::const_iterator it = _matches.begin();
		 it != _matches.end(); it++) {
		imgPoints_q.push_back(_keypoints_q[(*it).queryIdx].pt);
		imgPoints_t.push_back(_keypoints_t[(*it).trainIdx].pt);
	}
	
	// execute RANSAC to detect outliers
	vector<uchar> inliers(imgPoints_q.size(), 0);
	findFundamentalMat(imgPoints_t, imgPoints_q, CV_FM_RANSAC,
					   RANSACDist, RANSACConf, inliers);
	
	// remove outliers
	vector<uchar>::const_iterator itI = inliers.begin();
	vector<DMatch>::iterator  itM = _matches.begin();
	vector<Point2d>::iterator itQ = imgPoints_q.begin();
	vector<Point2d>::iterator itT = imgPoints_t.begin();
	if (_giveID) {
		int count1 = 0;
		int count2 = 0;
		for (; itI != inliers.end(); itI++){
			if (!*itI){
				// remove outlier
				itM = _matches.erase(itM);
				itQ = imgPoints_q.erase(itQ);
				itT = imgPoints_t.erase(itT);
			} else {
				// add ID to inlier
				if (_keypoints_t[itM->trainIdx].class_id == -1) {
					// newly mathced points
					_keypoints_q[itM->queryIdx].class_id = _keypoints_t[itM->trainIdx].class_id = ID++;
					count1++;
				} else if ( _keypoints_t[itM->trainIdx].class_id > -1 ) {
					// training keypoint has already been given ID
					_keypoints_q[itM->queryIdx].class_id = _keypoints_t[itM->trainIdx].class_id;
					count2++;
				}
				itM++;
				itQ++;
				itT++;
			}
		}
		cout << "[LabicReconstructor::matchImages] survived matched features: "
		<< _matches.size() << endl;
		cout << "[LabicReconstructor::matchImages] merged object points: " << count2 << endl;
		cout << "[LabicReconstructor::matchImages] new object points: " << count1 << endl;
	} else {
		for (; itI != inliers.end(); itI++){
			if (!*itI){
				itM = _matches.erase(itM);
				itQ = imgPoints_q.erase(itQ);
				itT = imgPoints_t.erase(itT);
			} else {
				itM++;
				itQ++;
				itT++;
			}
		}
		cout << "[LabicReconstructor::matchImages] survived matched features: "
		<< _matches.size() << endl;
	}
	
	return findFundamentalMat(imgPoints_t, imgPoints_q, CV_FM_8POINT);

};

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
