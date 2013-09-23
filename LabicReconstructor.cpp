#include "LabicReconstructor.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

LabicReconstructor::LabicReconstructor(int _minFeature, int _maxFeature) {
	
	ID = 0;
	
	minFeatures     = _minFeature;
	maxFeatures     = _maxFeature;
	maxDetectionIte = 100;
	minMatches      = 10;
	maxMatchDistance = 5;
	
	adjuster  = new FastAdjuster(100, true);
	extractor = new BriefDescriptorExtractor();
	matcher   = new BFMatcher(NORM_HAMMING, true); // gives infinite distance between matchings
	matcher2  = new BFMatcher(NORM_HAMMING, false);
	
	RANSACDist = 2.0;
	RANSACConf = .99;
};

bool LabicReconstructor::mainLoopPart(const int t) {
    if (cv->isReady()) {
        cout << endl << "[LabicReconstructor] Reconstructor got frames. Reconstructing..." << endl;
        
        performLoop(cv->rgbCurrent, cv->rgbPrevious, cv->depthCurrent, cv->depthPrevious);
        
        cout << "[LabicReconstructor] Finished reconstruction loop" << endl << endl;
        
        cv->restartState();
    }
    
    return true;
}

void LabicReconstructor::reconstruct() {
    cout << "[LabicReconstructor] Reconstructor initialized" << endl;
    /*
    while (!cv->isReady()) {
        // wait...
    }*/
    
    //cout << "[LabicReconstructor] Ready! Starting reconstructor" << endl;
}

void LabicReconstructor::performLoop(const Mat& rgbCurrent,
								  const Mat& rgbPrevious,
								  const uint16_t* depthCurrent,
								  const uint16_t* depthPrevious) {
	
	vector<KeyPoint> featuresCurrent, featuresPrevious;
	Mat descriptorsCurrent, descriptorsPrevious;
	vector<DMatch> relatedFeatures;
    vector<Point2f> featurePointsCurrent, featurePointsPrevious;
	PointCloud<PointXYZRGB> cloudCurrent, cloudPrevious, featureCloudCurrent, featureCloudPrevious, transformedCloudCurrent;
	pcl::registration::TransformationEstimationSVD<PointXYZRGB, PointXYZRGB, float> estimator;
	Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
    vector<int> consensusSetIndexes;
    pcl::visualization::PCLVisualizer vissvd;

    // int v1(0);
    // int v2(0);

    // vissvd.createViewPort(0.0, 0.0, -1.0, 0.0, v1);
    // vissvd.createViewPort(0.0, 0.0, -1.0, 0.0, v2);
    // vissvd.addText("Viewport 0 previous", 10, 10, "v0 text", v1);
    // vissvd.addText("Viewport 1 current", 10, 10, "v1 text", v2);

    Kinect::frameToPointCloud(rgbPrevious, depthPrevious, cloudPrevious);
    Kinect::frameToPointCloud(rgbCurrent, depthCurrent, cloudCurrent);
    // vissvd.addPointCloud(cloudPrevious.makeShared(), "previous", v1);
    // vissvd.addPointCloud(cloudCurrent.makeShared(), "current", v2);


    pcl::io::savePLYFileASCII("cloudPrevious.ply", cloudPrevious);
    pcl::io::savePLYFileASCII("cloudCurrent.ply", cloudCurrent);

    // vissvd.spin();

	// 1. Extract features from both images
	extractRGBFeatures(rgbPrevious, depthPrevious, featuresPrevious, descriptorsPrevious);
	extractRGBFeatures(rgbCurrent, depthCurrent, featuresCurrent, descriptorsCurrent);
	
	// 2. Get relationship (matches) between features from both images
	matchFeatures(featuresPrevious, descriptorsPrevious, featuresCurrent, descriptorsCurrent, relatedFeatures);
	if (relatedFeatures.size() < minMatches) {
		cout << "[LabicReconstructor::performLoop] IMAGES DO NOT MATCH! ABORTING RECONSTRUCTION" << endl;
		return;
	}
    LabicCV::showMatchesPreview(rgbPrevious, featuresPrevious, rgbCurrent, featuresCurrent, relatedFeatures);
    featurePointsPrevious.reserve(relatedFeatures.size());
    featurePointsCurrent.reserve(relatedFeatures.size());
    for (int i=0; i<relatedFeatures.size(); i++) {
        int previousIndex = relatedFeatures[i].trainIdx;
        int currentIndex = relatedFeatures[i].queryIdx;
        Point2f previousPoint = featuresPrevious[previousIndex].pt;
        Point2f currentPoint = featuresCurrent[currentIndex].pt;
        // Discard matches that do not have depth information
        if (depthPrevious[(int)(640*previousPoint.y + previousPoint.x)] > 0 &&
        	depthCurrent[(int)(640*currentPoint.y + currentPoint.x)] > 0) {
        	featurePointsPrevious.push_back(previousPoint);
        	featurePointsCurrent.push_back(currentPoint);
        }
    }
	
    cout << "[LabicReconstructor::performLoop] featurePointsPrevious: " << featurePointsPrevious << " points" << endl
    << "[LabicReconstructor::performLoop] featurePointsCurrent: " << featurePointsCurrent << " points" << endl;
    
	// 3. Generate PointClouds of related features (pointcloudsrc, pointcloudtgt)
    Kinect::frameToPointCloud(rgbPrevious, depthPrevious, featureCloudPrevious, featurePointsPrevious);
    Kinect::frameToPointCloud(rgbCurrent, depthCurrent, featureCloudCurrent, featurePointsCurrent);
    
    pcl::io::savePLYFileASCII("featureCloudPrevious.ply", featureCloudPrevious);
    pcl::io::savePLYFileASCII("featureCloudCurrent.ply", featureCloudCurrent);
	
	// 4. Alignment detection
    cout << "[LabicReconstructor::performLoop] transformation matrix before estimate:" << endl << transform << endl;

	//estimator.estimateRigidTransformation(featureCloudPrevious, featureCloudCurrent, transform);
    performRansacAlignment(featureCloudPrevious, featureCloudCurrent, consensusSetIndexes, transform);
	
    cout << "[LabicReconstructor::performLoop] final transformation matrix:" << endl << transform << endl;
	
	// 5. Apply transformation to all frame points
    transformPointCloud(cloudCurrent, transformedCloudCurrent, transform);
    pcl::io::savePLYFileASCII("transformedCloudCurrent.ply", transformedCloudCurrent);
    
    vissvd.addCoordinateSystem(0.1);
    vissvd.initCameraParameters();
    vissvd.setCameraPosition(0.0, 0.0, -1.0, 0.0, -1.0, 0.0);
    vissvd.addPointCloud(cloudPrevious.makeShared());
    vissvd.addPointCloud(transformedCloudCurrent.makeShared());
    
    cout << "[LabicReconstructor::performLoop] Displaying total of " << cloudPrevious.size() + transformedCloudCurrent.size() << " points" << endl;


    vissvd.spin();
    
	// return updated world pointcloud
	
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

void LabicReconstructor::performRansacAlignment(const PointCloud<PointXYZRGB>& cloudCurrent,
											   const PointCloud<PointXYZRGB>& cloudPrevious,
											   vector<int>& _inliersIndexes,
                                               Eigen::Matrix4f& _bestTransform) {
	
	// RANSAC initial parameters
	int maxIterations = 100; // k
	int nSamples = 3; // number of maybe_inliers (random samples)
	float threshold = 30.0; // max error
    int minInliers = 5;
	
	double bestError = INFINITY, thisError;
    Eigen::Matrix4f bestTransform, maybeTransform, thisTransform;
	vector<int> maybeIndexes, notMaybeIndexes, bestConsensusSetIndexes, consensusSetIndexes;
    
    pcl::registration::TransformationEstimationSVD<PointXYZRGB, PointXYZRGB, float> estimator;
//    pcl::registration::TransformationEstimationLM<PointXYZRGB, PointXYZRGB, float> estimator;
	
	srand(time(NULL));
	bestTransform.setIdentity();

    int iterations = 0;

	while (iterations < maxIterations) {
        cout << ">>> RANSAC iteration " << iterations+1 << endl;
        
        maybeIndexes.clear();
        notMaybeIndexes.clear();
        consensusSetIndexes.clear();
        maybeTransform = Eigen::Matrix4f::Zero();
		
		// Determine random sample (maybe)
        cout << "       maybeIndexes/consensusSetIndexes = [";
		for (int i=0; i<nSamples; ) {
            int randomSample = rand() % cloudCurrent.size();
            cout << randomSample << ", ";
            if (find(maybeIndexes.begin(), maybeIndexes.end(), randomSample) == maybeIndexes.end()) {
                maybeIndexes.push_back(randomSample);
                consensusSetIndexes.push_back(randomSample);
                i++;
            }
		}
        cout << "]" << endl;
        
        // Estimate transformation from maybe set (size = nSamples)
        estimator.estimateRigidTransformation(cloudCurrent, maybeIndexes, cloudPrevious, maybeIndexes, maybeTransform);
        //estimateRigidTransformationSVD(cloudCurrent, maybeIndexes, cloudPrevious, maybeIndexes, maybeTransform);
        
        cout << "       maybeTransform = " << endl << maybeTransform << endl;
        
        // Create vector of points that are not in maybeInliers
        // notMaybeIndexes = cloudCurrent \ maybeIndexes
        cout << "       notMaybeIndexes = [";
        for (int i=0; i<cloudCurrent.size(); i++) {
            bool isInMaybeIndexes = false;
            for (int j=0; j<maybeIndexes.size(); j++) {
                if (i == maybeIndexes[j]) {
                    isInMaybeIndexes = true;
                    break;
                }
            }
            if (isInMaybeIndexes) continue;
            cout << i << ", ";
            notMaybeIndexes.push_back(i);
        }
        cout << "]" << endl;

		// Test transformation with other points that are not in maybeIndexes
		for (int i=0; i<notMaybeIndexes.size(); i++) {
            int pointIndex = notMaybeIndexes[i];
            PointCloud<PointXYZRGB> transformedPoint;
            transformedPoint.push_back(cloudCurrent.points[pointIndex]);
            transformPointCloud(transformedPoint, transformedPoint, maybeTransform);
            double transformedDistance = euclideanDistance(transformedPoint.points[0], cloudPrevious.points[pointIndex]);
            
            cout << "       Point " << i << " distance = " << transformedDistance;
			if (transformedDistance <= threshold) {
				consensusSetIndexes.push_back(pointIndex);
                cout << " (added to consensus set!)";
			}
            cout << endl;
		}
		
        cout << "       consensusSet has " << consensusSetIndexes.size() << " points" << endl;
		if (consensusSetIndexes.size() >= minInliers) {
            cout << "           (ok! we may have found a good transformation. comparing to the best..." << endl;
            
			// Recalculate transformation from new consensus set
            estimator.estimateRigidTransformation(cloudPrevious, consensusSetIndexes, cloudCurrent, consensusSetIndexes, thisTransform);
            //estimateRigidTransformationSVD(cloudPrevious, consensusSetIndexes, cloudCurrent, consensusSetIndexes, thisTransform);
            
            cout << "       thisTransform = " << endl << thisTransform << endl;
            
            // Generate the transformed cloud using thisTransform
            // Note that this cloud will include points that are not in consensus set, be careful
            PointCloud<PointXYZRGB> transformedCloudCurrent;
            transformPointCloud(cloudCurrent, transformedCloudCurrent, thisTransform);
            thisError = getAlignmentError(transformedCloudCurrent, cloudPrevious, consensusSetIndexes);
			
            cout << "       thisError = " << thisError;
			if (thisError < bestError) {
                cout << " (great! best error so far. updating best parameters)";
				bestTransform = thisTransform;
				bestError = thisError;
				bestConsensusSetIndexes = consensusSetIndexes;
			}
            cout << endl;
		}
		
		// TODO test to stop if found error < ok_error
		cout << endl;
		iterations++;
	}
	
    _inliersIndexes = bestConsensusSetIndexes;
    _bestTransform = bestTransform;

}

double LabicReconstructor::getAlignmentError(const PointCloud<PointXYZRGB>& cloud1,
                                             const PointCloud<PointXYZRGB>& cloud2,
                                             const vector<int> inliersIndexes) {
    // Check if two clouds have the same size
    assert(cloud1.size() == cloud2.size());
    double error = 0;
    
    for (int i=0; i<inliersIndexes.size(); i++) {
        int inlierIndex = inliersIndexes[i];
        
        error += squaredEuclideanDistance(cloud1.points[inlierIndex], cloud2.points[inlierIndex]);
    }
    
    error /= inliersIndexes.size();
    return error;
}

Mat LabicReconstructor::filterMatches(vector<KeyPoint>&   _keypoints_q,
									  vector<KeyPoint>&   _keypoints_t,
									  vector<DMatch>&     _matches,
									  bool                         _giveID) {
	
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
/*
template <typename PointSource, typename PointTarget> void
LabicReconstructor::estimateRigidTransformationSVD (const pcl::PointCloud<PointSource> &cloud_src,
                                     const std::vector<int> &indices_src,
                                     const pcl::PointCloud<PointTarget> &cloud_tgt,
                                     const std::vector<int> &indices_tgt,
                                     Eigen::Matrix4f &transformation_matrix)
{
    if (indices_src.size () != indices_tgt.size ())
    {
		cout << "[LabicReconstructor::estimateRigidTransformationSVD] Number or points in source (%lu) differs than target (%lu)!" << endl;
        //PCL_ERROR ("[pcl::estimateRigidTransformationSVD] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
        return;
    }
    
    // <cloud_src,cloud_src> is the source dataset
    transformation_matrix.setIdentity ();
    
    Eigen::Vector4f centroid_src, centroid_tgt;
    // Estimate the centroids of source, target
    compute3DCentroid (cloud_src, indices_src, centroid_src);
    compute3DCentroid (cloud_tgt, indices_tgt, centroid_tgt);
    
    // Subtract the centroids from source, target
    Eigen::MatrixXf cloud_src_demean;
    demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);
    
    Eigen::MatrixXf cloud_tgt_demean;
    demeanPointCloud (cloud_tgt, indices_tgt, centroid_tgt, cloud_tgt_demean);
    
    // Assemble the correlation matrix H = source * target'
    Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();
    
    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f u = svd.matrixU ();
    Eigen::Matrix3f v = svd.matrixV ();
    
    // Compute R = V * U'
    if (u.determinant () * v.determinant () < 0)
    {
        for (int x = 0; x < 3; ++x)
            v (x, 2) *= -1;
    }
    
    Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose ();
    
    // Return the correct transformation
    transformation_matrix.topLeftCorner(3, 3) = R;
    Eigen::Vector3f Rc = R * centroid_src.head(3);
   // const Eigen::Matrix<pcl::Scalar, 3, 1> Rc (R * centroid_src.head(3));
    transformation_matrix.block(0, 3, 3, 1) = centroid_tgt.head(3) - Rc;
}
*/