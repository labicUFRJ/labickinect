#include "LabicReconstructor.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

LabicReconstructor::LabicReconstructor(int _minFeature, int _maxFeature){
		
		ID = 0;
		
		minFeatures     = _minFeature;
		maxFeatures     = _maxFeature;
		maxDetectionIte = 100;
        minMatches      = 10;
		
		adjuster  = new FastAdjuster(100, true);
        extractor = new BriefDescriptorExtractor();
		matcher   = new BFMatcher(NORM_HAMMING, true); // gives infinite distance between matchings
		matcher2  = new BFMatcher(NORM_HAMMING, false);
		
		KPDistThresh = 100;
		
		RANSACDist = 2.0;
		RANSACConf = .99;
		
	};
	
unsigned int LabicReconstructor::getID() { return ID; };

void LabicReconstructor::extractRGBFeatures(const Mat& img, vector<KeyPoint>& keypoints, Mat& descriptors) {
    
    cout << "[Labic::LabicReconstructor::computeFeatures] computing features" << endl;
    
    // detect features
    for (int i=0; i<maxDetectionIte; i++) {
        adjuster->detect(img, keypoints);
        extractor -> compute(img, keypoints, descriptors);
        
        if ( keypoints.size() < minFeatures){
            adjuster->tooFew (minFeatures, keypoints.size());
        } else if ( keypoints.size() > maxFeatures) {
            adjuster->tooMany(maxFeatures, keypoints.size());
        } else {
            cout << "[Labic::LabicReconstructor::computeFeatures] the number of features: " << keypoints.size() << endl
            << "[Labic::LabicReconstructor::computeFeatures] target range: " << minFeatures << " to " << maxFeatures
            << ", iteration: " << i << endl;
            return;
        }
    }
};

void LabicReconstructor::matchFeatures(vector<KeyPoint>&   _keypoints_q,
                    const Mat&               _descriptors_q,
                    vector<KeyPoint>&   _keypoints_t,
                    const Mat&               _descriptors_t,
                    vector<DMatch>&     _matches) {
    
    cout << "[Labic::LabicReconstructor::matchImages] matching features\n";
    vector<DMatch> matches;
    
    int iter = 0;
        if (iter == 10) {
            cout << "[Labic::LabicReconstructor::matchImages] ERRO MATCH" << endl;
        }
		matcher2->match(_descriptors_q, _descriptors_t, matches);
        cout << "[Labic::LabicReconstructor::matchImages] initial matched features: " << matches.size() << endl;
        
        iter++;
    
    _matches.clear();
        
    // Distance filter
    for (int i=0; i<matches.size(); i++) {
        if (matches[i].distance < KPDistThresh) {
            _matches.push_back(matches[i]);
        }
    }

    cout << "[Labic::LabicReconstructor::matchImages] final matched features: " << _matches.size() << endl;
    
    if (_matches.size() < minMatches){
        cout << "[Labic::LabicReconstructor::matchImages] images do not match!" << endl;
    }
}

Mat LabicReconstructor::perform_ransac_alignment(const PointCloud<PointXYZRGB>& cloud_src,
                                                 const PointCloud<PointXYZRGB>& cloud_tgt,
                                                 const vector<DMatch>& _matches) {
    
    // RANSAC initial parameters
    int max_iterations = 1; // k
    int n_samples = 3; // number of maybe_inliers (random samples)
    float max_dist = 3.0; // max error
    
    int iterations = 0;
    int random_indexes[3];
    double best_error = INFINITY, this_error;
    Mat best_model, maybe_model;
    vector<DMatch> maybe_inliers(0), other_inliers(0);
    vector<DMatch> best_consensus_set(0), consensus_set(0);
    
    srand(time(NULL));
    
    while (iterations < max_iterations) {
        
        // maybe_inliers/initial consensus set
        for (int i=0; i<n_samples; i++) {
            random_indexes[i] = rand() & _matches.size();
            maybe_inliers.push_back(_matches[random_indexes[i]]);
            consensus_set.push_back(_matches[random_indexes[i]]);
        }
        
        // other_inliers
        // the complexity of this loop can be improved?
        for (int i=0; i<_matches.size(); i++) {
            if (i != random_indexes[0] && i != random_indexes[1] && i != random_indexes[2]) {
                other_inliers.push_back(_matches[i]);
            }
        }
        
        maybe_model = sba_transform(maybe_inliers);
        
        for (int i=0; i<other_inliers.size(); i++) {
            if (transformation_error(maybe_model, other_inliers[i]) < max_dist) {
                consensus_set.push_back(other_inliers[i]);
            }
        }
        
        if (consensus_set.size() > minMatches) {
            // recalculate model
            maybe_model = sba_transform(consensus_set);
            this_error = transformation_error_set(maybe_model, consensus_set);
            
            if (this_error < best_error) {
                best_model = maybe_model;
                best_error = this_error;
                best_consensus_set = consensus_set;
            }
        }
        
        // TODO test to stop if found error < ok_error
        
        iterations++;
    }
    
    return best_model;
}

Mat LabicReconstructor::sba_transform(vector<DMatch> _matches) {
    return Mat();
}

double LabicReconstructor::transformation_error(Mat transform, DMatch match) {
    return 0.0;
}

double LabicReconstructor::transformation_error_set(Mat transform, vector<DMatch> matches) {
    return 0.0;
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
        cout << "[Labic::LabicReconstructor::matchImages] survived matched features: "
        << _matches.size() << endl;
        cout << "[Labic::LabicReconstructor::matchImages] merged object points: " << count2 << endl;
        cout << "[Labic::LabicReconstructor::matchImages] new object points: " << count1 << endl;
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
        cout << "[Labic::LabicReconstructor::matchImages] survived matched features: "
        << _matches.size() << endl;
    }
    
    return findFundamentalMat(imgPoints_t, imgPoints_q, CV_FM_8POINT);
    
};
