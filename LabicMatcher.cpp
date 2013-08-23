//
//  cv_matcher.cpp
//  LabicKinect
//
//  Created by Mário Cecchi on 10/06/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//

#include "LabicMatcher.h"

using namespace Labic;

Matcher::Matcher(int _minFeature, int _maxFeature){
		
		ID = 0;
		
		minFeatures     = _minFeature;
		maxFeatures     = _maxFeature;
		maxDetectionIte = 100;
        minMatches      = 10;
		
		adjuster  = new cv::FastAdjuster(100, true);
//		extractor = new cv::FREAK(true, true, 22, 4);
        extractor = new cv::BriefDescriptorExtractor();
		matcher   = new cv::BFMatcher(cv::NORM_HAMMING, true); // gives infinite distance between matchings
		matcher2  = new cv::BFMatcher(cv::NORM_HAMMING, false);
		
		KPDistThresh = 100;
		
		RANSACDist = 2.0;
		RANSACConf = .99;
		
	};
	
unsigned int Matcher::getID() { return ID; };
	
///////////////////////////////////////////////////////////////////////////////
// compute features of a given image
void Matcher::computeFeatures(const cv::Mat&               img,
                     std::vector<cv::KeyPoint>&   keypoints,
                     cv::Mat&                     descriptors) {
    
    std::cout << "[Labic::Matcher::computeFeatures] computing features" << std::endl;
    
    // detect features
    for (int i=0; i<maxDetectionIte; i++) {
        adjuster->detect(img, keypoints);
        extractor -> compute(img, keypoints, descriptors);
        
        if ( keypoints.size() < minFeatures){
            adjuster->tooFew (minFeatures, keypoints.size());
        } else if ( keypoints.size() > maxFeatures) {
            adjuster->tooMany(maxFeatures, keypoints.size());
        } else {
            std::cout << "[Labic::Matcher::computeFeatures] the number of features: " << keypoints.size() << std::endl
            << "[Labic::Matcher::computeFeatures] target range: " << minFeatures << " to " << maxFeatures
            << ", iteration: " << i << std::endl;
            return;
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
// match images
void Matcher::matchImages(std::vector<cv::KeyPoint>&   _keypoints_q,
                    const cv::Mat&               _descriptors_q,
                    std::vector<cv::KeyPoint>&   _keypoints_t,
                    const cv::Mat&               _descriptors_t,
                    std::vector<cv::DMatch>&     _matches) {
    
    std::cout << "[Labic::Matcher::matchImages] matching features\n";
//    std::vector< std::vector<cv::DMatch> >      matches;
        std::vector<cv::DMatch> matches;
    
    int iter = 0;
//    do {
        if (iter == 10) {
            std::cout << "[Labic::Matcher::matchImages] ERRO MATCH" << std::endl;
//            break;
        }
//        matcher2->knnMatch(_descriptors_q, _descriptors_t, matches, 1);
		matcher2->match(_descriptors_q, _descriptors_t, matches);
        std::cout << "[Labic::Matcher::matchImages] initial matched features: " << matches.size() << std::endl;
        
        iter++;
//    } while (matches.size() >= MIN(_keypoints_q.size(), _keypoints_t.size()));
    
    
    _matches.clear();
    
//    double min_dist = 100; double max_dist = 0;
//    for (int i=0; i<_descriptors_q.rows; i++) {
//        double dist = matches[i].distance;
//        min_dist = MIN(min_dist, dist);
//        max_dist = MAX(max_dist, dist);
//    }
    
    // Distance filter
    for (int i=0; i<matches.size(); i++) {
        if (matches[i].distance < KPDistThresh) {
            _matches.push_back(matches[i]);
        }
    }

//    for (std::vector< std::vector<cv::DMatch> >::const_iterator it = matches.begin();
//         it != matches.end(); it++) {
//        if (it->size() != 0) {
//            if ((*it)[0].distance < KPDistThresh){
//                _matches.push_back((*it)[0]);
//            } //else { std::cout << "distance bigger than threshold " << (*it)[0].distance << "\n"; }
//        }
//    }
    std::cout << "[Labic::Matcher::matchImages] final matched features: " << _matches.size() << std::endl;
    
    if (_matches.size() < minMatches){
        std::cout << "[Labic::Matcher::matchImages] images do not match!" << std::endl;
    }
}

cv::Mat Matcher::perform_ransac_alignment(std::vector<cv::KeyPoint>&   _keypoints_q,
                                          std::vector<cv::KeyPoint>&   _keypoints_t,
                                          std::vector<cv::DMatch>&     _matches) {
    
    // RANSAC initial parameters
    int max_iterations = 1; // k
    int n_samples = 3; // number of maybe_inliers (random samples)
    float max_dist = 3.0; // max error
    
    int iterations = 0;
    int random_indexes[3];
    double best_error = INFINITY, this_error;
    cv::Mat best_model, maybe_model;
    std::vector<cv::DMatch> maybe_inliers(0), other_inliers(0);
    std::vector<cv::DMatch> best_consensus_set(0), consensus_set(0);
    
    srand(time(NULL));
    
    while (iterations < max_iterations) {
        
        // maybe_inliers/initial consensus set
        for (int i=0; i<n_samples; i++) {
            random_indexes[i] = rand() & _matches.size();
            maybe_inliers.push_back(_matches[random_indexes[i]]);
            consensus_set.push_back(_matches[random_indexes[i]]);
        }
        
        // other_inliers
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

cv::Mat Matcher::sba_transform(std::vector<cv::DMatch> _matches) {
    return cv::Mat();
}

double Matcher::transformation_error(cv::Mat transform, cv::DMatch match) {
    return 0.0;
}

double Matcher::transformation_error_set(cv::Mat transform, std::vector<cv::DMatch> matches) {
    return 0.0;
}

cv::Mat Matcher::filterMatches(std::vector<cv::KeyPoint>&   _keypoints_q,
                               std::vector<cv::KeyPoint>&   _keypoints_t,
                               std::vector<cv::DMatch>&     _matches,
                               bool                         _giveID) {
    
    std::vector<cv::Point2d>    imgPoints_q, imgPoints_t;
    
    for (std::vector<cv::DMatch>::const_iterator it = _matches.begin();
         it != _matches.end(); it++) {
        imgPoints_q.push_back(_keypoints_q[(*it).queryIdx].pt);
        imgPoints_t.push_back(_keypoints_t[(*it).trainIdx].pt);
    }
    
    // execute RANSAC to detect outliers
    std::vector<uchar> inliers(imgPoints_q.size(), 0);
    cv::findFundamentalMat(imgPoints_t, imgPoints_q, CV_FM_RANSAC,
                           RANSACDist, RANSACConf, inliers);
    
    // remove outliers
    std::vector<uchar>::const_iterator itI = inliers.begin();
    std::vector<cv::DMatch>::iterator  itM = _matches.begin();
    std::vector<cv::Point2d>::iterator itQ = imgPoints_q.begin();
    std::vector<cv::Point2d>::iterator itT = imgPoints_t.begin();
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
        std::cout << "[Labic::Matcher::matchImages] survived matched features: "
        << _matches.size() << std::endl;
        std::cout << "[Labic::Matcher::matchImages] merged object points: " << count2 << std::endl;
        std::cout << "[Labic::Matcher::matchImages] new object points: " << count1 << std::endl;
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
        std::cout << "[Labic::Matcher::matchImages] survived matched features: "
        << _matches.size() << std::endl;
    }
    
    return cv::findFundamentalMat(imgPoints_t, imgPoints_q, CV_FM_8POINT);
    
};

///////////////////////////////////////////////////////////////////////////////
// match images without symmetric test and RANSAC refinement but with masking
void Matcher::matchImages2(std::vector<cv::KeyPoint>&   _keypoints_q,
                  const cv::Mat&               _descriptors_q,
                  std::vector<cv::KeyPoint>&   _keypoints_t,
                  const cv::Mat&               _descriptors_t,
                  std::vector<cv::DMatch>&     _matches,
                  cv::Mat&                     _mask) {
    
    std::cout << "[Labic::Matcher::matchImages2] matching features\n";
    std::vector< std::vector<cv::DMatch> >      matches1, matches2;
    matcher2->knnMatch(_descriptors_q, _descriptors_t, matches1, 1, _mask.t());
    matcher2->knnMatch(_descriptors_t, _descriptors_q, matches2, 1, _mask);
    
    _matches.clear();
    for (std::vector< std::vector<cv::DMatch> >::const_iterator itM1 = matches1.begin();
         itM1 != matches1.end(); itM1++) {
        for (std::vector< std::vector<cv::DMatch> >::const_iterator itM2 = matches2.begin();
             itM2 != matches2.end(); itM2++) {
            if (itM1->size() != 0 && itM2->size() != 0 ) {
                if ((*itM1)[0].queryIdx == (*itM2)[0].trainIdx &&
                    (*itM2)[0].queryIdx == (*itM1)[0].trainIdx ){
                    if ((*itM1)[0].distance < 2*KPDistThresh){
                        _matches.push_back((*itM1)[0]);
                    }
                }
            }
        }
    }
    
    std::cout << "[Labic::Matcher::matchImages2] matched features: " << _matches.size() << std::endl;
    
    return;
    
};

/*
 void Matcher::projectionMatch(std::vector< cv::Point2d >&  _objProjection,
 cv::Mat&                     _objDescriptors,
 std::vector< cv::KeyPoint >& _imgKeyPoints,
 cv::Mat&                     _imgDescriptors,
 std::vector< cv::DMatch >&   _matches) {
 
 std::cout << "[Labic::Matcher::projectionMatch] matching features\n";
 std::vector< std::vector<cv::DMatch> >      matches;
 
 matcher->knnMatch(_imgDescriptors, _objDescriptors, matches, 1);
 
 _matches.clear();
 std::vector<cv::Point2d>    objPoints, imgPoints;
 for (std::vector< std::vector<cv::DMatch> >::const_iterator it = matches.begin();
 it != matches.end(); it++) {
 if (it->size() != 0) {
 if ((*it)[0].distance < KPDistThresh){
 _matches.push_back((*it)[0]);
 imgPoints.push_back(_imgKeyPoints[(*it)[0].queryIdx].pt);
 objPoints.push_back(_objProjection[(*it)[0].trainIdx]);
 }
 }
 }
 
 std::cout << "[Labic::Matcher::projectionMatch] initial matched features: " << _matches.size() << std::endl;
 
 if (_matches.size() < 8){
 std::cout << "[Labic::Matcher::projectionMatch] tracking failed!" << std::endl;
 return;
 }
 
 // execute RANSAC to detect outliers
 std::vector<uchar> inliers(imgPoints.size(), 0);
 cv::findFundamentalMat(objPoints, imgPoints, CV_FM_RANSAC,
 RANSACDist, RANSACConf, inliers);
 
 // remove outliers
 std::vector<uchar>::const_iterator itI = inliers.begin();
 std::vector<cv::DMatch>::iterator  itM = _matches.begin();
 std::vector<cv::Point2d>::iterator itQ = imgPoints.begin();
 std::vector<cv::Point2d>::iterator itT = objPoints.begin();
 for (; itI != inliers.end(); itI++){
 if (!*itI){
 itM = _matches.erase(itM);
 itQ = imgPoints.erase(itQ);
 itT = objPoints.erase(itT);
 } else {
 itM++;
 itQ++;
 itT++;
 }
 }
 
 // _matches = matches;
 std::cout << "[Labic::Matcher::projectionMatch] survived matched features: " << _matches.size() << std::endl;
 };
 */
