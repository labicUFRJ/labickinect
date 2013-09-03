//
//  cvmatcher.cpp
//  LabicKinect
//
//  Created by Mário Cecchi on 10/06/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"

namespace labic {
	
	class Matcher {
		
	private:
		
		unsigned int ID;
		
		int     minFeatures;
		int     maxFeatures;
		int     maxDetectionIte;
        int     minMatches;
		
		cv::Ptr<cv::FastAdjuster>         adjuster;
		cv::Ptr<cv::DescriptorMatcher>    matcher;
		cv::Ptr<cv::DescriptorMatcher>    matcher2;
		cv::Ptr<cv::DescriptorExtractor>  extractor;
		
		float   KPDistThresh;
		
		double  RANSACDist;
		double  RANSACConf;
		
	public:
		
		Matcher(int _minFeature, int _maxFeature);
		
		unsigned int getID();
		
		void computeFeatures(const cv::Mat&               img,
							 std::vector<cv::KeyPoint>&   keypoints,
							 cv::Mat&                     descriptors);
		
		void matchImages(std::vector<cv::KeyPoint>&   _keypoints_q,
						 const cv::Mat&               _descriptors_q,
						 std::vector<cv::KeyPoint>&   _keypoints_t,
						 const cv::Mat&               _descriptors_t,
						 std::vector<cv::DMatch>&     _matches);
        
        cv::Mat filterMatches(std::vector<cv::KeyPoint>&   _keypoints_q,
							  std::vector<cv::KeyPoint>&   _keypoints_t,
                              std::vector<cv::DMatch>&     _matches,
                              bool                         _giveID);
        
        cv::Mat perform_ransac_alignment(std::vector<cv::KeyPoint>&   _keypoints_q,
										 std::vector<cv::KeyPoint>&   _keypoints_t,
										 std::vector<cv::DMatch>&     _matches);
        
        cv::Mat sba_transform(std::vector<cv::DMatch> _matches);
        
        double transformation_error(cv::Mat transform, cv::DMatch match);
        double transformation_error_set(cv::Mat transform, std::vector<cv::DMatch> matches);
		
		void matchImages2(std::vector<cv::KeyPoint>&   _keypoints_q,
						  const cv::Mat&               _descriptors_q,
						  std::vector<cv::KeyPoint>&   _keypoints_t,
						  const cv::Mat&               _descriptors_t,
						  std::vector<cv::DMatch>&     _matches,
						  cv::Mat&                     _mask);
		
	};
}