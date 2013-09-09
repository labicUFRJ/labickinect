#ifndef __LabicReconstructor__
#define __LabicReconstructor__

#include <iostream>
#include "LabicCV.h"
#include "LabicPCL.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <pcl/registration/transformation_estimation_svd.h>

namespace labic {
	
	class LabicReconstructor {
		
	private:
		unsigned int ID;
		int     minFeatures;
		int     maxFeatures;
		int     maxDetectionIte;
		int     minMatches;
		float   KPDistThresh;
		double  RANSACDist;
		double  RANSACConf;
		cv::Ptr<cv::FastAdjuster>         adjuster;
		cv::Ptr<cv::DescriptorMatcher>    matcher;
		cv::Ptr<cv::DescriptorMatcher>    matcher2;
		cv::Ptr<cv::DescriptorExtractor>  extractor;
		
		void performLoop(const cv::Mat& rgbCurrent,
						 const cv::Mat& rgbPrevious,
						 const uint16_t* depthCurrent,
						 const uint16_t* depthPrevious);
		void extractRGBFeatures(const cv::Mat&               img,
								std::vector<cv::KeyPoint>&   keypoints,
								cv::Mat&                     descriptors);
		void matchFeatures(std::vector<cv::KeyPoint>&   _keypoints_q,
						   const cv::Mat&               _descriptors_q,
						   std::vector<cv::KeyPoint>&   _keypoints_t,
						   const cv::Mat&               _descriptors_t,
						   std::vector<cv::DMatch>&     _matches) const;
		cv::Mat filterMatches(std::vector<cv::KeyPoint>&   _keypoints_q,
							  std::vector<cv::KeyPoint>&   _keypoints_t,
							  std::vector<cv::DMatch>&     _matches,
							  bool                         _giveID);
		cv::Mat sba_transform(std::vector<cv::DMatch> _matches);
		double transformation_error(cv::Mat transform, cv::DMatch match);
		double transformation_error_set(cv::Mat transform, std::vector<cv::DMatch> matches);
		
	public:
		
		LabicCV *cv;
		LabicPCL *pcl;
		
		LabicReconstructor(int _minFeature, int _maxFeature);
		
		
		cv::Mat performRansacAlignment(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_src,
									   const pcl::PointCloud<pcl::PointXYZRGB>& cloud_tgt,
									   const std::vector<cv::DMatch>&     _matches);
		
		
		void matchImages2(std::vector<cv::KeyPoint>&   _keypoints_q,
						  const cv::Mat&               _descriptors_q,
						  std::vector<cv::KeyPoint>&   _keypoints_t,
						  const cv::Mat&               _descriptors_t,
						  std::vector<cv::DMatch>&     _matches,
						  cv::Mat&                     _mask);
		
	};
}

#endif /* __LabicReconstructor__ */