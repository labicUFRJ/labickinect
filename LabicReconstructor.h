#ifndef __LABICKINECT_RECONSTRUCTOR_H__
#define __LABICKINECT_RECONSTRUCTOR_H__

#include "common.h"
#include "opencv2/features2d/features2d.hpp"
#include "LabicCV.h"
#include "LabicPCL.h"
#include "RANSACAligner.h"

namespace labic {
	
	class LabicReconstructor {
	public:
		LabicCV *cv;
		LabicPCL *pcl;

		LabicReconstructor(bool* _stop);

		void start();
		void join();
        void close();

		void performLoop(const cv::Mat& rgbCurrent,
						 const uint16_t* depthCurrent);

	private:
        boost::thread m_Thread;
		unsigned int ID;
		int     minFeatures;
		int     maxFeatures;
		int     maxDetectionIte;
		int     minMatches;
		float   maxMatchDistance;
		int 	minInliersToValidateTransformation;
		int		reconstructionsGenerated;
		int		reconstructionsAccepted;
		bool*	stop;
		RANSACAligner* ransac;
		cv::Ptr<cv::FastAdjuster>         adjuster;
		cv::Ptr<cv::DescriptorMatcher>    matcher;
		cv::Ptr<cv::DescriptorMatcher>    matcher2;
		cv::Ptr<cv::DescriptorExtractor>  extractor;
		pcl::PointCloud<pcl::PointXYZRGB> world, alignedCloudPrevious;
		cv::Mat							  rgbPrevious, descriptorsPrevious;
		uint16_t*						  depthPrevious;
		std::vector<cv::KeyPoint>		  featuresPrevious;
		Eigen::Matrix4d 				  transformPrevious;

		void reconstruct();
        

		void extractRGBFeatures(const cv::Mat&               img,
								const uint16_t* 			 depth,
								std::vector<cv::KeyPoint>&   keypoints,
								cv::Mat&                     descriptors);

		void matchFeatures(std::vector<cv::KeyPoint>&   _keypoints_q,
						   const cv::Mat&               _descriptors_q,
						   std::vector<cv::KeyPoint>&   _keypoints_t,
						   const cv::Mat&               _descriptors_t,
						   std::vector<cv::DMatch>&     _matches) const;
		
	};
}

#endif /* __LABICKINECT_RECONSTRUCTOR_H__ */
