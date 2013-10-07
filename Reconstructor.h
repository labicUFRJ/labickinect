#ifndef __LABICKINECT_RECONSTRUCTOR_H__
#define __LABICKINECT_RECONSTRUCTOR_H__

#include "common.h"
#include "opencv2/features2d/features2d.hpp"
#include "RGBDImage.h"
#include "LabicCV.h"
#include "LabicPCL.h"
#include "RANSACAligner.h"

namespace labic {
	
	class Reconstructor {
	public:
		LabicCV *cv;
		LabicPCL *pcl;

		Reconstructor(bool* _stop);

		void setBadTransformAction(int act) { badTransformAction = act; }

		void start();
		void join();
        void close();

		void performLoop(const cv::Mat& rgbCurrent,
						 const uint16_t* depthCurrent);

		void performLoop(const RGBDImage& rgbdCurrent);

		void printStats() const;

	private:
		static const int DISCARD = 1;
		static const int USE_LAST_TRANSFORM = 2;
        boost::thread m_Thread;
		unsigned int ID;
		unsigned int minFeatures;
		unsigned int maxFeatures;
		unsigned int maxDetectionIte;
		unsigned int minMatches;
		float  		 maxMatchDistance;
		unsigned int minInliersToValidateTransformation;
		unsigned int framesAnalyzed;
		unsigned int reconstructionsGenerated;
		unsigned int reconstructionsAccepted;
		unsigned int featuresExtracted;
		unsigned int featuresMatched;
		unsigned int matchesDiscarded;
		unsigned int pointsDetected;
		int			 badTransformAction;
		bool*	stop;
		clock_t	totalTime;
		RANSACAligner* ransac;
		cv::Ptr<cv::FastAdjuster>         adjuster;
		cv::Ptr<cv::DescriptorMatcher>    matcher;
		cv::Ptr<cv::DescriptorMatcher>    matcher2;
		cv::Ptr<cv::DescriptorExtractor>  extractor;
		pcl::PointCloud<pcl::PointXYZRGB> world, alignedCloudPrevious;
		cv::Mat							  descriptorsPrevious;
		std::vector<cv::KeyPoint>		  featuresPrevious;
		Eigen::Matrix4d 				  transformPrevious;
		RGBDImage						  rgbdPrevious;

		void reconstruct();
        
		void extractRGBFeatures(const RGBDImage&			 rgbd,
								std::vector<cv::KeyPoint>&   keypoints,
								cv::Mat&                     descriptors);

		void matchFeatures(std::vector<cv::KeyPoint>&   _keypoints_q,
						   const cv::Mat&               _descriptors_q,
						   std::vector<cv::KeyPoint>&   _keypoints_t,
						   const cv::Mat&               _descriptors_t,
						   std::vector<cv::DMatch>&     _matches);
		
	};
}

#endif /* __LABICKINECT_RECONSTRUCTOR_H__ */
