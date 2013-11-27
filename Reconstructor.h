#ifndef __LABICKINECT_RECONSTRUCTOR_H__
#define __LABICKINECT_RECONSTRUCTOR_H__

#include "common.h"
#include "opencv2/features2d/features2d.hpp"
#include "RGBDImage.h"
#include "queue.h"
#include "RANSACAligner.h"

namespace labic {
	
	class Reconstructor {
	public:
		Reconstructor(bool* _stop, Queue<RGBDImage>& q);
		void start() { m_Thread = boost::thread(&Reconstructor::threadFunc, this); }
		void join() { m_Thread.join(); }
        void close() { join(); }
		void performAlignment();
		void printStats() const;

	private:
		void threadFunc();

		void extractRGBFeatures(const RGBDImage&			 rgbd,
								std::vector<cv::KeyPoint>&   keypoints,
								cv::Mat&                     descriptors);

		void matchFeatures(std::vector<cv::KeyPoint>&   _keypoints_q,
						   const cv::Mat&               _descriptors_q,
						   std::vector<cv::KeyPoint>&   _keypoints_t,
						   const cv::Mat&               _descriptors_t,
						   std::vector<cv::DMatch>&     _matches);
        boost::thread m_Thread;
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
		bool*		 stop;
		bool		 autoSave;
		double	     totalTime;
		double		 lastError;
		double		 totalError;
		RANSACAligner* ransac;
		cv::Ptr<cv::FastAdjuster>         adjuster;
		cv::Ptr<cv::DescriptorMatcher>    matcher;
		cv::Ptr<cv::DescriptorMatcher>    matcher2;
		cv::Ptr<cv::DescriptorExtractor>  extractor;
		pcl::PointCloud<pcl::PointXYZRGB> world;
		cv::Mat							  descriptorsPrevious;
		std::vector<cv::KeyPoint>		  featuresPrevious;
		Eigen::Matrix4d 				  transformPrevious, transformFinal;
		RGBDImage						  rgbdPrevious, rgbdCurrent;
		Queue<RGBDImage>& queue;
		
	};
}

#endif /* __LABICKINECT_RECONSTRUCTOR_H__ */
