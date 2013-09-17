#ifndef __LabicReconstructor__
#define __LabicReconstructor__

#include <iostream>
#include <algorithm>
#include <cassert>
#include <boost/thread.hpp>
#include "LabicKinect.h"
#include "LabicCV.h"
#include "LabicPCL.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/io/ply_io.h>

namespace labic {
	
	class LabicReconstructor {
		
	private:
        boost::thread m_Thread;
		unsigned int ID;
		int     minFeatures;
		int     maxFeatures;
		int     maxDetectionIte;
		int     minMatches;
		float   maxMatchDistance;
		double  RANSACDist;
		double  RANSACConf;
		cv::Ptr<cv::FastAdjuster>         adjuster;
		cv::Ptr<cv::DescriptorMatcher>    matcher;
		cv::Ptr<cv::DescriptorMatcher>    matcher2;
		cv::Ptr<cv::DescriptorExtractor>  extractor;
        
		void reconstruct();
        
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
        static double getAlignmentError(const pcl::PointCloud<pcl::PointXYZRGB>& cloud1,
                                        const pcl::PointCloud<pcl::PointXYZRGB>& cloud2,
                                        const std::vector<int> inliersIndexes);
        
        template <typename PointSource, typename PointTarget> void
        estimateRigidTransformationSVD (const pcl::PointCloud<PointSource> &cloud_src,
                                                            const std::vector<int> &indices_src,
                                                            const pcl::PointCloud<PointTarget> &cloud_tgt,
                                                            const std::vector<int> &indices_tgt,
                                                            Eigen::Matrix4f &transformation_matrix);
		
	public:
		LabicCV *cv;
		LabicPCL *pcl;
		
		LabicReconstructor(int _minFeature, int _maxFeature);
		
		void start();
        bool mainLoopPart(const int t);
		void join();
        void close();
		
		void performRansacAlignment(const pcl::PointCloud<pcl::PointXYZRGB>& cloudCurrent,
                                    const pcl::PointCloud<pcl::PointXYZRGB>& cloudPrevious,
                                    std::vector<int>& _inliersIndexes,
                                    Eigen::Matrix4f& _bestTransform);
		
		
		void matchImages2(std::vector<cv::KeyPoint>&   _keypoints_q,
						  const cv::Mat&               _descriptors_q,
						  std::vector<cv::KeyPoint>&   _keypoints_t,
						  const cv::Mat&               _descriptors_t,
						  std::vector<cv::DMatch>&     _matches,
						  cv::Mat&                     _mask);
		
	};
}

#endif /* __LabicReconstructor__ */