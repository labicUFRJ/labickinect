#ifndef __LABICKINECT_RECONSTRUCTOR_H__
#define __LABICKINECT_RECONSTRUCTOR_H__

#include "../common.h"
#include "../rgbd_image.h"
#include "../tools/queue.h"
#include "visual_reconstructor.h"

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

        boost::thread m_Thread;
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
		pcl::PointCloud<pcl::PointXYZRGB> world;
		cv::Mat							  descriptorsPrevious;
		std::vector<cv::KeyPoint>		  featuresPrevious;
		Eigen::Matrix4d 				  transformPrevious, transformGlobal;
		RGBDImage						  rgbdPrevious, rgbdCurrent;
		Queue<RGBDImage>& queue;
		VisualReconstructor visualReconstructor;
		
	};
}

#endif /* __LABICKINECT_RECONSTRUCTOR_H__ */
