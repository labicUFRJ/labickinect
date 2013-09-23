//
//  kinect.h
//  LabicKinect
//
//  Created by Mario Cecchi on 8/22/13.
//
//

#ifndef __LabicKinect__
#define __LabicKinect__

#include "libfreenect.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <pcl/common/common_headers.h>
#include <ctime>
#include <boost/thread.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace labic {
	
	class Mutex {
		
	public:
		Mutex();
		void lock();
		void unlock();
		
		class ScopedLock
		{
			Mutex & _mutex;
		public:
			ScopedLock(Mutex & mutex)
			: _mutex(mutex)
			{
				_mutex.lock();
			}
			~ScopedLock()
			{
				_mutex.unlock();
			}
		};
	private:
		pthread_mutex_t m_mutex;
	};
	
	
	class Kinect : public Freenect::FreenectDevice {
	public:
		Kinect(freenect_context *_ctx, int _index);
		void close();
		void VideoCallback(void* _rgb, uint32_t timestamp);
		void DepthCallback(void* _depth, uint32_t timestamp);
		bool getVideo(std::vector<uint8_t> &buffer);
		bool getDepth(uint16_t* buffer);
		bool getVideoMat(cv::Mat& output);
		bool getDepthMat(cv::Mat& output);
		bool getFrame(cv::Mat &video, uint16_t *depth);
		void setTilt(double _tilt);
		int mmToRaw(float depthValue) const;
		static cv::Point3d ptToPoint3d (float cgx, float cgy, float cgz);
		static pcl::PointXYZRGB ptToPointXYZRGB (float cgx, float cgy, float cgz);
		static bool frameToPointCloud(const cv::Mat& rgb, const uint16_t* depth, pcl::PointCloud<pcl::PointXYZRGB>& _cloud, const std::vector<cv::Point2f> pts = std::vector<cv::Point2f>());
		static void getPointCloudThread(pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat &rgb, uint16_t *depth, int start, int end);
		void teste();
		double tilt;
		bool stop;
		static const int DEPTH_BLANK = 2047;
        
	private:
		std::vector<uint8_t> m_buffer_depth;
		uint16_t *depth_buffer;
		std::vector<uint8_t> m_buffer_video;
		std::vector<uint16_t> m_gamma;
		cv::Mat depthMat;
		cv::Mat rgbMat;
		cv::Mat ownMat;
		Mutex m_rgb_mutex;
		Mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
	};
}

#endif /* __LabicKinect__ */