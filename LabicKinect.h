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
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <pcl/common/common_headers.h>
#include <ctime>
#include <boost/thread.hpp>

#ifndef BOOST_THREAD_INCLUDED
#define BOOST_THREAD_INCLUDED
#include <boost/thread.hpp>
#endif

#endif

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
		bool getDepth(uint16_t* &buffer);
		bool getVideoMat(cv::Mat& output);
		bool getDepthMat(cv::Mat& output);
		bool getFrame(cv::Mat &video, uint16_t *depth);
		void setTilt(double _tilt);
		int mmToRaw(float depthValue);
		cv::Point3d ptToPoint3d (float cgx, float cgy, float cgz);
		pcl::PointXYZRGB ptToPointXYZRGB (float cgx, float cgy, float cgz);
		bool getPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const int nThreads);
		void getPointCloudThread(pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat &rgb, uint16_t *depth, int start, int end);
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

