//
//  kinect.h
//  LabicKinect
//
//  Created by Mario Cecchi on 8/22/13.
//
//

#ifndef __LABICKINECT_KINECTCONTROLLER_H__
#define __LABICKINECT_KINECTCONTROLLER_H__

#include "common.h"
#include "libfreenect.h"
#include <pthread.h>

namespace labic {
	
	class Mutex {
	public:
		Mutex();
		void lock();
		void unlock();
		
		class ScopedLock {
			Mutex & _mutex;
		public:
			ScopedLock(Mutex & mutex) : _mutex(mutex) { _mutex.lock(); }
			~ScopedLock() { _mutex.unlock(); }
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
		double tilt;
        
	private:
		std::vector<uint8_t> m_buffer_depth;
		uint16_t *depth_buffer;
		std::vector<uint8_t> m_buffer_video;
		std::vector<uint16_t> m_gamma;
		Mutex m_rgb_mutex;
		Mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
		cv::Mat depthMat;
		cv::Mat rgbMat;
		cv::Mat ownMat;
	};
}

#endif /* __LABICKINECT_KINECTCONTROLLER_H__ */
