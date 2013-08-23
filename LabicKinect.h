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
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <cmath>
#include <vector>

#endif

namespace Labic {

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
	//~MyFreenectDevice(){}
	// Do not call directly even in child
	void VideoCallback(void* _rgb, uint32_t timestamp);
	// Do not call directly even in child
	void DepthCallback(void* _depth, uint32_t timestamp);
	bool getRGB(std::vector<uint8_t> &buffer);
    
	bool getDepth(std::vector<uint8_t> &buffer);
    
private:
	std::vector<uint8_t> m_buffer_depth;
	std::vector<uint8_t> m_buffer_video;
	std::vector<uint16_t> m_gamma;
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;
};
}

