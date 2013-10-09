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
#include "Mutex.h"
#include "RGBDImage.h"

namespace labic {
	class KinectController : public Freenect::FreenectDevice {
	public:
		KinectController(freenect_context *_ctx, int _index);
		void close();
		void VideoCallback(void* _rgb, uint32_t timestamp);
		void DepthCallback(void* _depth, uint32_t timestamp);
		bool grabRGBDImage(RGBDImage& rgbd);
		void setTilt(double _tilt);
		double tilt;
        
	private:
		std::vector<uint16_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_video;
		std::vector<uint16_t> raw_depth;
		std::vector<uint8_t> raw_rgb;
		uint32_t last_timestamp;
		uint32_t last_timestamp_grabbed;
		std::vector<uint16_t> m_gamma;
		Mutex m_rgb_mutex;
		Mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
	};
}

#endif /* __LABICKINECT_KINECTCONTROLLER_H__ */
