#include <ctime>
#include <cmath>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "KinectController.h"

using namespace std;
using namespace labic;

KinectController::KinectController(freenect_context *_ctx, int _index)
: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes),
  m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), last_timestamp(0), last_timestamp_grabbed(0),
  m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false) {
    setTilt(0.0);
    setLed(LED_RED);
}

void KinectController::close() {
    setLed(LED_BLINK_GREEN);
}

void KinectController::VideoCallback(void* _rgb, uint32_t timestamp) {
    m_rgb_mutex.lock();
    last_timestamp = timestamp;
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    std::copy(rgb, rgb+width*height*3, m_buffer_video.begin());
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
}

void KinectController::DepthCallback(void* _depth, uint32_t timestamp) {
    m_depth_mutex.lock();
    last_timestamp = timestamp;
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    std::copy(depth, depth+640*480, m_buffer_depth.begin());
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
}

bool KinectController::grabRGBDImage(RGBDImage& rgbd) {
    m_rgb_mutex.lock();
    m_depth_mutex.lock();

    if (m_new_rgb_frame && m_new_depth_frame) {
    	rgbd.update(m_buffer_video, m_buffer_depth, last_timestamp);
    	m_new_depth_frame = false;
    	m_new_rgb_frame = false;
        m_rgb_mutex.unlock();
        m_depth_mutex.unlock();
        last_timestamp_grabbed = last_timestamp;
        return true;
    }

    m_rgb_mutex.unlock();
    m_depth_mutex.unlock();
    return false;
}

void KinectController::setTilt(double _tilt) {
    tilt = !_tilt ? 0 : tilt+_tilt;
    if (tilt < -30) tilt = -30;
    else if (tilt > 30) tilt = 30;
    setTiltDegrees(tilt);
}

