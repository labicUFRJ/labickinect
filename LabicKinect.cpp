#include <ctime>
#include <cmath>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "LabicKinect.h"

using namespace std;
using namespace labic;

Kinect::Kinect(freenect_context *_ctx, int _index)
: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_REGISTERED),m_buffer_video(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false),
depthMat(cv::Size(width,height),CV_16UC1), rgbMat(cv::Size(width,height),CV_8UC3,cv::Scalar(0)), ownMat(cv::Size(width,height),CV_8UC3,cv::Scalar(0)) {
    setTilt(0.0);
    setLed(LED_RED);
	depth_buffer = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
}

void Kinect::close() {
    setLed(LED_BLINK_GREEN);
}

void Kinect::VideoCallback(void* _rgb, uint32_t timestamp) {
    m_rgb_mutex.lock();
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    rgbMat.data = rgb;
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
}

void Kinect::DepthCallback(void* _depth, uint32_t timestamp) {
    m_depth_mutex.lock();
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    depth_buffer = depth;
    depthMat.data = (uchar*) depth;
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
}

bool Kinect::getVideo(std::vector<uint8_t> &buffer) {
    m_rgb_mutex.lock();
    if(m_new_rgb_frame) {
        buffer.swap(m_buffer_video);
        m_new_rgb_frame = false;
        m_rgb_mutex.unlock();
        return true;
    } else {
        m_rgb_mutex.unlock();
        return false;
    }
}

bool Kinect::getDepth(uint16_t* buffer) {
    uint16_t* tmp;
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
        tmp = buffer;
        buffer = depth_buffer;
        depth_buffer = tmp;
        m_new_depth_frame = false;
        m_depth_mutex.unlock();
        return true;
    } else {
        m_depth_mutex.unlock();
        return false;
    }
}

bool Kinect::getVideoMat(cv::Mat& output) {
    m_rgb_mutex.lock();
    if(m_new_rgb_frame) {
        cvtColor(rgbMat, output, CV_RGB2BGR);
        m_new_rgb_frame = false;
        m_rgb_mutex.unlock();
        return true;
    } else {
        m_rgb_mutex.unlock();
        return false;
    }
}

bool Kinect::getDepthMat(cv::Mat& output) {
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
        depthMat.copyTo(output);
        m_new_depth_frame = false;
        m_depth_mutex.unlock();
        return true;
    } else {
        m_depth_mutex.unlock();
        return false;
    }
}


bool Kinect::getFrame(cv::Mat &video, uint16_t *depth) {
	if (depth == NULL) {
		cout << "[LabicKinect::getFrame] depth not allocated" << endl;
		return false;
	}

    uint16_t* tmp;
	
    m_rgb_mutex.lock();
    m_depth_mutex.lock();
    
    if (m_new_rgb_frame && m_new_depth_frame) {
        tmp = depth;
		std::copy(depth_buffer, depth_buffer + height*width, depth);
        depth_buffer = tmp;
        cvtColor(rgbMat, video, CV_RGB2BGR);
        m_new_rgb_frame = false;
        m_new_depth_frame = false;
        m_rgb_mutex.unlock();
        m_depth_mutex.unlock();
		return true;
    } else {
        m_rgb_mutex.unlock();
        m_depth_mutex.unlock();
        return false;
    }
    
}

void Kinect::setTilt(double _tilt) {
    tilt = !_tilt ? 0 : tilt+_tilt;
    if (tilt < -30) tilt = -30;
    else if (tilt > 30) tilt = 30;
    setTiltDegrees(tilt);
}

Mutex::Mutex() {
	pthread_mutex_init(&m_mutex, NULL);
}

void Mutex::lock() {
	pthread_mutex_lock(&m_mutex);
}

void Mutex::unlock() {
	pthread_mutex_unlock(&m_mutex);
}
