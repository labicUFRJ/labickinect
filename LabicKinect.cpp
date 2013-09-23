#include "LabicKinect.h"

using namespace std;
using namespace labic;

Kinect::Kinect(freenect_context *_ctx, int _index)
: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_REGISTERED),m_buffer_video(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false),
depthMat(cv::Size(640,480),CV_16UC1), rgbMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0)), ownMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0))
{
    tilt = 0;
	stop = false;
    setLed(LED_RED);
	depth_buffer = (uint16_t*) malloc(sizeof(uint16_t)*640*480);
    // ...
}

void Kinect::close() {
    setLed(LED_BLINK_GREEN);
}

void Kinect::VideoCallback(void* _rgb, uint32_t timestamp) {
    m_rgb_mutex.lock();
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
//    copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
    rgbMat.data = rgb;
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
};

void Kinect::DepthCallback(void* _depth, uint32_t timestamp) {
    m_depth_mutex.lock();
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    depth_buffer = depth;
	//std::copy(depth, depth + 480*640, depth_buffer);
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
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
        buffer = depth_buffer;
        //std::copy(depth_buffer, depth_buffer + 480*640, buffer);
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
	
    m_rgb_mutex.lock();
    m_depth_mutex.lock();
    
    if (m_new_rgb_frame && m_new_depth_frame) {
        //depth = (uint16_t*) malloc(sizeof(uint16_t)*640*480);
        //depth = depth_buffer;
		std::copy(depth_buffer, depth_buffer + 480*640, depth);
        cvtColor(rgbMat, video, CV_RGB2BGR);
        m_rgb_mutex.unlock();
        m_depth_mutex.unlock();
		return true;
    } else {
        m_rgb_mutex.unlock();
        m_depth_mutex.unlock();
        return false;
    }
    
}

int Kinect::mmToRaw(float depthValue) const {
    int z = DEPTH_BLANK;
    if (depthValue > 0) {
        z = (int) MAX(0, ((1.0/(depthValue/1000.0)) - 3.3309495161)/(-0.0030711016));
    }
    return z;
}

cv::Point3d Kinect::ptToPoint3d(float cgx, float cgy, float cgz) {
    double fx_d = 1.0 / 5.9421434211923247e+02;
    double fy_d = 1.0 / 5.9104053696870778e+02;
    double cx_d = 3.3930780975300314e+02;
    double cy_d = 2.4273913761751615e+02;
    
    cgz = cgz/1000.0; // cgz has to be in mm
    
    if (cgz == 0 || cgz == DEPTH_BLANK) {
        return cv::Point3d(0,0,0);
    }
        
    return cv::Point3d(
                       (cgx - cx_d) * cgz * fx_d,
                       (cgy - cy_d) * cgz * fy_d,
                       cgz
                       );
}

pcl::PointXYZRGB Kinect::ptToPointXYZRGB(float cgx, float cgy, float cgz) {
    double fx_d = 1.0 / 5.9421434211923247e+02;
    double fy_d = 1.0 / 5.9104053696870778e+02;
    double cx_d = 3.3930780975300314e+02;
    double cy_d = 2.4273913761751615e+02;
    pcl::PointXYZRGB pt;
    
    cgz = cgz/1000.0; // cgz has to be in mm
    
    if (cgz == 0 || cgz == DEPTH_BLANK) {
        pt.x = pt.y = pt.z = 0;
    } else {
        
        pt.x = (cgx - cx_d) * cgz * fx_d;
        pt.y = (cgy - cy_d) * cgz * fy_d;
        pt.z = cgz;
    }
    
    return pt;
}

bool Kinect::frameToPointCloud(const cv::Mat& rgb, const uint16_t* depth, pcl::PointCloud<pcl::PointXYZRGB>& _cloud, const int nThreads, const vector<cv::Point2f> pts) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointXYZRGB pt;
/*    uint16_t *depth;
    cv::Mat rgb(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));*/
    cv::Vec3b ptRGB;
    int i;
    int x, y;
    clock_t t;
    
    cloud.clear();
    cloud.reserve(640*480);
	
    t = clock();
    /*
    boost::thread_group threads;
	
	std::vector<pcl::PointCloud<pcl::PointXYZRGB> > miniclouds(nThreads);
    int pointsPerThread = 640*480/nThreads;
    
    for (i=0; i<nThreads; i++) {
       threads.create_thread(boost::bind(&Kinect::getPointCloudThread, this, miniclouds[i], rgb, depth, i*pointsPerThread, (i+1)*pointsPerThread));
        //getPointCloudThread(miniclouds[i], rgb, depth, i*pointsPerThread, (i+1)*pointsPerThread);
//        std::cout << "Started thread " << i << " from " << i*pointsPerThread << " to " << (i+1)*pointsPerThread << std::endl;
    }
    
    //threads.join_all();
	
	for (i=0; i<nThreads; i++) {
		cloud += miniclouds[i];
		std::cout << "Mergin with miniclouds " << i << " with " << miniclouds[i].points.size() << std::endl;

	}
	
	cloud.width = cloud.points.size();
	cloud.height = 1;

    std::cout << "Total points after join: " << cloud.width << "" << std::endl;
    */
	
    // If no specific points were specified
    if (pts.empty()) {
        for (i=0; i<640*480; i++) {
            y = i/640;
            x = i%640;
            
            // If point has no depth available, skip it
            if (depth[i] == 0) continue;

            pt = ptToPointXYZRGB(x, y, depth[i]);
            ptRGB = rgb.at<cv::Vec3b>(y,x);
            pt.r = ptRGB[2];
            pt.g = ptRGB[1];
            pt.b = ptRGB[0];
            
            cloud.push_back(pt);
        }
    } else {
        for (i=0; i<pts.size(); i++) {
            y = pts[i].y;
            x = pts[i].x;
            int index = y*640 + x;
            
            // If point has no depth available, skip it
            if (depth[i] == 0) continue;
            
            pt = ptToPointXYZRGB(x, y, depth[index]);
            ptRGB = rgb.at<cv::Vec3b>(y,x);
            pt.r = ptRGB[2];
            pt.g = ptRGB[1];
            pt.b = ptRGB[0];
            
            cloud.push_back(pt);
        }
    }
    
    t = clock() - t;
    
    _cloud = cloud;
    //free(depth);
    
//    cout << "[LabicKinect] frameToPointCloud time: " << 1000*((float)t)/CLOCKS_PER_SEC << " ms " << endl;
    
    return true;
    
}

void Kinect::getPointCloudThread(pcl::PointCloud<pcl::PointXYZRGB>& cloud, cv::Mat &rgb, uint16_t *depth, int start, int end) {
//	pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointXYZRGB pt;
    cv::Vec3b ptRGB;
    int x, y;
    int i;
    
    for (i=start; i<end; i++) {
        y = i/640;
        x = i%640;
        
        pt = ptToPointXYZRGB(x, y, depth[i]);
        ptRGB = rgb.at<cv::Vec3b>(y,x);
        pt.r = ptRGB[2];
        pt.g = ptRGB[1];
        pt.b = ptRGB[0];
        
        cloud.push_back(pt);
    }
	
//	_cloud = cloud;
	
	cout << "exiting thread with " << cloud.points.size() << " points" << endl;
}

void Kinect::setTilt(double _tilt) {
    tilt = !_tilt ? 0 : tilt+_tilt;
    if (tilt < -30) tilt = -30;
    else if (tilt > 30) tilt = 30;
    setTiltDegrees(tilt);
}

Mutex::Mutex() {
	pthread_mutex_init( &m_mutex, NULL );
}

void Mutex::lock() {
	pthread_mutex_lock( &m_mutex );
}

void Mutex::unlock() {
	pthread_mutex_unlock( &m_mutex );
}