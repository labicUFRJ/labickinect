#include "LabicKinect.h"

using namespace Labic;

Kinect::Kinect(freenect_context *_ctx, int _index)
: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_REGISTERED),m_buffer_video(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false),
depthMat(cv::Size(640,480),CV_16UC1), rgbMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0)), ownMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0))
{
    tilt = 0;
    setLed(LED_RED);
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
    depthMat.data = (uchar*) depth;
//    std::copy(depth2, depth2+640*480*sizeof(uint16_t), m_buffer_depth_f.begin());
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

bool Kinect::getDepth(uint16_t* &buffer) {
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
        //buffer = depth_buffer;
        std::copy(depth_buffer, depth_buffer + 480*640, buffer);
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

int Kinect::mmToRaw(float depthValue) {
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

bool Kinect::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointXYZRGB pt;
    uint16_t *depth;
    cv::Mat rgb(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));
    cv::Vec3b ptRGB;
    int i, x, y;
    
    m_rgb_mutex.lock();
    m_depth_mutex.lock();
    
    if (m_new_rgb_frame && m_new_depth_frame) {
        depth = (uint16_t*) malloc(sizeof(uint16_t)*640*480);
        std::copy(depth_buffer, depth_buffer + 480*640, depth);
        cvtColor(rgbMat, rgb, CV_RGB2BGR);
        m_rgb_mutex.unlock();
        m_depth_mutex.unlock();
    } else {
        m_rgb_mutex.unlock();
        m_depth_mutex.unlock();
        return false;
    }
    
    cloud.clear();
    cloud.width = 640*480;
    cloud.height = 1;
    
    for (i=0; i<640*480; i++) {
        y = i/640;
        x = i%640;
        
        pt = ptToPointXYZRGB(x, y, depth[i]);
        ptRGB = rgb.at<cv::Vec3b>(y,x);
        pt.r = ptRGB[2];
        pt.g = ptRGB[1];
        pt.b = ptRGB[0];
        
        cloud.points.push_back(pt);
    }

    _cloud = cloud;
    free(depth);
    return true;

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

//Freenect::Freenect freenect;
//Kinect* device;
//
//
//GLuint gl_depth_tex;
//GLuint gl_rgb_tex;
//
//
//double freenect_angle(0);
//int got_frames(0),window(0);
//int g_argc;
//char **g_argv;
//
//void DrawGLScene()
//{
//	static std::vector<uint8_t> depth(640*480*4);
//	static std::vector<uint8_t> rgb(640*480*4);
//    
//	// using getTiltDegs() in a closed loop is unstable
//	/*if(device->getState().m_code == TILT_STATUS_STOPPED){
//     freenect_angle = device->getState().getTiltDegs();
//     }*/
//	device->updateState();
//	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, device->getState().getTiltDegs());
//	fflush(stdout);
//    
//	device->getDepth(depth);
//	device->getRGB(rgb);
//    
//	got_frames = 0;
//    
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	glLoadIdentity();
//    
//	glEnable(GL_TEXTURE_2D);
//    
//	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
//	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);
//    
//	glBegin(GL_TRIANGLE_FAN);
//	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
//	glTexCoord2f(0, 0); glVertex3f(0,0,0);
//	glTexCoord2f(1, 0); glVertex3f(640,0,0);
//	glTexCoord2f(1, 1); glVertex3f(640,480,0);
//	glTexCoord2f(0, 1); glVertex3f(0,480,0);
//	glEnd();
//    
//	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
//	if (device->getVideoFormat() == FREENECT_VIDEO_RGB || device->getVideoFormat() == FREENECT_VIDEO_YUV_RGB)
//		glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &rgb[0]);
//	else
//		glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, &rgb[0]);
//    
//	glBegin(GL_TRIANGLE_FAN);
//	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
//	glTexCoord2f(0, 0); glVertex3f(640,0,0);
//	glTexCoord2f(1, 0); glVertex3f(1280,0,0);
//	glTexCoord2f(1, 1); glVertex3f(1280,480,0);
//	glTexCoord2f(0, 1); glVertex3f(640,480,0);
//	glEnd();
//    
//	glutSwapBuffers();
//}
//
//void keyPressed(unsigned char key, int x, int y)
//{
//	if (key == 27) {
//		glutDestroyWindow(window);
//	}
//	if (key == '1') {
//		device->setLed(LED_GREEN);
//	}
//	if (key == '2') {
//		device->setLed(LED_RED);
//	}
//	if (key == '3') {
//		device->setLed(LED_YELLOW);
//	}
//	if (key == '4') {
//		device->setLed(LED_BLINK_GREEN);
//	}
//	if (key == '5') {
//		// 5 is the same as 4
//		device->setLed(LED_BLINK_GREEN);
//	}
//	if (key == '6') {
//		device->setLed(LED_BLINK_RED_YELLOW);
//	}
//	if (key == '0') {
//		device->setLed(LED_OFF);
//	}
//	if (key == 'f') {
//		if (requested_format == FREENECT_VIDEO_IR_8BIT)
//			requested_format = FREENECT_VIDEO_RGB;
//		else if (requested_format == FREENECT_VIDEO_RGB)
//			requested_format = FREENECT_VIDEO_YUV_RGB;
//		else
//			requested_format = FREENECT_VIDEO_IR_8BIT;
//		device->setVideoFormat(requested_format);
//	}
//    
//	if (key == 'w') {
//		freenect_angle++;
//		if (freenect_angle > 30) {
//			freenect_angle = 30;
//		}
//	}
//	if (key == 's' || key == 'd') {
//		freenect_angle = 0;
//	}
//	if (key == 'x') {
//		freenect_angle--;
//		if (freenect_angle < -30) {
//			freenect_angle = -30;
//		}
//	}
//	if (key == 'e') {
//		freenect_angle = 10;
//	}
//	if (key == 'c') {
//		freenect_angle = -10;
//	}
//	device->setTiltDegrees(freenect_angle);
//}
//
//void ReSizeGLScene(int Width, int Height)
//{
//	glViewport(0,0,Width,Height);
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	glOrtho (0, 1280, 480, 0, -1.0f, 1.0f);
//	glMatrixMode(GL_MODELVIEW);
//}
//
//void InitGL(int Width, int Height)
//{
//	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
//	glClearDepth(1.0);
//	glDepthFunc(GL_LESS);
//	glDisable(GL_DEPTH_TEST);
//	glEnable(GL_BLEND);
//	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//	glShadeModel(GL_SMOOTH);
//	glGenTextures(1, &gl_depth_tex);
//	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glGenTextures(1, &gl_rgb_tex);
//	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	ReSizeGLScene(Width, Height);
//}
//
//void *gl_threadfunc(void *arg)
//{
//	printf("GL thread\n");
//    
//	glutInit(&g_argc, g_argv);
//    
//	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
//	glutInitWindowSize(1280, 480);
//	glutInitWindowPosition(0, 0);
//    
//	window = glutCreateWindow("LibFreenect");
//    
//	glutDisplayFunc(&DrawGLScene);
//	glutIdleFunc(&DrawGLScene);
//	glutReshapeFunc(&ReSizeGLScene);
//	glutKeyboardFunc(&keyPressed);
//    
//	InitGL(1280, 480);
//    
//	glutMainLoop();
//    
//	return NULL;
//}
//
////int main(int argc, char **argv) {
////	device = &freenect.createDevice<MyFreenectDevice>(0);
////	device->startVideo();
////	device->startDepth();
////	gl_threadfunc(NULL);
////	device->stopVideo();
////	device->stopDepth();
////	return 0;
////}
