//
//  LabicKinect
//  Author: Mario Cecchi <macecchi@gmail.com>
//  Laboratorio de Inteligencia Computacional
//  www.labic.nce.ufrj.br
//

#define LABIC_ENABLE_CV   OFF
#define LABIC_ENABLE_PCL  ON

#include "LabicKinect.h"

#if LABIC_ENABLE_CV == ON
#include "LabicCV.h"
#endif

#if LABIC_ENABLE_PCL == ON
#include "LabicPCL.h"
#endif

using namespace std;
using namespace labic;

//void kinectLoop(Kinect *kinect, cv::Mat rgbMat, uint16_t *depth) {
//	while (!kinect->stop) {
//		kinect->getFrame(rgbMat, depth);
//	}
//}

int main(int argc, char **argv) {
    Freenect::Freenect freenect;
    Kinect *kinect;
    #if LABIC_ENABLE_CV == ON
	LabicCV *cv;
	#endif
	#if LABIC_ENABLE_PCL == ON
	LabicPCL *pcl;
	#endif
//	cv::Mat rgbMat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));
//    uint16_t *depth = (uint16_t*) malloc(sizeof(uint16_t)*640*480);
	
	cout << "[main] Initializing Kinect device..." << endl;
	
	try {
		kinect = &freenect.createDevice<Kinect>(0);
		
		cout << "[main] Kinect initialized." << endl
			 << "[main] Starting streams..." << endl;
		
		kinect->startVideo();
		kinect->startDepth();
		
		cout << "[main] Streams started." << endl;
	} catch (runtime_error &e) {
		cout << "[main] Kinect ERROR: " << e.what() << endl;
		return 1;
	}
    
//	boost::thread frameCatcher(kinectLoop);
	
	// OpenCV thread
	#if LABIC_ENABLE_CV == ON
	cv = new LabicCV(kinect, 640, 480); // TODO const
    cv->init();
	cv->start();
	#endif
	
    // PCL thread
	#if LABIC_ENABLE_PCL == ON
	pcl = new LabicPCL(kinect, 640, 480);
	//pcl->start();
	pcl->display();
	#endif
    
	// matcher thread
	// TODO
	
	// Wait for threads to finish
	#if LABIC_ENABLE_CV == ON
	cv->join();
	#endif
	//pcl->join();
	
	
//	frameCatcher.join();
	
    
	cout << "[main] All threads have finished. Closing Kinect..." << endl;
	
    kinect->stopVideo();
	kinect->stopDepth();
    kinect->close();
	
	cout << "[main] Kinect closed." << endl
		 << "[main] Finishing main. Bye!" << endl;

	return 0;
}