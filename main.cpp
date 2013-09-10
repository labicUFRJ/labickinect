//
//  LabicKinect
//  Author: Mario Cecchi <macecchi@gmail.com>
//  Laboratorio de Inteligencia Computacional
//  www.labic.nce.ufrj.br
//

#define LABIC_ENABLE_CV      ON
#define LABIC_ENABLE_PCL     ON
#define LABIC_ENABLE_MATCHER ON
#define REFRESH_INTERVAL     1

#include <iostream>
#include <ctime>

#include "LabicKinect.h"

#if LABIC_ENABLE_CV == ON
#include "LabicCV.h"
#endif

#if LABIC_ENABLE_PCL == ON
#include "LabicPCL.h"
#endif

#if LABIC_ENABLE_MATCHER == ON
#include "LabicReconstructor.h"
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
#if LABIC_ENABLE_MATCHER == ON
	LabicReconstructor *recon;
#endif
    clock_t t, t1, t2, t3;
    float timeTotal, timeCV, timePCL, timeReconstructor;
    
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
	
#if LABIC_ENABLE_MATCHER == ON
	recon = new LabicReconstructor(100,500);
#endif
    
	// OpenCV thread
#if LABIC_ENABLE_CV == ON
	cv = new LabicCV(kinect, 640, 480); // TODO const
#if LABIC_ENABLE_MATCHER == ON
	recon->cv = cv;
#endif
    cv->init();
	cv->start();
#endif
	
    // PCL thread
#if LABIC_ENABLE_PCL == ON
	pcl = new LabicPCL(kinect, 640, 480);
#if LABIC_ENABLE_MATCHER == ON
	recon->pcl = pcl;
#endif
	pcl->start();
	//pcl->display();
#endif
    
    // Reconstructor thread
#if LABIC_ENABLE_MATCHER == ON
	recon->start();
#endif
    
    // TODO add ifdef
    while (1) {
        t = clock();
        t1 = clock();
        if (!pcl->mainLoopPart(REFRESH_INTERVAL)) break;
        t1 = clock() - t1;
        t2 = clock();
        if (!cv->mainLoopPart(REFRESH_INTERVAL)) break;
        t2 = clock() - t2;
        t3 = clock();
        if (!recon->mainLoopPart(REFRESH_INTERVAL)) break;
        t3 = clock() - t3;
        
        t = clock() - t;
        
        timeTotal = 1000*((float)t)/CLOCKS_PER_SEC;
        timePCL = 1000*((float)t1)/CLOCKS_PER_SEC;
        timeCV = 1000*((float)t2)/CLOCKS_PER_SEC;
        timeReconstructor = 1000*((float)t3)/CLOCKS_PER_SEC;
        
        cout << "[main] Loop time: "
        << timeTotal << "ms "
        << "CV: " << timeCV << "ms "
        << "PCL: " << timePCL << "ms "
        << "Rec: " << timeReconstructor << "ms " <<
        endl;
        
    }
    
    cout << "[main] Stop requested. Joining threads..." << endl;
	
	// Wait for threads to finish
#if LABIC_ENABLE_CV == ON
	cv->join();
#endif
#if LABIC_ENABLE_PCL == ON
	pcl->join();
#endif
#if LABIC_ENABLE_MATCHER == ON
	recon->join();
#endif
	
    //	frameCatcher.join();
	
    
	cout << "[main] All threads have finished. Closing Kinect..." << endl;
	
    kinect->stopVideo();
	kinect->stopDepth();
    kinect->close();
	
	cout << "[main] Kinect closed." << endl
    << "[main] Finishing main. Bye!" << endl;
    
	return 0;
}