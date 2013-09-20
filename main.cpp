//
//  LabicKinect
//  Author: Mario Cecchi <macecchi@gmail.com>
//  Laboratorio de Inteligencia Computacional
//  www.labic.nce.ufrj.br
//

#define LABIC_ENABLE_CV      1
#define LABIC_ENABLE_PCL 	 1
#define LABIC_ENABLE_MATCHER 1
#define REFRESH_INTERVAL     1

#include <iostream>
#include <ctime>
#include "LabicKinect.h"
#include "LabicCV.h"
#include "LabicPCL.h"
#include "LabicReconstructor.h"

using namespace std;
using namespace labic;

int main(int argc, char **argv) {
	Freenect::Freenect freenect;
	Kinect *kinect;
	LabicCV *cv;
	LabicPCL *pcl;
	LabicReconstructor *recon;
	clock_t t, t1, t2, t3;
	float timeTotal, timeCV, timePCL, timeReconstructor;
	
	cout << "[main] Initializing Kinect device..." << endl;
	
	try {
		kinect = &freenect.createDevice<Kinect>(0);
		
		cout << "[main] Kinect initialized." << endl
		<< "[main] Starting streams..." << endl;
		
		kinect->startVideo();
		kinect->startDepth();
		
		cout << "[main] Streams started." << endl;
	} catch (runtime_error &e) {
		cout << "[main] Connection error: " << e.what() << endl;
		return 1;
	}
	
	recon = new LabicReconstructor(100,500);
	
	// OpenCV thread
	cv = new LabicCV(kinect, 640, 480); // TODO const
	recon->cv = cv;
#if LABIC_ENABLE_CV
	cv->init();
	cv->start();
#endif
	
    // PCL thread
	pcl = new LabicPCL(kinect, 640, 480);
	recon->pcl = pcl;
#if LABIC_ENABLE_PCL
	pcl->start();
	//pcl->display();
#endif
	
    // Reconstructor thread
#if LABIC_ENABLE_MATCHER
	recon->start();
#endif
	
	while (1) {
		t = clock();
		t1 = clock();
#if LABIC_ENABLE_PCL
		if (!pcl->mainLoopPart(REFRESH_INTERVAL)) break;
#endif
		t1 = clock() - t1;
		t2 = clock();
#if LABIC_ENABLE_CV
		if (!cv->mainLoopPart(REFRESH_INTERVAL)) break;
#endif
		t2 = clock() - t2;
		t3 = clock();
#if LABIC_ENABLE_MATCHER
		if (!recon->mainLoopPart(REFRESH_INTERVAL)) break;
#endif
		t3 = clock() - t3;
		t = clock() - t;
		
		timeTotal = 1000*((float)t)/CLOCKS_PER_SEC;
		timePCL = 1000*((float)t1)/CLOCKS_PER_SEC;
		timeCV = 1000*((float)t2)/CLOCKS_PER_SEC;
		timeReconstructor = 1000*((float)t3)/CLOCKS_PER_SEC;
        /*
		 cout << "[main] Iteration time: "
		 << timeTotal << "ms ("
		 << "CV: " << timeCV << "ms "
		 << "PCL: " << timePCL << "ms "
		 << "Rec: " << timeReconstructor << "ms)" <<
		 endl;*/
		
	}
	
	cout << "[main] Stop requested. Joining threads..." << endl;
	
	// Wait for threads to finish
#if LABIC_ENABLE_CV
	cv->close();
#endif
#if LABIC_ENABLE_PCL
	pcl->close();
#endif
#if LABIC_ENABLE_MATCHER
	recon->close();
#endif
	
	cout << "[main] All threads have finished. Closing Kinect..." << endl;
	
	kinect->stopVideo();
	kinect->stopDepth();
	kinect->close();
	
	cout << "[main] Kinect closed. Bye!" << endl;
	
	return 0;
}
