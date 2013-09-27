//
//  LabicKinect
//  Author: Mario Cecchi <macecchi@gmail.com>
//  Laboratorio de Inteligencia Computacional
//  www.labic.nce.ufrj.br
//

#define LABIC_ENABLE_CV      1
#define LABIC_ENABLE_PCL 	 0
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
	bool stop;
	float timeTotal, timeCV, timePCL, timeReconstructor;

	cout << "[main] Initializing Kinect... ";

	try {
		kinect = &freenect.createDevice<Kinect>(0);

		cout << "Done" << endl << "[main] Starting streams... ";

		kinect->startVideo();
		kinect->startDepth();

		cout << "Done" << endl;
	} catch (runtime_error &e) {
		cout << "Connection error: " << e.what() << endl;
		return 1;
	}

	recon = new LabicReconstructor(&stop);

	// OpenCV thread
	cv = new LabicCV(kinect, &stop, 640, 480); // TODO const
	recon->cv = cv;
#if LABIC_ENABLE_CV
	cv->init();
	cv->start();
#endif

#if LABIC_ENABLE_PCL
    // PCL thread
	pcl = new LabicPCL(kinect, &stop, 640, 480);
	recon->pcl = pcl;
	pcl->start();
	//pcl->display();
#endif
	
    // Reconstructor thread
#if LABIC_ENABLE_MATCHER
	recon->start();
#endif

	while (!stop) {
#if LABIC_ENABLE_PCL
		pcl->mainLoopPart(REFRESH_INTERVAL);
#endif
#if LABIC_ENABLE_CV
		cv->mainLoopPart(REFRESH_INTERVAL);
#endif
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

	cout << "[main] Everything closed. Bye!" << endl;
	
	return 0;
}
