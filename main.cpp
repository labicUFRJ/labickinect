//
//  maintestekinect.cpp
//  LabicKinect
//
//  Created by Mario Cecchi on 8/22/13.
//
//


#include "LabicKinect.h"
#include "LabicCV.h"
#include "LabicPCL.h"

using namespace std;
using namespace Labic;

int main(int argc, char **argv) {
    Freenect::Freenect freenect;
    Kinect *kinect;
    freenect_video_format requested_format(FREENECT_VIDEO_RGB);

    LabicCV *cv;
//	LabicPCL *pcl;
	
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
    
	// opencv thread
	cv = new LabicCV(kinect, 640, 480); // TODO const
    cv->init();
	cv->start();
	
    // pcl thread
//	pcl = new LabicPCL(kinect);
//	pcl->start();
	
    // matcher thread
	// TODO
	
	// wait for threads to finish
	cv->join();
	free(cv);
//	pcl->join();
//	free(pcl);
    
	cout << "[main] All threads have finished. Closing Kinect..." << endl;
	
    kinect->stopVideo();
	kinect->stopDepth();
	
	cout << "[main] Kinect closed." << endl
		 << "[main] Finishing main. Bye!" << endl;

	return 0;
}