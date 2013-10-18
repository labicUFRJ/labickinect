//
//  pcl.cpp
//  LabicKinect
//
//  Created by Mario Cecchi on 05/07/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//

#include "LabicPCL.h"
#include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace labic;

LabicPCL::LabicPCL(KinectController *_kinect, bool* _stop, FrameQueue& q)  : kinect(_kinect), stop(_stop), queue(q) {
    viewPort = 1;
    std::cout << "[LabicPCL] Viewer initialized\n";
}

void LabicPCL::display() {
	std::cout << "[LabicPCL] Display started" << std::endl;

	viewer.reset(new PCLVisualizer("PCLVisualizer Thread Test"));
    viewer->setBackgroundColor(0,0,0);

    RGBDImage rgbdCurrent;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr liveCloud;
    PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;

    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -1, 0, -1, 0);
    viewer->addText("Live PointCloud (empty)", 20, 10, "size");

	while (!viewer->wasStopped() && !*stop) {
		if (!kinect->grabRGBDImage(rgbdCurrent)) {
			viewer->spinOnce(100);
			continue;
		}

		liveCloud = rgbdCurrent.pointCloud().makeShared();
		rgb.setInputCloud(liveCloud);

		if (!viewer->updatePointCloud<PointXYZRGB>(liveCloud, rgb)) {
			viewer->addPointCloud<PointXYZRGB>(liveCloud, rgb);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
		}

		std::stringstream txtSize;
		txtSize << "Live PointCloud size: " << liveCloud->size() << " points";

		viewer->updateText(txtSize.str(), 20, 10, "size");

		viewer->spinOnce(100);
	    boost::this_thread::sleep(boost::posix_time::microseconds(100000)); // WHY
	}

	cout << "[LabicPCL] User closed window" << endl;

	viewer->close();
	*stop = true;
	
    std::cout << "[LabicPCL] Display finished\n";
}

void LabicPCL::addCameras(const std::vector<cv::Mat>&         T,
                          const std::vector<cv::Mat>&         R) {
    
    // add camera coordinates
    for (unsigned int i = 0; i < R.size(); i++){
        cv::Mat           Rw, Tw;
        Eigen::Matrix4f   _t;
        Eigen::Affine3f   t;
        
        // optimized camera coordinate
        Rw =  R[i].t();
        Tw = -Rw*T[i];
        
        // _t = [R t; 0 0 0 1]
        _t << Rw.at<double>(0,0), Rw.at<double>(0,1), Rw.at<double>(0,2), Tw.at<double>(0,0),
        Rw.at<double>(1,0), Rw.at<double>(1,1), Rw.at<double>(1,2), Tw.at<double>(1,0),
        Rw.at<double>(2,0), Rw.at<double>(2,1), Rw.at<double>(2,2), Tw.at<double>(2,0),
        0.0, 0.0, 0.0, 1.0;
        
        t = _t;
        std::cout << "addCamera[" << i << "]\nR[i]:\n" << R[i] << "\nT[i]:\n" << T[i] << "\n_t:\n" << _t << "\n\n";
//        viewer.addCoordinateSystem(0.15 + 0.02*i, t, viewPort);
    }
}

void LabicPCL::start() {
    m_Thread = boost::thread(&LabicPCL::display, this);
}

void LabicPCL::join() {
    if (m_Thread.joinable()) m_Thread.join();
}

void LabicPCL::close() {
//    cout << "[LabicPCL] Asked to close PCL" << endl;
    viewer->close();
    join();
}

