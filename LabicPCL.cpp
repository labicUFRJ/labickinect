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

LabicPCL::LabicPCL(KinectController *_kinect, bool* _stop)  : kinect(_kinect), stop(_stop) {
    viewPort = 1;
    
    std::cout << "[LabicPCL] Viewer initialized\n";
}

bool LabicPCL::mainLoopPart(const int t) {
    if (!viewer.wasStopped() && !*stop) {
		viewer.spinOnce(t);
        return true;
    } else {
        cout << "[LabicPCL] User closed PCLVisualizer window" << endl;

        close();
        
        cout << "[LabicPCL] mainLoopPart finished" << endl;
        return false;
    }
}

void LabicPCL::display() {
    cv::Mat rgb(cv::Size(640, 480), CV_8UC3, cv::Scalar(0));
    
	std::cout << "[LabicPCL] Display started\n";
    
    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    viewer.setCameraPosition(0.0, 0.0, -1.0, 0.0, -1.0, 0.0);
    viewer.addText("Live PointCloud", 10, 10);    
    
    /*
    while (!viewer.wasStopped() && !*stop) {
        if (!kinect->getFrame(rgb, depth)) continue;
        if (!frameToPointCloud(rgb, depth, liveCloud)) continue;
        
        if (!savedPLY) { pcl::io::savePLYFileASCII("cloud.ply", liveCloud); savedPLY = true; }
        
        if (!viewer.updatePointCloud(liveCloud.makeShared())) {
            viewer.addPointCloud<PointXYZRGB>(liveCloud.makeShared());
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
        }
	}
	*/
    
    viewer.close();
	
    std::cout << "[LabicPCL] Display finished\n";
}

void LabicPCL::generateDepthCloud(uint16_t *depth) {
    int x, y, i;
    PointXYZRGB pt;
    
    cloud.clear();
    cloud.width = width*height;
    cloud.height = 1;
    
    for (i=0; i<width*height; i++) {
        y = i/width;
        x = i%width;
        
        pt = ptToPointXYZRGB(x, y, depth[i]);
        pt.r = pt.g = pt.b = 255;
        
        cloud.points.push_back(pt);
    }
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

void LabicPCL::updateCloud(std::vector< cv::Point3d >& objPoints, cv::Mat img) {
	if (!cloud.empty()) cloud.clear();
	
    cloud.width    = objPoints.size();
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    
    int imgx, imgy;
    
    for (size_t i = 0; i < cloud.points.size (); ++i){
        // Set cloud point position
        cloud.points[i].x = objPoints[i].x;
        cloud.points[i].y = objPoints[i].y;
        cloud.points[i].z = objPoints[i].z - 1.0;
        
        // Set cloud point color
        // Color the point as black if there is no depth information
        if (objPoints[i].x == 0 && objPoints[i].y == 0 && objPoints[i].z == 0) {
            
            cloud.points[i].r = cloud.points[i].g = cloud.points[i].b = 255;
            
        } else {
        
            imgx = i/640;
            imgy = i%640;
            
            cv::Vec3b cor = img.at<cv::Vec3b>(imgx, imgy);
            
            cloud.points[i].r = cor.val[2];
            cloud.points[i].g = cor.val[1];
            cloud.points[i].b = cor.val[0];
        }        
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
    *stop = true;
    viewer.close();
    join();
}

