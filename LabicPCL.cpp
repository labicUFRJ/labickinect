//
//  pcl.cpp
//  LabicKinect
//
//  Created by Mario Cecchi on 05/07/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//

#include "LabicPCL.h"

using namespace Labic;

LabicPCL::LabicPCL(Kinect *_kinect) {
    std::cout << "[PCLViewer] Initializing viewer\n";
    
    this->kinect = _kinect;
	
//    viewer.setWindowName("PCL Visualizer");
    viewPort = 1;
    
    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, viewPort);
    viewer.setBackgroundColor(0, 0, 0, viewPort);
    // viewer.addCoordinateSystem (0.3);
    viewer.initCameraParameters ();
    viewer.setCameraPosition(0.0, 0.0, -5.0, 0.0, -1.0, 0.0, viewPort);
   // viewer.addPointCloud<pcl::PointXYZRGB> (cloud.makeShared(), "points", viewPort);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points", viewPort);
    
    std::cout << "[PCLViewer] Viewer initialized!\n";
    
}

void LabicPCL::addCameras(const std::vector<cv::Mat>&         T,
                           const std::vector<cv::Mat>&         R) {
    
    // add camera coordinates
    for (int i = 0; i < R.size(); i++){
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
        viewer.addCoordinateSystem(0.15 + 0.02*i, t, viewPort);
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

void LabicPCL::display() {
	std::cout << "[PCLViewer] Entering display()\n";
//    viewer.addCoordinateSystem(0.05, viewPort);
	if (cloud.empty()) {
		std::cout <<"ERROR EMPTY\n";return;
	}
	


    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud.makeShared());
    if (!viewer.updatePointCloud<pcl::PointXYZRGB> (cloud.makeShared(), color, "points")) {
		viewer.addPointCloud<pcl::PointXYZRGB> (cloud.makeShared(), color, "points");
	}
	
//	viewer.spinOnce(100);
	viewer.spin();
    
//	while (!viewer.wasStopped()) {
//		viewer.spin();
//
//	}
	
	std::cout << "[PCLViewer] Exiting display()\n";
}


// legacy functions

void show_pcl() {
    std::cout << "INSIDE SHOW PCL!!!\n";
    // point cloud and viewer
    pcl::visualization::PCLVisualizer         viewer("Points");
    pcl::PointCloud<pcl::PointXYZRGB>         cloud;
    int viewPort = 1;
    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, viewPort);
    viewer.setBackgroundColor(0, 0, 0, viewPort);
    // viewer.addCoordinateSystem (0.3);
    viewer.initCameraParameters ();
    viewer.setCameraPosition(0.0, 0.0, -5.0, 0.0, -1.0, 0.0, viewPort);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud.makeShared(), "points", viewPort);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points", viewPort);
    
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud.makeShared());
    viewer.updatePointCloud<pcl::PointXYZRGB> (cloud.makeShared(), color, "points");
    //    addCameras(T, R, viewer, viewPort);
    // viewer.resetCamera();
    
    // write cloud file
    // pcl::io::savePCDFileASCII ("output.pcd", cloud);
    
    cloud.width    = 1;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    
    for (size_t i = 0; i < 1; ++i){
        cloud.points[i].x = 1;
        cloud.points[i].y = 1;
        cloud.points[i].z = 1;
        cloud.points[i].r = 255;
        cloud.points[i].g = cloud.points[i].b = 0;
    }
    
    
    while (1) {
        //    viewer.spinOnce(50);
        viewer.spin();
    }
    
    std::cout << "EXITING SHOW PCL!!!\n";

    
    
}

