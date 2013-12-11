//
//  pcl.h
//  LabicKinect
//
//  Created by Mario Cecchi on 05/07/13.
//  Copyright (c) 2013 Mario Cecchi. All rights reserved.
//

#ifndef __LABICKINECT_PCL_H__
#define __LABICKINECT_PCL_H__

#include <pcl/visualization/pcl_visualizer.h>
#include "common.h"
#include "kinect/kinect_controller.h"
#include "tools/queue.h"
#include "rgbd_image.h"

namespace labic {
	
	class LabicPCL {
	public:
        LabicPCL(bool* _stop, Cloud& cloud);
		void start() { m_Thread = boost::thread(&LabicPCL::display, this); }
        void close() { viewer->close(); if (m_Thread.joinable()) m_Thread.join(); }
        void display();
        void addCameras(const std::vector<cv::Mat>& T, const std::vector<cv::Mat>& R);
        

	private:
		boost::thread m_Thread;
        bool* stop;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        Cloud& worldCloud;
        const int viewPort;

    };
}

#endif /* __LABICKINECT_PCL_H__ */
