#ifndef __LABICKINECT_PCL_H__
#define __LABICKINECT_PCL_H__

#include "common.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace labic {
	
	class LabicPCL {
	public:
        LabicPCL(bool* _stop);
		void start() { m_Thread = boost::thread(&LabicPCL::display, this); }
        void close() { viewer->close(); if (m_Thread.joinable()) m_Thread.join(); }
        void display();
        void addCameras(const std::vector<cv::Mat>& T, const std::vector<cv::Mat>& R);
        
	private:
		boost::thread m_Thread;
        bool* stop;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        Cloud::Ptr liveCloud;
        const int viewPort;

    };
}

#endif /* __LABICKINECT_PCL_H__ */
