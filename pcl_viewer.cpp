#include "pcl_viewer.h"
#include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace labic;

LabicPCL::LabicPCL(bool* _stop, Cloud::Ptr cloud):
  stop(_stop), liveCloud(cloud), viewPort(1) {
    std::cout << "[PCLVisualizer] Viewer initialized\n";
}

void LabicPCL::display() {
	std::cout << "[PCLVisualizer] Display started" << std::endl;

	viewer.reset(new PCLVisualizer("Current World"));
    viewer->setBackgroundColor(0,0,0);

    PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;

    size_t lastSize = 0;

    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -1, 0, -1, 0);
    viewer->addText("Cloud empty", 20, 10, "size");

	while (!viewer->wasStopped() && !*stop) {
		if (liveCloud->size() != lastSize) {
			lastSize = liveCloud->size();
//			rgb.setInputCloud(liveCloud);
//
			if (!viewer->updatePointCloud(liveCloud, rgb)) {
				viewer->addPointCloud(liveCloud, rgb);
			    rgb.setInputCloud(liveCloud);
			}

			stringstream txtSize;
			txtSize << "Cloud size: " << liveCloud->size() << " points";
			viewer->updateText(txtSize.str(), 20, 10, "size");
		}

		viewer->spinOnce(100, true);
	    boost::this_thread::sleep(boost::posix_time::milliseconds(100)); // TODO WHY
	}

	cout << "[PCLVisualizer] User closed window" << endl;

	viewer->close();
	*stop = true;
	
    std::cout << "[PCLVisualizer] Display finished\n";
}

void LabicPCL::addCameras(const vector<cv::Mat>& T, const vector<cv::Mat>& R) {
    
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

