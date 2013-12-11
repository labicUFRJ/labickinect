#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

using namespace std;

template <class T>
pcl::PointCloud<T> downsample(const pcl::PointCloud<T>& cloud, double leafSize = 0.01) {
	pcl::PointCloud<T> cloud_filtered;

	std::cout << "PointCloud before filtering: " << cloud.width * cloud.height
	   << " data points (" << pcl::getFieldsList(cloud) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<T> sor;
	sor.setInputCloud(cloud.makeShared());
	sor.setLeafSize(leafSize, leafSize, leafSize);
	sor.filter(cloud_filtered);

	std::cout << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height
	   << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << std::endl;

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGBNormal> resample(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, double radius = 0.03) {
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud.makeShared());
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(radius);

	// Reconstruct
	mls.process(mls_points);

	return mls_points;
}

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
//
  // Fill in the cloud data
  pcl::PLYReader reader;
  // Replace the path below with the path where you saved your file
  reader.read("world.ply", cloud); // Remember to download the file first!
//
//  std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
//       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
//
//  // Create the filtering object
//  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.01f, 0.01f, 0.01f);
//  sor.filter (*cloud_filtered);
//
//  std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PLYWriter writer;

//  cout << "DOWNSAMPLING..." << endl;
//  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered = downsample(cloud, 0.01f);
//  cout << "Exporting downsampled cloud" << endl;
//  writer.write("world_downsampled.ply", cloud_filtered);

  cout << "RESAMPLING..." << endl;
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_filtered = resample(cloud, 0.01);
  cout << "Exporting resampled cloud" << endl;
  writer.write("world_resampled.ply", cloud_filtered);

  std::cout << std::endl << "Done" << std::endl;
  return (0);
}
