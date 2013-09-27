/*
 * RANSACAligner.h
 *
 *  Created on: Sep 25, 2013
 *      Author: macecchi
 */

#ifndef RANSACALIGNER_H_
#define RANSACALIGNER_H_

#include <iostream>
#include <cassert>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/io/ply_io.h>

namespace labic {
	class RANSACAligner {
	public:
		RANSACAligner();
		void estimate(pcl::PointCloud<pcl::PointXYZRGB>& cloudPrevious, pcl::PointCloud<pcl::PointXYZRGB>& cloudCurrent);
		void setMaxIterations(int _maxIterations) { maxIterations = _maxIterations; }
		void setMinInliers(int _minInliers) { minInliers = _minInliers; }
		void setDistanceThreshold(double _dist) { inlierThreshold = _dist; }
		void setNumSamples(int _n) { nSamples = _n; }
		const Eigen::Matrix4d& getFinalTransform() const { return bestTransform; }
		const std::vector<int>& getFinalInliers() const { return bestConsensusSetIndexes; }

	private:
		void getRandomSamples(std::vector<int>& maybe, std::vector<int>& notMaybe) const;
		double getAlignmentError(const pcl::PointCloud<pcl::PointXYZRGB>& transformedCloud, const pcl::PointCloud<pcl::PointXYZRGB>& cloudPrevious, const std::vector<int>& inliersIndexes) const;

		int maxIterations; // k
		int nSamples; // number of maybe_inliers (random samples)
		double inlierThreshold; // max error
		int minInliers;
		int numFeatures;
		int bestIteration;
		double bestError;
		Eigen::Matrix4d bestTransform;
		std::vector<int> bestConsensusSetIndexes;
		pcl::PointCloud<pcl::PointXYZRGB> cloudCurrent, cloudPrevious;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB, double> estimatorSVD;
		pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB, double> estimatorLM;
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB, double>* estimator;
	};
}


#endif /* RANSACALIGNER_H_ */
