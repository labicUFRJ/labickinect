#ifndef __LABICKINECT_RANSACALIGNER_H__
#define __LABICKINECT_RANSACALIGNER_H__

#include "../common.h"
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>

namespace labic {
	class RANSACAligner {
	public:
		RANSACAligner();
		void estimate(Cloud& cloudPrevious, Cloud& cloudCurrent);
		void setMaxIterations(int _maxIterations) { maxIterations = _maxIterations; }
		void setMinInliers(int _minInliers) { minInliers = _minInliers; }
		void setDistanceThreshold(double _dist) { inlierThreshold = _dist; }
		void setNumSamples(int _n) { nSamples = _n; }
		const Eigen::Matrix4d& getFinalTransform() const { return bestTransform; }
		const std::vector<int>& getFinalInliers() const { return bestConsensusSetIndexes; }
		double getFinalError() const { return bestError; }

	private:
		void reset();
		void getRandomSamples(std::vector<int>& maybe, std::vector<int>& notMaybe) const;
		double getAlignmentError(const Cloud& transformedCloud, const Cloud& cloudPrevious, const std::vector<int>& inliersIndexes) const;

		unsigned int maxIterations; // k
		unsigned int nSamples; // number of maybe_inliers (random samples)
		double inlierThreshold; // max error
		unsigned int minInliers;
		unsigned int numFeatures;
		unsigned int bestIteration;
		double bestError;
		Eigen::Matrix4d bestTransform;
		std::vector<int> bestConsensusSetIndexes;
		//Cloud cloudCurrent, cloudPrevious;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB, double> estimatorSVD;
		pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB, double> estimatorLM;
		pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB, double>* estimator;
	};
}


#endif /* __LABICKINECT_RANSACALIGNER_H__ */
