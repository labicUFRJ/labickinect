#include "ransac_aligner.h"
#include <cassert>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>

using namespace std;
using namespace pcl;
using namespace labic;

RANSACAligner::RANSACAligner() : maxIterations(100), nSamples(3), inlierThreshold(5.0), minInliers(10) {
	reset();

	estimator = &estimatorSVD;

	srand(time(NULL));
}

void RANSACAligner::reset() {
	bestTransform.setZero();
	bestError = INFINITY;
	bestIteration = 0;
	bestConsensusSetIndexes.clear();
	numFeatures = 0;
}

void RANSACAligner::getRandomSamples(std::vector<int>& maybeIndexes, std::vector<int>& notMaybeIndexes) const {
	maybeIndexes.clear();
	notMaybeIndexes.clear();

//	ddebug << "	maybeIndexes[";
	while (maybeIndexes.size() < nSamples) {
		int randomSample = rand() % numFeatures;
//		ddebug << randomSample << ", ";
		unsigned int i;
		for (i=0; i<maybeIndexes.size(); i++) {
			if (randomSample == maybeIndexes[i]) break;
		}

		// If i reached position n, then no match was found (it is new)
		if (i == maybeIndexes.size()) maybeIndexes.push_back(randomSample);
	}
//	ddebug << "]" << endl;
//	if (maybeIndexes.size() > nSamples) derr << "RANDOM SAMPLES WITH TOO MANY SAMPLES!!" << endl;

	//ddebug << "	not[";
	for (unsigned int i=0; i<numFeatures; i++) {
		bool isInMaybeIndexes = false;
		for (unsigned int j=0; j<maybeIndexes.size(); j++) {
			if (i == (unsigned int)maybeIndexes[j]) {
				isInMaybeIndexes = true;
				break;
			}
		}
		if (isInMaybeIndexes) continue;
		//ddebug << i << ", ";
		notMaybeIndexes.push_back(i);
	}
	//ddebug << "]" << endl;

}

double RANSACAligner::getAlignmentError(const PointCloud<PointXYZRGB>& transformedCloud, const PointCloud<PointXYZRGB>& cloudPrevious, const vector<int>& inliersIndexes) const {
    assert(transformedCloud.size() == cloudPrevious.size());

    double error = 0.0;

    for (unsigned int i=0; i<inliersIndexes.size(); i++) {
        int inlierIndex = inliersIndexes[i];
//        error += euclideanDistance(transformedCloud.points[inlierIndex], cloudPrevious.points[inlierIndex])/1000; // distance divided by 1000 because original measure is in MM not in meters
        error += squaredPixelDistance(transformedCloud.points[inlierIndex], cloudPrevious.points[inlierIndex]);
    }

    error /= inliersIndexes.size();
    return error;
}

void RANSACAligner::estimate(pcl::PointCloud<pcl::PointXYZRGB>& cloudPrevious, pcl::PointCloud<pcl::PointXYZRGB>& cloudCurrent) {
    assert(cloudCurrent.size() == cloudPrevious.size());
    reset();

	double thisError;
	Eigen::Matrix4d maybeTransform, thisTransform;
	vector<int> maybeIndexes, notMaybeIndexes, consensusSetIndexes;

	unsigned int iterations = 0;
	numFeatures = cloudCurrent.size();

	while (iterations < maxIterations) {
		ddebug << ">>> RANSAC iteration " << iterations << endl;

		consensusSetIndexes.clear();
		maybeTransform.setZero();

		// Determine random sample (maybe)
		getRandomSamples(maybeIndexes, notMaybeIndexes);
		consensusSetIndexes = maybeIndexes;

		// Estimate transformation from maybe set (size = nSamples)
		estimator->estimateRigidTransformation(cloudCurrent, maybeIndexes, cloudPrevious, maybeIndexes, maybeTransform);

		for (unsigned int i=0; i<notMaybeIndexes.size(); i++) {
			int pointIndex = notMaybeIndexes[i];
			// creating pointcloud just to test transformation with a single point
			PointCloud<PointXYZRGB> transformedPoint;
			transformedPoint.clear();
			transformedPoint.push_back(cloudCurrent.points[pointIndex]);
			transformPointCloud(transformedPoint, transformedPoint, maybeTransform);

			float transformedDistance;
//			transformedDistance = euclideanDistance(transformedPoint.points[0], cloudPrevious.points[pointIndex])/1000; // distance divided by 1000 because original measure is in MM not in meters
			transformedDistance = pixelDistance(transformedPoint.points[0], cloudPrevious.points[pointIndex]);

//			ddebug << "       Point " << pointIndex << " distance (" << transformedPoint.points[0] << " and " << cloudPrevious.points[pointIndex] << ") = " << transformedDistance;
			if (transformedDistance < inlierThreshold) {
				consensusSetIndexes.push_back(pointIndex);
				//ddebug << " (added to consensus set!)";
			}
			//ddebug << endl;
		}

//		ddebug << "	Consensus set has " << consensusSetIndexes.size() << " points" << endl;

		if (consensusSetIndexes.size() > minInliers) {
//			ddebug << "	Consensus set has more than " << minInliers << " inliers. Finding consensus set transformation" << endl;

			// Recalculate transformation from new consensus set
			estimator->estimateRigidTransformation(cloudCurrent, consensusSetIndexes, cloudPrevious, consensusSetIndexes, thisTransform);

//			ddebug << "       thisTransform = " << endl << thisTransform << endl;

			// Generate the transformed cloud using thisTransform
			// Note that this cloud will include points that are not in consensus set, be careful
			PointCloud<PointXYZRGB> transformedCloudCurrent;
			transformPointCloud(cloudCurrent, transformedCloudCurrent, thisTransform);
			thisError = getAlignmentError(transformedCloudCurrent, cloudPrevious, consensusSetIndexes);

//			ddebug << "       thisError = " << thisError;
			if (thisError < bestError) {
//				ddebug << "	Great! best error so far (thisError = "  << thisError << "). Updating best parameters." << endl;
				bestIteration = iterations;
				bestTransform = thisTransform;
				bestError = thisError;
				bestConsensusSetIndexes = consensusSetIndexes;
			} else {
//				ddebug << "	Iteration was not the best. thisError = "  << thisError << endl;
			}
		}

		// TODO test to stop if found error < ok_error
		ddebug << "	Best iteration so far: " << bestIteration << " (bestError = " << bestError << ")" << endl;
		iterations++;
	}

	dinfo << "[RANSAC] Ransac finished. Best iteration: " << bestIteration << ". Best error: " << bestError << ". Best consensus set: " << bestConsensusSetIndexes.size() << " points" << endl;
}
