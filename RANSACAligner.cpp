#include "RANSACAligner.h"
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
	//srand(5);
}

void RANSACAligner::reset() {
	bestTransform.setIdentity();
	bestError = INFINITY;
	bestIteration = 0;
	numFeatures = 0;
}

void RANSACAligner::getRandomSamples(std::vector<int>& maybeIndexes, std::vector<int>& notMaybeIndexes) const {
	int i;

	maybeIndexes.clear();
	notMaybeIndexes.clear();

	cout << "	maybeIndexes[";
	while (maybeIndexes.size() < nSamples) {
		int randomSample = rand() % numFeatures;
		cout << randomSample << ", ";
		for (i=0; i<maybeIndexes.size(); i++) {
			if (randomSample == maybeIndexes[i]) break;
		}
		if (i == maybeIndexes.size()) {
			maybeIndexes.push_back(randomSample);
		}
	}
	cout << "]" << endl;

	//cout << "	not[";
	for (int i=0; i<numFeatures; i++) {
		bool isInMaybeIndexes = false;
		for (int j=0; j<maybeIndexes.size(); j++) {
			if (i == maybeIndexes[j]) {
				isInMaybeIndexes = true;
				break;
			}
		}
		if (isInMaybeIndexes) continue;
		//cout << i << ", ";
		notMaybeIndexes.push_back(i);
	}
	//cout << "]" << endl;

}

double RANSACAligner::getAlignmentError(const PointCloud<PointXYZRGB>& transformedCloud, const PointCloud<PointXYZRGB>& cloudPrevious, const vector<int>& inliersIndexes) const {
    assert(transformedCloud.size() == cloudPrevious.size());

    double error = 0.0;

    for (int i=0; i<inliersIndexes.size(); i++) {
        int inlierIndex = inliersIndexes[i];
        // TODO squaredEuclideanDistance
        error += euclideanDistance(transformedCloud.points[inlierIndex], cloudPrevious.points[inlierIndex]);
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

	int iterations = 0;
	numFeatures = cloudCurrent.size();

	while (iterations < maxIterations) {
		cout << ">>> RANSAC iteration " << iterations << endl;

		consensusSetIndexes.clear();
		maybeTransform.setZero();

		// Determine random sample (maybe)
		getRandomSamples(maybeIndexes, notMaybeIndexes);
		consensusSetIndexes = maybeIndexes;
//		cout << "       maybeIndexes size = " << maybeIndexes.size() << endl;
//		cout << "       notMaybeIndexes size = " << notMaybeIndexes.size() << endl;

		// Estimate transformation from maybe set (size = nSamples)
		estimator->estimateRigidTransformation(cloudCurrent, maybeIndexes, cloudPrevious, maybeIndexes, maybeTransform);

//		cout << "       maybeTransform = " << endl << maybeTransform << endl;

		// Test transformation with other points that are not in maybeIndexes
		for (int i=0; i<notMaybeIndexes.size(); i++) {
			int pointIndex = notMaybeIndexes[i];
			// creating pointcloud just to test transformation with a single point
			PointCloud<PointXYZRGB> transformedPoint;
			transformedPoint.clear();
			transformedPoint.push_back(cloudCurrent.points[pointIndex]);
			transformPointCloud(transformedPoint, transformedPoint, maybeTransform);
			float transformedDistance = euclideanDistance(transformedPoint.points[0], cloudPrevious.points[pointIndex]);

			cout << "       Point " << i << " distance = " << transformedDistance;
			if (transformedDistance < inlierThreshold) {
				consensusSetIndexes.push_back(pointIndex);
				cout << " (added to consensus set!)";
			}
			//cout << endl;
		}

		cout << "	Consensus set has " << consensusSetIndexes.size() << " points" << endl;

		if (consensusSetIndexes.size() > minInliers) {
			cout << "	Consensus set has more than " << minInliers << " inliers. Finding consensus set transformation" << endl;

			// Recalculate transformation from new consensus set
			estimator->estimateRigidTransformation(cloudCurrent, consensusSetIndexes, cloudPrevious, consensusSetIndexes, thisTransform);

//			cout << "       thisTransform = " << endl << thisTransform << endl;

			// Generate the transformed cloud using thisTransform
			// Note that this cloud will include points that are not in consensus set, be careful
			PointCloud<PointXYZRGB> transformedCloudCurrent;
			transformPointCloud(cloudCurrent, transformedCloudCurrent, thisTransform);
			thisError = getAlignmentError(transformedCloudCurrent, cloudPrevious, consensusSetIndexes);

//			cout << "       thisError = " << thisError;
			if (thisError < bestError) {
				cout << "	Great! best error so far (thisError = "  << thisError << "). Updating best parameters." << endl;
				bestIteration = iterations;
				bestTransform = thisTransform;
				bestError = thisError;
				bestConsensusSetIndexes = consensusSetIndexes;
			} else {
				cout << "	Iteration was not the best. thisError = "  << thisError << endl;
			}
		}

		// TODO test to stop if found error < ok_error
		cout << "	Best iteration so far: " << bestIteration << " (bestError = " << bestError << ")" << endl;
		iterations++;
	}

}
