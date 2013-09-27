#include "RANSACAligner.h"

using namespace std;
using namespace pcl;
using namespace labic;

RANSACAligner::RANSACAligner() : maxIterations(100), nSamples(3), inlierThreshold(5.0), minInliers(10) {

	bestTransform.setIdentity();
	bestError = INFINITY;

	estimator = &estimatorSVD;

	//srand(time(NULL));
	srand(5);
}

void RANSACAligner::getRandomSamples(std::vector<int>& maybeIndexes, std::vector<int>& notMaybeIndexes) const {
	cout << "	maybeIndexes[";
	for (int i=0; i<nSamples; ) {
		int randomSample = rand() % numFeatures;
		cout << randomSample << ", ";
		if (find(maybeIndexes.begin(), maybeIndexes.end(), randomSample) == maybeIndexes.end()) {
			maybeIndexes.push_back(randomSample);
			i++;
		}
	}
	cout << "]" << endl << "	not[";
	for (int i=0; i<numFeatures; i++) {
		bool isInMaybeIndexes = false;
		for (int j=0; j<maybeIndexes.size(); j++) {
			if (i == maybeIndexes[j]) {
				isInMaybeIndexes = true;
				break;
			}
		}
		if (isInMaybeIndexes) continue;
		cout << i << ", ";
		notMaybeIndexes.push_back(i);
	}
	cout << "]" << endl;

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

	double thisError;
	Eigen::Matrix4d maybeTransform, thisTransform;
	vector<int> maybeIndexes, notMaybeIndexes, consensusSetIndexes;

	int iterations = 0;
	bestIteration = 0;
	numFeatures = cloudCurrent.size();

	while (iterations < maxIterations) {
		cout << ">>> RANSAC iteration " << iterations+1 << endl;

		maybeIndexes.clear();
		notMaybeIndexes.clear();
		consensusSetIndexes.clear();
		maybeTransform = Eigen::Matrix4d::Zero();

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
			transformedPoint.push_back(cloudCurrent.points[pointIndex]);
			transformPointCloud(transformedPoint, transformedPoint, maybeTransform);
			float transformedDistance = euclideanDistance(transformedPoint.points[0], cloudPrevious.points[pointIndex]);

			//cout << "       Point " << i << " distance = " << transformedDistance;
			if (transformedDistance < inlierThreshold) {
				consensusSetIndexes.push_back(pointIndex);
				//cout << " (added to consensus set!)";
			}
			//cout << endl;
		}

		cout << "       consensusSet has " << consensusSetIndexes.size() << " points" << endl;

		if (consensusSetIndexes.size() > minInliers) {
			cout << "           (Consensus set has more than minInliers. Finding new transformation and comparing it to the best..." << endl;

			// Recalculate transformation from new consensus set
			estimator->estimateRigidTransformation(cloudPrevious, consensusSetIndexes, cloudCurrent, consensusSetIndexes, thisTransform);

//			cout << "       thisTransform = " << endl << thisTransform << endl;

			// Generate the transformed cloud using thisTransform
			// Note that this cloud will include points that are not in consensus set, be careful
			PointCloud<PointXYZRGB> transformedCloudCurrent;
			transformPointCloud(cloudCurrent, transformedCloudCurrent, thisTransform);
			thisError = getAlignmentError(transformedCloudCurrent, cloudPrevious, consensusSetIndexes);

//			cout << "       thisError = " << thisError;
			if (thisError < bestError) {
				cout << " (Great! best error so far. updating best parameters)" << endl;
				bestIteration = iterations;
				bestTransform = thisTransform;
				bestError = thisError;
				bestConsensusSetIndexes = consensusSetIndexes;
			}
		}

		// TODO test to stop if found error < ok_error
		cout << ">>> Best iteration was " << bestIteration << " (bestError is " << bestError << ")" << endl;
		iterations++;
	}

}
