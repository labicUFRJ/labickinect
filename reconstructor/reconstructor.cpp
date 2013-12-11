#include <pcl/common/eigen.h>
#include <pcl/io/ply_io.h>
#include "../kinect/kinect_controller.h"
#include "reconstructor.h"
#include "reconstruction_exception.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace labic;

Reconstructor::Reconstructor(bool* _stop, Queue<RGBDImage>& q) : stop(_stop), autoSave(false), queue(q) {
	// Reconstructor parameters
	minInliersToValidateTransformation = 10; // gamma - min inliers to accept ransac

	framesAnalyzed = 0;
	reconstructionsGenerated = 0;
	reconstructionsAccepted = 0;
	featuresExtracted = 0;
	featuresMatched = 0;
	matchesDiscarded = 0;
	pointsDetected = 0;
	totalTime = 0;
	totalError = 0;
	lastError = 0;

	transformGlobal.setIdentity();
	transformPrevious.setIdentity();
}

void Reconstructor::threadFunc() {
	hrclock::time_point t;
	double timeReconstruction;
    cout << "[Reconstructor] Reconstructor initialized" << endl;

    while (!*stop || queue.size() > 0) {
    	if (queue.size() > 0) {
    		// If this is the first frame received, just save it
    		if (world.empty()) {
    		    dinfo << "[Reconstructor] Preparing first frame" << endl;
    			rgbdPrevious = queue.pop();

    			world = rgbdPrevious.pointCloud();

    			pointsDetected = world.size();

    			dinfo << "[LabicReconstructor] Initial frame saved" << endl;

    		} else {
				cout << "[Reconstructor] Reconstructing frame " << reconstructionsGenerated+1 << "..." << endl;

				rgbdCurrent = queue.pop();

				if (rgbdCurrent == rgbdPrevious) {
					derr << "ERROR - LOOP WITH EQUAL FRAMES" << endl;
					continue;
				}

	        	t = hrclock::now();
	        	performAlignment();
	    		totalTime += (timeReconstruction = diffTime(hrclock::now(), t));

	    		dinfo << "[Reconstructor] Finished reconstruction loop (" << timeReconstruction << " secs)" << endl;
				ddebug << queue << endl;
    		}

			framesAnalyzed++;
		}
    	else {
    		//boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    	}
    }

    cout << "[Reconstructor] Exporting final world. Please wait... " << endl;
    if (world.size() > 0) {
    	pcl::io::savePLYFileASCII("world.ply", world);
    	cout << "Done." << endl;
    }
    else cout << "Nothing to be done." << endl;

    printStats();

    cout << "[Reconstructor] Reconstructor finished" << endl;

}

void Reconstructor::performAlignment() {
	Eigen::Matrix4d pairTransform;
	vector<int> pairTransformInliersIndexes;
	double ransacError;

	try {
		// Steps 1 to 3: Visual RERANSAC
		visualReconstructor(rgbdPrevious, rgbdCurrent);

		pairTransform = visualReconstructor.getFinalTransform();

		ransacError = visualReconstructor.getFinalError();
		pairTransformInliersIndexes = visualReconstructor.getFinalInliers();

		dinfo << "[Reconstructor] Final transformation matrix that resulted in " << pairTransformInliersIndexes.size() << " inliers: " << endl << pairTransform << endl;

		reconstructionsGenerated++;

		// Step 4: Check if transformation generated the correct set of inliers
		if (pairTransformInliersIndexes.size() < minInliersToValidateTransformation) {
			dwarn << "[Reconstructor] Visual RANSAC transformation NOT accepted (did not generate the mininum of inliers)" << endl;

			// Step 5: Use same transformation as previous reconstruction
			transformGlobal = transformPrevious * transformGlobal;
			ddebug << "Repeating previous transformation:\n" << transformPrevious << endl;

			// Step 6: Clean untrustworthy associations
			pairTransformInliersIndexes.clear();
			ransacError = lastError;
		} else {
			dinfo << "[Reconstructor] Initial visual transformation accepted" << endl;

			// TODO ICP Loop

			transformPrevious = pairTransform;
			transformGlobal = pairTransform * transformGlobal; // TODO or the opposite??????????
			lastError = ransacError;
			reconstructionsAccepted++;
		}

		totalError += ransacError;

		// 5. Apply transformation to all frame points
		ddebug << "Final accumulated transformation:\n" << transformGlobal << endl;

		Cloud cloudCurrent = rgbdCurrent.pointCloud();
		Cloud alignedCloudCurrent;
		transformPointCloud(cloudCurrent, alignedCloudCurrent, transformGlobal);

		// 6. Update 'previous' variables
		rgbdCurrent.copyTo(rgbdPrevious);

		// 7. Update world with new frame
		world += alignedCloudCurrent;
		cout << "[Reconstructor] World updated. Total of " << world.size() << " points (estimated size: " << ((world.size()*sizeof(PointXYZRGB))>>20)*1.35 << " MB)" << endl;

		if (autoSave) {
			char filenameply[25];
			sprintf(filenameply, "world%d.ply", reconstructionsGenerated);
			pcl::io::savePLYFileASCII(filenameply, world);
			cout << "[Reconstructor] World exported with filename '" << filenameply << "'" << endl;
		}
	} catch (ReconstructionException& e) {
		derr << "[Reconstructor] Error obtaining alignment: " << e.what() << endl;
	}
}


void Reconstructor::printStats() const {
	cout << "[Reconstructor] STATS" << endl
		 << "	Frames analyzed: " << framesAnalyzed << endl;
	if (framesAnalyzed > 1)
	cout << "	Total error: " << totalError << " (avg. " << totalError/reconstructionsGenerated << ")" << endl
		<< "	Features extracted: " << featuresExtracted << " (avg. " << featuresExtracted/framesAnalyzed << ")" << endl
		 << "	Features matched: " << featuresMatched << " (avg. " << featuresMatched/(framesAnalyzed-1) << ")" << endl
		 << "	Matches discarded: " << matchesDiscarded << " (avg. " << matchesDiscarded/(framesAnalyzed-1) << ")" << endl
		 << "	Points detected: " << pointsDetected << " (avg. " << pointsDetected/framesAnalyzed << ")" << endl
		 << "	Reconstructions generated: " << reconstructionsGenerated << " (" << reconstructionsGenerated-reconstructionsAccepted << " NOT accepted)" << endl
		 << "	Reconstruction time: " << totalTime << " secs (avg. " << totalTime/reconstructionsGenerated << " secs)" << endl
	;
}
