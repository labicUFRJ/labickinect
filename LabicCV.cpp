#include "LabicCV.h"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "rgbd_image.h"

using namespace std;
using namespace cv;
using namespace labic;

const string LabicCV::input_window = "Kinect Input";

LabicCV::LabicCV(KinectController *_kinect, bool* _stop, Queue<RGBDImage>& q)
: kinect(_kinect),  windowClosed(false), currentSet(false), stop(_stop), captureInterval(-1), captureHold(false), startCapture(true), cameras(Mat(height, width*2, CV_8UC3)), queue(q), savedFrames(0), processedFrames(0), totalTime(0) {
    for (unsigned int i=0; i<2048; i++) {
		float v = i/2048.0;
		v = pow(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

    namedWindow(input_window);
}

void LabicCV::display() {
	Mat depthMat(Size(width, height), CV_8UC3, Scalar(0));
    Mat left(cameras, Rect(0, 0, width, height));
    Mat right(cameras, Rect(width, 0, width, height));
    RGBDImage frame;

    uint32_t timestampPrevious = 0;
    long long unsigned int clicks = 0;
    double fps = 0;
    double seconds = 0;

    startCapture = !captureHold;

    hrclock::time_point first, start, end, lastCap;

    dinfo << "[LabicCV] Display started" << endl;

    first = lastCap = start = hrclock::now();

    do {
        kinect->grabRGBDImage(frame);
        
        // Skip redrawing if there was no change
        if (frame.timestamp() == timestampPrevious) continue;

        generateDepthImage(frame.depth(), depthMat);
        
        frame.rgb().copyTo(left);
        depthMat.copyTo(right);
        
        timestampPrevious = frame.timestamp();

        // Calculate FPS
        end = hrclock::now();
        seconds = diffTime(end, start);
        if (seconds > 0) fps = clicks++ / seconds;

        // If automatic mode is on and can start capture
        if (captureInterval > 0 && startCapture) {
			seconds = diffTimeMs(end,lastCap);

			if (seconds > captureInterval || !savedFrames) {
				//ddebug << "difftime for capture in ms: " << seconds << endl;
				lastCap = hrclock::now();
				if (!savedFrames) first = lastCap;
				saveFrame();
				savedFrames++;
			}
        }

        ostringstream fps_str;
        fps_str << "OpenCV FPS: " << fps;
        putText(cameras, fps_str.str(), Point(20,30), FONT_HERSHEY_PLAIN, 1.0f, Scalar::all(0));

        processedFrames++;
    } while (!*stop);

    windowClosed = true;

    totalTime = diffTime(lastCap, first);
}

void LabicCV::generateDepthImage(const Mat1f& depth, Mat depthMat) {
    int x, y;
    int depthValue;

    for (y=0; y<height; y++) {
    	for (x=0; x<width; x++) {
    		depthValue = mmToRaw(depth(y,x));
    		depthMat.at<Vec3b>(y,x) = depthToColor(depthValue);
    	}
    }
}

void LabicCV::saveFrame() {
	RGBDImage tmp;
	while (!kinect->grabRGBDImage(tmp)) {}
	dinfo << "[LabicCV] Pushed frame " << tmp.timestamp() << " to queue" << endl;
	queue.push(tmp);
	ddebug << queue << endl;
}

void LabicCV::printStats() const {
    dinfo << "[LabicCV] Display finished\n"
    	  << "          Total run time: " << totalTime << " seconds\n"
    	  << "          Total processed frames: " << processedFrames << "\n"
    	  << "          Total captures: " << savedFrames << " (" << ((double)savedFrames)/totalTime << " captures/sec)" << endl;
}

Vec3b LabicCV::depthToColor(float rawDepthValue) {
    double r,g,b;
	int pval = t_gamma[(int)rawDepthValue];
    int lb = pval & 0xff;
    switch (pval>>8) {
        case 0:
            r = 255;
            g = 255-lb;
            b = 255-lb;
            break;
        case 1:
            r = 255;
            g = lb;
            b = 0;
            break;
        case 2:
            r = 255-lb;
            g = 255;
            b = 0;
            break;
        case 3:
            r = 0;
            g = 255;
            b = lb;
            break;
        case 4:
            r = 0;
            g = 255-lb;
            b = 255;
            break;
        case 5:
            r = 0;
            g = 0;
            b = 255-lb;
            break;
        default:
            r = 0;
            g = 0;
            b = 0;
            break;
    }
    
    return Vec3b(b,g,r);
}

void LabicCV::keyboardHandler(int key) {
    switch (key) {
        case 27:
            *stop = true;
            destroyAllWindows();
            break;
        case 'w':
            kinect->setTilt(+1.0);
            break;
        case 's':
            kinect->setTilt(0.0);
            break;
        case 'x':
            kinect->setTilt(-1.0);
            break;
        case ' ':
        	if (!captureHold && captureInterval > 0) break;
        	if (captureHold && captureInterval > 0) startCapture = true;
			saveFrame();
            break;
        default:
        	break;

    }
}

void LabicCV::mainLoopPart(const int t) {
	if (*stop) return ;
	imshow(input_window, cameras);
    keyboardHandler(waitKey(t));
}

void LabicCV::close() {
    *stop = true;
    destroyAllWindows();
    join();
}
