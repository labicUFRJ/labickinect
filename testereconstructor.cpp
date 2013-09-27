//
//  LabicKinect
//  Author: Mario Cecchi <macecchi@gmail.com>
//  Laboratorio de Inteligencia Computacional
//  www.labic.nce.ufrj.br
//

#include <iostream>
#include "LabicReconstructor.h"
#include "opencv2/core/core.hpp"


using namespace std;
using namespace cv;
using namespace labic;

int main(int argc, char **argv) {
	bool stop = false;
	LabicReconstructor recon(&stop);
	uint16_t* depthPrevious;
	uint16_t* depthCurrent;
	Mat rgbPrevious, rgbCurrent;

	int width = 640;
	int height = 480;

    depthPrevious = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
    depthCurrent = (uint16_t*) malloc(sizeof(uint16_t)*width*height);
    rgbPrevious = Mat(Size(width, height), CV_8UC3, Scalar(0));
    rgbCurrent = Mat(Size(width, height), CV_8UC3, Scalar(0));

    FILE* f;
    long size;

    f = fopen("rgbPrevious.bin", "rb");
    if (f == NULL) cerr << "Erro abrindo rgbPrevious.bin" << endl;
    size = ftell(f);
    rewind(f);
    fread(&rgbPrevious, sizeof(Mat), size, f);
    fclose(f);

    f = fopen("rgbCurrent.bin", "rb");
    if (f == NULL) cerr << "Erro abrindo rgbCurrent.bin" << endl;
    size = ftell(f);
    rewind(f);
    fread(&rgbCurrent, sizeof(Mat), size, f);
    fclose(f);

    f = fopen("depthPrevious.bin", "rb");
    if (f == NULL) cerr << "Erro abrindo depthPrevious.bin" << endl;
    size = ftell(f);
    rewind(f);
    fread(depthPrevious, sizeof(uint16_t), size, f);
    fclose(f);

    f = fopen("depthCurrent.bin", "rb");
    if (f == NULL) cerr << "Erro abrindo depthCurrent.bin" << endl;
    size = ftell(f);
    rewind(f);
    fread(depthCurrent, sizeof(uint16_t), size, f);
    fclose(f);


    // Reconstructor
	//recon.performLoop(rgbCurrent, rgbPrevious, depthCurrent, depthPrevious);

	cout << "[main] Everything closed. Bye!" << endl;

	return 0;
}
