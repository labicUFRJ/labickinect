#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <libfreenect/libfreenect.h>

#include "cv_matcher.h"


#define DEPTH_ON 1 // Enable depth capture
#define RGB_ON 1 // Enable RGB video capture
#define DEPTH_GRAPH_ON 0 // Enable depth graph
#define DETECTOR_ON 0 // Enable detector
#define CV_ON 1 // Enable CV graphics
#define RECONSTRUCTOR_ON 1 // Enable 3d reconstructor

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define GRAPH_WIDTH 800
#define GRAPH_HEIGHT 600

#define DEPTH_BLANK 2047 // valor retornado pelo Kinect quando a distancia eh desconhecido

// Constantes do detector e CV para processamento dos dados de profundidade
#define MAX_DISTANCE 10.0
//#define EVAL_Y_MIN 220 // altura min para processamento
//#define EVAL_Y_MAX 260 // altura max para processamento
#define MAX_DIST_X 1.0
#define MAX_DIST_Y 1.0
#define MAX_DOORS 10
#define EVAL_ERROR 0.06

#define REFRESH_INTERVAL 1  // intervalo de atualizacao do Kinect (ms)

// Funcoes de minimo e maximo padroes
//#define max(a,b) \
//({ __typeof__ (a) _a = (a); \
//__typeof__ (b) _b = (b); \
//_a > _b ? _a : _b; })
//#define min(a,b) \
//({ __typeof__ (a) _a = (a); \
//__typeof__ (b) _b = (b); \
//_a < _b ? _a : _b; })

// Estrutura do detector para selecionar onde comecam e terminam as paredes laterais
typedef enum {
    NO_WALLS = 1,
    LEFT_WALL = 2,
    RIGHT_WALL = 3,
    BOTH_WALLS = 4
} state_walls;

// Lista para armazenar onde foram detectadas portas
typedef struct _doors_list {
    int x;
    struct _doors_list *pt;
} doors_list;

// Ponto do espaco R3, correspondente a localizacao do mudno real da profundidade lida
typedef struct _xyz {
    double x, y, z;
} xyz;

// sistema_fuzzy.cpp
void fuzzy_rodar();

// main.c
void encerrar(int);
float distancia_media(int);
void export_xydata();
void save_state(char);

// kinect.c
int kinect_start();
void kinect_shutdown();
void kinect_tilt();
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp);
void rgb_cb(freenect_device *dev, void *rgbin, uint32_t timestamp);
void *freenect_threadfunc(void *arg);
float raw_depth_to_meters(float);
int mm_to_raw (float);
xyz depth_to_xyz (float cgx, float cgy, float cgz);
cv::Point3d depth_to_point3d (float cgx, float cgy, float cgz);

// detector.c
void *detector(void *args);
void detector_add_door(int x);

// cv.c
void cv_initialize();
void cv_depth();
void cv_rgb();
void *cv_depth_thread(void *args);
void *cv_rgb_thread(void *args);
void *cv_reconstructor_thread(void *args);
static void cv_mouse_handler(int event, int x, int y, int flags, void* param);
cv::Vec3b cv_depth_to_color(float);
void cv_write_fps(cv::Mat, float);
void cv_3d_recon();

// Variaveis compartilhadas
extern pthread_mutex_t em_depth, em_rgb, em_frame;
extern pthread_cond_t cond_depth, cond_rgb;
extern int kinect_mode; // is live Kinect mode on?
extern int kinect_open; // is Kinect open?
extern int kinect_stop; // was Kinect asked to stop?
extern int kinect_angle; // tilt
extern uint16_t *depth, *depth_s, *depth_t;
extern uint8_t *rgb, *rgb_back, *rgb_front, *rgb_s, *rgb_t;
extern int got_depth, got_rgb;
extern state_walls state;
extern int left_wall_x, right_wall_x;
extern int doors[MAX_DOORS];
extern int n_doors;
extern int saves_state;

extern int EVAL_Y_MIN;
extern int EVAL_Y_MAX;
