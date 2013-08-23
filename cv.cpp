/**
 * Contem as funcoes relacionadas ao OpenCV.
**/

#include "labickinect.h"
#include "pcl.h"

using namespace cv;

string rgb_window("RGB Camera"), rgb_t_window("Target RGB Camera"), rgb_s_window("Source RGB Camera");
string depth_window("Depth"), graph_window("Graph");
string rgbd_window("RGBD Video");
Mat image;
Mat depth_image;
Mat depth_graph;
Mat rgb_image;
Mat rgb_image_t;
Mat rgb_image_s;
Mat rgbd_image;
uint16_t t_gamma[2048];
int key_alt_pressed;
int EVAL_Y_MIN = 220;
int EVAL_Y_MAX = 260;
int window_flags = CV_WINDOW_NORMAL | CV_GUI_EXPANDED | CV_WINDOW_KEEPRATIO;

pthread_cond_t cond_reconstructor;

float solve_pov(int dist) {
    float a = 0.142846048164583;
    float b = 0.00768043410597597;
    return a * (dist+b);
}

void cv_initialize() {
    pthread_cond_init(&cond_reconstructor, NULL);
    
	if (DEPTH_ON) {
        namedWindow(depth_window, window_flags);
        resizeWindow(depth_window, IMG_WIDTH, IMG_HEIGHT);
        moveWindow(depth_window, 50, 0);
    }
	if (RGB_ON) {
        namedWindow(rgb_window, window_flags);
        namedWindow(rgb_t_window, window_flags);
        namedWindow(rgb_s_window, window_flags);
        resizeWindow(rgb_window, IMG_WIDTH, IMG_HEIGHT);
        resizeWindow(rgb_t_window, IMG_WIDTH, IMG_HEIGHT);
        resizeWindow(rgb_s_window, IMG_WIDTH, IMG_HEIGHT);
        moveWindow(rgb_window, 50, 300);
        moveWindow(rgb_t_window, 650, 0);
        moveWindow(rgb_s_window, 650, 300);
    }
}

void *cv_rgb_thread(void *args) {
    printf("-- CV: Starting cv_rgb thread\n");
    cv_rgb();
    printf("-- CV: Exiting cv_rgb thread\n");
	pthread_exit(NULL);
}

void *cv_depth_thread(void *args) {
    printf("-- CV: Starting cv_depth thread\n");
	cv_depth();
    printf("-- CV: Exiting cv_depth thread\n");
	pthread_exit(NULL);
}

void *cv_reconstructor_thread(void *args) {
    printf("-- CV: Starting cv_reconstructor thread\n");
//    while (saves_state < 5 && !kinect_stop && kinect_open);
//    if (!kinect_stop && kinect_open) {
//    printf("GOT RECON\n");
//	cv_3d_recon();
//    }
    printf("-- CV: Exiting cv_reconstructor thread\n");
	pthread_exit(NULL);
}

void cv_rgb() {
    uint8_t *tmp;
    int key;
        
	printf("CV: Waiting for rgb to start...\n");
//	while (!got_rgb);
//	printf("CV: Got rgb...\n");
    
    
    do {
        pthread_mutex_lock(&em_rgb);
       
        while (!got_rgb) {
            pthread_cond_wait(&cond_rgb, &em_rgb);
        }
                
        // not sure why, copied from glview.cpp
        tmp = rgb_front;
        rgb_front = rgb;
        rgb = tmp;
        got_rgb = 0;
        pthread_mutex_unlock(&em_rgb);
        
        rgb_image = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, rgb_front);
		cvtColor(rgb_image, rgb_image, CV_RGB2BGR);
        imshow(rgb_window, rgb_image);
        
        if (saves_state == 3) {
            rgb_image_t = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, rgb_t);
            imshow(rgb_t_window, rgb_image_t);
            saves_state = 0;
        }
        else if (saves_state == 4) {
            rgb_image_s = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, rgb_s);
            imshow(rgb_s_window, rgb_image_s);
            saves_state = 5;
        }
        
		key = waitKey(REFRESH_INTERVAL);
        if (key == 27) {
            kinect_stop = 1;
            break;
        }
    } while (kinect_open && kinect_mode && !kinect_stop);
    
    printf("CV: cv_rgb() has stopped\n");
}

void cv_depth() {
    printf("-- CV: Starting cv_depth\n");

	int x, y, i, depth_value, graph_y, graph_x, key;
    uint16_t *depth_buf = (uint16_t*) malloc(IMG_HEIGHT*IMG_WIDTH*sizeof(uint16_t));
	
    int display_final = 1;
    
    // Things used to calculate fps
    time_t start, end;
    double fps, sec;
    int counter = 0;
    
    Labic::PCLViewer pcl;
    std::vector<cv::Point3d> livePoints;
    
    setMouseCallback(depth_window, cv_mouse_handler);
    
    // Color range for depth image
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
    
	printf("CV: Waiting for depth to start...\n");
	//while (!got_depth);
	printf("CV: Got depth...\n");
    
    printf("CV: Esc: quit\n");
    printf("    Left/right arrows: control source area height\n");
    printf("    Up/down arrows: move source area position\n");
    printf("    W,S,X: control Kinect angle\n");
    
    float min_x, max_x, min_y, max_y;
    xyz pt = depth_to_xyz(0, 0, depth[0]);
    min_x = max_x = pt.x;
    min_y = max_y = pt.y;
    xyz max_pt[IMG_WIDTH];
    
    time(&start);
    
    do {
        depth_image = Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);

		if (!key_alt_pressed) {
            for (i=0;i<IMG_WIDTH;i++) {
                max_pt[i].x = 0;
                max_pt[i].y = 0;
                max_pt[i].z = 0;
            }
        }
        
        pthread_mutex_lock(&em_depth);
        pthread_mutex_lock(&em_rgb);
        while (!got_depth) {
            pthread_cond_wait(&cond_depth, &em_depth);
        }
        depth_buf = (uint16_t*) memcpy(depth_buf, depth, IMG_HEIGHT*IMG_WIDTH*sizeof(uint16_t));
        pthread_mutex_unlock(&em_rgb);
        pthread_mutex_unlock(&em_depth);
        
        livePoints.clear();
		
        for (i=0;i<IMG_WIDTH*IMG_HEIGHT;i++) {
            y = i/IMG_WIDTH;
            x = i%IMG_WIDTH;
			
			depth_value = depth_buf[i];
            
			// Generate colored depth image
            depth_image.at<Vec3b>(y,x) = cv_depth_to_color(depth_value);
            
			pt = depth_to_xyz(x, y, depth_value);
            livePoints.push_back(depth_to_point3d(x, y, depth_value));
            
            // apenas para debug
            min_y = MIN(min_y, pt.y);
            max_y = MAX(max_y, pt.y);
            min_x = MIN(min_x, pt.x);
            max_x = MAX(max_x, pt.x);
            
            int divisoes = 10;
            int intervalo = IMG_WIDTH/divisoes;
            float resultado;
            float dx;
            
            if (x%intervalo <= 10 && pt.z != DEPTH_BLANK && y >= EVAL_Y_MIN && y <= EVAL_Y_MAX) {
                dx = fabs(IMG_WIDTH/2 - x);
                resultado = atan(pt.z/fabs(pt.x))*180.0/3.14159265;
                //printf("          Ponto x=%f.0, resultado=%f.1\n", pt.x, resultado);
            }
            
            if (raw_depth_to_meters(depth_value) > max_pt[x].z && y >= EVAL_Y_MIN && y <= EVAL_Y_MAX) {
                max_pt[x] = pt;
            }
        }
        
        pthread_mutex_lock(&em_depth);
        pthread_mutex_lock(&em_rgb);
        pcl.updateCloud(livePoints, rgb_image);
        pthread_mutex_unlock(&em_rgb);
        pthread_mutex_unlock(&em_depth);
        
        pcl.display();
        
        // RGB-D Image
//        if (RGB_ON & DEPTH_ON && rgb_image.size == depth_image.size) {
//            addWeighted(rgb_image, 0.5, depth_image, 0.5, 0.0, rgbd_image);
//            imshow(rgbd_window, rgbd_image);
//        }
        
        if (saves_state > 4) cv_3d_recon();
        
        
		// see how much time has elapsed
        time(&end);
        
        // calculate current FPS
        ++counter;
        sec = difftime(end, start);
        fps = counter/sec;
        
        image = depth_image.clone();
//        cv_write_fps(image, fps);
        imshow(depth_window, image);
		
        key = waitKey(REFRESH_INTERVAL);
        switch (key) {
            case 27:
                display_final = 0;
                kinect_stop = 1;
                break;
            case '1':
            case '2':
                save_state(key);
                break;
			case 'w':
				kinect_angle = min(30, kinect_angle+1);
				kinect_tilt();
				break;
			case 's':
				kinect_angle = 0;
				kinect_tilt();
				break;
			case 'x':
				kinect_angle = max(-30, kinect_angle-1);
				kinect_tilt();
				break;
            case ' ':
                display_final = 1;
                kinect_stop = 1;
                break;
            case 63235: // right, bigger area
                EVAL_Y_MAX++;
                break;
            case 63234: // left, smaller area
                if (EVAL_Y_MAX > EVAL_Y_MIN) EVAL_Y_MAX--;
                break;
            case 63232: // up, area closer to top
                EVAL_Y_MIN -= 2;
                EVAL_Y_MAX -= 2;
                break;
            case 63233: // down, area closer to bottom
                EVAL_Y_MIN += 2;
                EVAL_Y_MAX += 2;
                break;
                
        }
        
        // Free frames before entering again on loop to avoid memory leak
        if (kinect_open) depth_image.release();
        image.release();
        
//        printf("CV: min,max x = %f, %f  y = %f, %f\n", min_x, max_x, min_y, max_y);
                
    } while (kinect_open && kinect_mode && !kinect_stop);
	
	printf("CV: Finished loop. Closing Kinect. [kinect_open = %d, kinect_mode = %d]\n", kinect_open, kinect_mode);
    
    kinect_stop = 1;
    
    // Display final depth util ESC is pressed
	if (display_final) {
        while (waitKey(0) != 27);
    }
    
//	Labic::Matcher matcher(800, 1000);
	
    printf("-- CV: Exiting cv_depth()...\n");
}

void cv_3d_recon() {
    std::vector< std::vector<KeyPoint> >  imgKeyPoints;
    std::vector< Mat >                    imgDescriptors;
    Labic::Matcher   matcher(100, 300);
    std::vector< DMatch >     matches, matches_ok;
    
    std::vector< Mat >                    T, R;
    std::vector< Point3d >                objPoints;
    std::vector<std::vector<Point2d> >    imgPoints;
    Mat                                   objDescriptors;
    std::vector< KeyPoint >               objKeyPoints;
    std::vector< Mat >                    cameraMatrix;
    std::vector< Mat >                    distCoeffs;
    
    LevMarqSparse                         lms;
    std::vector<std::vector<int> >            visibility;
    TermCriteria                          criteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 200, 1e-3);
    Mat                                   k;
    Mat                                       kinect_camera_matrix;
    
    Mat bw_t, bw_s;
    Mat img_match;
    
    cv::cvtColor(rgb_image_t, bw_t, CV_RGB2GRAY);
    cv::cvtColor(rgb_image_s, bw_s, CV_RGB2GRAY);
        
    k = Mat::zeros(8,1,CV_64F);
    
//    imwrite("target.jpg", rgb_image_t);
//    imwrite("source.jpg", rgb_image_s);
//    printf("CV: Target and source images exported\n");

    // 1. reconstruct initial map from the first two image
    
    // process images (0-> source, 1-> target)
    T.push_back(Mat::zeros(3,1, CV_64F));
    R.push_back(Mat::eye(3,3, CV_64F));
    cameraMatrix.push_back(kinect_camera_matrix);
    distCoeffs.push_back(k);
    imgKeyPoints.push_back(std::vector<cv::KeyPoint>(0));
    imgDescriptors.push_back(cv::Mat());
    matcher.computeFeatures(bw_s, imgKeyPoints[0], imgDescriptors[0]);
    
    T.push_back(Mat::zeros(3,1, CV_64F));
    R.push_back(Mat::eye(3,3, CV_64F));
    cameraMatrix.push_back(kinect_camera_matrix);
    distCoeffs.push_back(k);
    imgKeyPoints.push_back(std::vector<cv::KeyPoint>(0));
    imgDescriptors.push_back(cv::Mat());
    matcher.computeFeatures(bw_t, imgKeyPoints[1], imgDescriptors[1]);

    // Match keypoints and descriptors from images (source, target)
    matcher.matchImages(imgKeyPoints[0], imgDescriptors[0],
                        imgKeyPoints[1], imgDescriptors[1], matches);
    
    //    cv::Mat F = matcher.filterMatches(imgKeyPoints[0], imgKeyPoints[1], matches, true);
    
    std::cout << "ID: " << matcher.getID() << std::endl;
//    std::cout << "F = \n" << F << "\n";
    
    std::vector<Point3d> objPointsSource = std::vector<cv::Point3d>();
    xyz _p;
    Point3d _p3d;
    
    std::cout << "Matching finished with " << matches.size() << " matches" << std::endl;
    
    for (int i=0; i<matches.size(); i++) {
        // Get RGB coordinate from source image
        int sourceIdx = matches[i].queryIdx;
        int imgx_s = imgKeyPoints[0][sourceIdx].pt.x;
        int imgy_s = imgKeyPoints[0][sourceIdx].pt.y;
        float imgz_s = depth_s[imgy_s*IMG_WIDTH+imgx_s];
        
        // Get RGB coordinate from target image
        int targetIdx = matches[i].trainIdx;
        int imgx_t = imgKeyPoints[1][targetIdx].pt.x;
        int imgy_t = imgKeyPoints[1][targetIdx].pt.y;
        float imgz_t = depth_t[imgy_t*IMG_WIDTH+imgx_t];
        
        // Check if match has depth information in both images
        if (imgz_t > 0 && imgz_s > 0) {
            matches_ok.push_back(matches[i]);
            
            // Get 3d point from source KeyPoint
            _p = depth_to_xyz(imgx_s, imgy_s, imgz_s);
            _p3d = Point3d(_p.x, _p.y, _p.z);
            objPointsSource.push_back(_p3d);
        } else {
            // If there is no depth info for match in both images, remove from matches
            std::cout << "Discarting match " << i << " (no depth information from " << ((imgz_t>0) ? "source" : "target") << ")" << std::endl;
        }
        
    }
    
    // Double check if matches worked or ignore
    if (matches_ok.size() > MIN(imgKeyPoints[0].size(), imgKeyPoints[1].size())) {
        std::cout << "Matching failed" << std::endl;
        saves_state = 0;
        return;
    }
    
    
    std::cout << "\nobjPointsSource:" << objPointsSource.size() << "\n";
    std::cout << objPointsSource << std::endl;
    
//    cv::Mat F = matcher.perform_ransac_alignment(imgKeyPoints[0], imgKeyPoints[1], matches);
    
    
    // Display matching image
    drawMatches(bw_s, imgKeyPoints[0],
                bw_t, imgKeyPoints[1], matches_ok, img_match);
    imshow("match", img_match);
    while (waitKey() != 27);
    destroyWindow("match");
    saves_state = 0;
}

static void cv_mouse_handler(int event, int x, int y, int flags, void* param) {
    int i = y*IMG_WIDTH + x;
    float depth_value;
    
    pthread_mutex_lock(&em_depth);
    depth_value = depth[i];
    pthread_mutex_unlock(&em_depth);
    
    xyz pt = depth_to_xyz(x,y,depth_value);
    if (flags == CV_EVENT_FLAG_CTRLKEY) printf("(%d,%d) = %f m -- x: %f y: %f\n", x, y, raw_depth_to_meters(depth_value), pt.x, pt.y);
    key_alt_pressed = (flags == CV_EVENT_FLAG_SHIFTKEY) ? 1 : 0;
}

Vec3b cv_depth_to_color(float depth_value) {
    // convert from mm to raw depth to calculate corresponding color
    depth_value = mm_to_raw(depth_value);
    
    double r,g,b;
	int pval = t_gamma[(int)depth_value];
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

// essa funcao da erro sei la pq
void cv_write_fps(Mat img, float _fps) {
    char fps_text[10];
    sprintf(fps_text, "FPS: %.2f", _fps);
    putText(img, fps_text, Point(20,30), CV_FONT_HERSHEY_PLAIN, 1.0f, CV_RGB(255,255,255));
}

