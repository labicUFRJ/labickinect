#include "labickinect.h"

freenect_context *f_ctx;
freenect_device *f_dev;

int got_depth, got_rgb;

int kinect_start() {
    if (freenect_init(&f_ctx, NULL) < 0) {
        printf("Kinect: freenect_init() failed\n");
        return 0;
    }
    
    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
    
    int nr_devices = freenect_num_devices (f_ctx);
    printf ("Kinect: Number of devices found: %d\n", nr_devices);
    
    int user_device_number = 0;
    
    if (nr_devices < 1) {
        freenect_shutdown(f_ctx);
        return 0;
    }
    
    if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
        printf("Kinect: Could not open device %d\n", user_device_number);
        freenect_shutdown(f_ctx);
        return 0;
    }
    
    printf("Kinect: Device open\n");
    kinect_open = 1;
    
    kinect_angle = 0;
    kinect_tilt();
    return 1;
}

void kinect_shutdown() {
    freenect_set_led(f_dev,LED_GREEN);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
    printf("Kinect: Device closed\n");
    kinect_open = 0;
}

void kinect_tilt() {
    freenect_set_tilt_degs(f_dev, kinect_angle);
}

void *freenect_threadfunc(void *arg) {
	printf("Kinect: Starting thread kinect\n");
//    int i=0;
    
	if (freenect_set_led(f_dev,LED_RED) < 0) printf("Kinect: Error setting LED.\n");
	
    // Depth settings
    if (DEPTH_ON) {
        freenect_set_depth_callback(f_dev, depth_cb);
        if (freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED)) < 0) printf("Kinect: Error setting depth mode\n");
        
        if (freenect_start_depth(f_dev) < 0) printf("Kinect: Error starting depth\n");
    }
    
    // Video settings
    if (RGB_ON) {
        freenect_video_format current_format = FREENECT_VIDEO_RGB;
        freenect_set_video_callback(f_dev, rgb_cb);
        freenect_set_video_buffer(f_dev, rgb_back);
        if (freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format)) < 0) printf("Kinect: Error setting video mode\n");
        
        if (freenect_start_video(f_dev) < 0) printf("Kinect: Error starting video\n");
    }
    
    // Kinect loop
    while (freenect_process_events(f_ctx) >= 0 && !kinect_stop) {}
    
    printf("\n");
    
    if (DEPTH_ON) {
        freenect_stop_depth(f_dev);
        printf("Kinect: Depth capture stopped.\n");
    }
    if (RGB_ON) {
        freenect_stop_video(f_dev);
        printf("Kinect: Video capture stopped.\n");
    }
    
	printf("Kinect: Closing thread and shutting down kinect\n");
    
	kinect_shutdown();
    pthread_exit(NULL);
}

// Depth processing thread
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp) {
    pthread_mutex_lock(&em_depth);
	depth = (uint16_t*)v_depth;
    got_depth = 1;
    pthread_cond_signal(&cond_depth);
    pthread_mutex_unlock(&em_depth);
}

// RGB processing thread
void rgb_cb(freenect_device *dev, void *rgbin, uint32_t timestamp) {
//    printf("rgb callback\n");
    pthread_mutex_lock(&em_rgb);
    
    // swap buffers
    assert(rgb_back == rgbin);
	rgb_back = rgb;
    freenect_set_video_buffer(dev, rgb_back);
	rgb = (uint8_t*)rgbin;
    got_rgb = 1;
    pthread_cond_signal(&cond_rgb);
    pthread_mutex_unlock(&em_rgb);
}

// Convert raw 11-bit depth data from Kinect to meters
float raw_depth_to_meters(float depth_value) {
    return depth_value/1000.0;
    
    float depth_value_f = (float) depth_value;
    float depth = 0;
    
    if (depth_value < DEPTH_BLANK){
        depth = 1.0 / (depth_value_f * (-0.0030711016) + 3.3309495161);
    }
    
    return depth;
}

int mm_to_raw (float depth_value) {
    int z = DEPTH_BLANK;
    if (depth_value > 0) {
        z = (int) MAX(0, ((1.0/(depth_value/1000.0)) - 3.3309495161)/(-0.0030711016));
    }
    return z;
}

// Convert Kinect depth image (x,y,depth) to real-world coordinates
xyz depth_to_xyz (float cgx, float cgy, float cgz) {
    double fx_d = 1.0 / 5.9421434211923247e+02;
    double fy_d = 1.0 / 5.9104053696870778e+02;
    double cx_d = 3.3930780975300314e+02;
    double cy_d = 2.4273913761751615e+02;
    
    cgz = mm_to_raw(cgz);
    
    if (cgz == 0) {
        //    if (cgz == DEPTH_BLANK) {
        xyz zero = {0,0,0};
        return zero;
    }
    
    cgz = raw_depth_to_meters(cgz);
    
    xyz pt = {
        (float) (cgx - cx_d) * cgz * fx_d,
        (float) (cgy - cy_d) * cgz * fy_d,
        (float) cgz };
    
    return pt;
    
}

// Convert Kinect depth image (x,y,depth) to real-world coordinates
cv::Point3d depth_to_point3d (float cgx, float cgy, float cgz) {
    double fx_d = 1.0 / 5.9421434211923247e+02;
    double fy_d = 1.0 / 5.9104053696870778e+02;
    double cx_d = 3.3930780975300314e+02;
    double cy_d = 2.4273913761751615e+02;
    
    cgz = mm_to_raw(cgz);
    
    if (cgz == 0 || cgz == DEPTH_BLANK) {
        return cv::Point3d(0,0,0);
    }
    
    cgz = raw_depth_to_meters(cgz);
    
    return cv::Point3d(
                       (cgx - cx_d) * cgz * fx_d,
                       (cgy - cy_d) * cgz * fy_d,
                       cgz
                      );
    
}
