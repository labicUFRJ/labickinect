#include "labickinect.h"

using namespace std;
pthread_t freenect_thread, cv_thread[2], detector_thread;
pthread_mutex_t em_depth, em_rgb, em_frame;
pthread_cond_t cond_depth, cond_rgb;

int kinect_mode; // is live Kinect mode on?
int kinect_open; // is Kinect open?
int kinect_stop; // was Kinect asked to stop?
int kinect_angle; // tilt

// s -> source, t-> target
extern uint16_t *depth, *depth_s, *depth_t;
extern uint8_t *rgb, *rgb_back, *rgb_front, *rgb_s, *rgb_t;

extern int got_depth, got_rgb;

state_walls state;
int left_wall_x, right_wall_x;
int doors[MAX_DOORS];
int n_doors;

int saves_state; // 0 for none, 1 for target, 2 for source

uint16_t *depth, *depth_s, *depth_t;
uint8_t *rgb, *rgb_back, *rgb_front, *rgb_s, *rgb_t;

// Calcula a distancia media em uma altura y da imagem de profundidade
float distancia_media(int y) {
    int x, i, pontos_avaliados;
    float media;
    media = 0;
    pontos_avaliados = 0;
    
    for (x=0;x<IMG_WIDTH;x++) {
        i = y*IMG_WIDTH + x;
        if (depth[i] != DEPTH_BLANK) {
            media += depth[i];
            pontos_avaliados++;
        }
    }
    
    // Se nenhuma informacao estiver disponivel sobre pelo menos 2/3 da imagem, retornar 0
    media = (pontos_avaliados>2*IMG_WIDTH/3) ? media/pontos_avaliados : 0;
    return raw_depth_to_meters(media);
}

// Exporta os dados de profundidade, em metros, em uma certa altura
void export_xydata() {
    printf("Exporting xy data (x, depth)\n");
    FILE *fxy = fopen("xydata4.txt","w");
    int x;
    int y = 203; // altura y da imagem onde os dados serao capturados
    
    if (fxy == NULL) {
        printf("Error opening xy data file\n");
        return;
    }
    
    // Salva os pontos definidos como da parede esquerda e direita
    fprintf(fxy, "%d %d\n", left_wall_x, right_wall_x);
    
    for (x=0; x<IMG_WIDTH; x++) {
        fprintf(fxy, "%d %f\n", x, raw_depth_to_meters(depth[IMG_WIDTH*y + x]));
    }
    
    fclose(fxy);
}

void save_state(char dest) {
    pthread_mutex_lock(&em_depth);
    pthread_mutex_lock(&em_rgb);
    
    // Save state to target (1)
    if (dest == '1') {
        rgb_t = (uint8_t*) memcpy(rgb_t, rgb, 3*IMG_WIDTH*IMG_HEIGHT);
        depth_t = (uint16_t*) memcpy(depth_t, depth, sizeof(uint16_t)*IMG_WIDTH*IMG_HEIGHT);
        printf("Target state updated\n");
        saves_state = 3;
    }
    // Save state to source (2)
    else if (dest == '2') {
        rgb_s = (uint8_t*) memcpy(rgb_s, rgb, 3*IMG_WIDTH*IMG_HEIGHT);
        depth_s = (uint16_t*) memcpy(depth_s, depth, sizeof(uint16_t)*IMG_WIDTH*IMG_HEIGHT);
        printf("Source state updated\n");
        saves_state = 4;
    }
    
    pthread_mutex_unlock(&em_depth);
    pthread_mutex_unlock(&em_rgb);
}



// Chama as funcoes para encerrar o programa
void encerrar(int param) {
    if (kinect_open) kinect_shutdown();
//    if (kinect_mode && state == BOTH_WALLS) export_xydata();
}

// Main: recebe como parametro o endereco de um arquivo caso queira usar um dado
// de profundidade estatico ja exportado
int main(int argc, char *argv[]) {
//    fuzzy_rodar();
	int i;
    kinect_stop = 0;
	got_depth = 0;
	got_rgb = 0;
    state = NO_WALLS;
    saves_state = 0;
    pthread_mutex_init(&em_depth, NULL);
    pthread_mutex_init(&em_rgb, NULL);
    pthread_mutex_init(&em_frame, NULL);
    pthread_cond_init(&cond_depth, NULL);
    pthread_cond_init(&cond_rgb, NULL);
    
	
    // Captura os sinais de interrupcao para encerrar corretamente o programa
	signal(SIGINT, encerrar); // CTRL+C
	signal(SIGTERM, exit); // CTRL+Z
	
    // Identify if a static depth frame was specified. If not, use live Kinect stream
    kinect_mode = (argc < 2) ? 1 : 0;
    
    // Allocate the vectors to store depth and rgb information
    // Depth is stored as 16-bit because raw depth comes as 11-bit
	depth = (uint16_t*) malloc(sizeof(uint16_t)*IMG_WIDTH*IMG_HEIGHT);
	depth_t = (uint16_t*) malloc(sizeof(uint16_t)*IMG_WIDTH*IMG_HEIGHT);
	depth_s = (uint16_t*) malloc(sizeof(uint16_t)*IMG_WIDTH*IMG_HEIGHT);
 	if (RGB_ON) {
        rgb = (uint8_t*) malloc(IMG_WIDTH*IMG_HEIGHT*3);
        rgb_back = (uint8_t*)malloc(IMG_WIDTH*IMG_HEIGHT*3);
        rgb_front = (uint8_t*)malloc(IMG_WIDTH*IMG_HEIGHT*3);
        rgb_t = (uint8_t*) malloc(IMG_WIDTH*IMG_HEIGHT*3);
        rgb_s = (uint8_t*) malloc(IMG_WIDTH*IMG_HEIGHT*3);
    }
    
    if (depth == NULL || depth_t == NULL || depth_s == NULL || (RGB_ON && (rgb == NULL || rgb_s == NULL || rgb_t == NULL))) {
        printf("Error allocating memory for depth or rgb\n");
        return 666;
    }
    
    // If using kinect, start device and get live depth information
    if (kinect_mode) {
        if (!kinect_start()) {
            printf("Error starting comunication with Kinect.\n");
            return 1;
        }
        
        if (pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL)) {
            printf("pthread_create failed\n");
            kinect_shutdown();
            return 2;
        }
        
    }
    
    // If not using live kinect, get static depth image from specified file
    else {
        FILE *f;
        int d;
        
        f = fopen(argv[1], "r");
        if (f == NULL) {
            printf("Couldnt open depth file '%s'.\n", argv[1]);
            return 3;
        }
        
        printf("Importing static depth information from file '%s'\n", argv[1]);
        
        for (i=0;i<640*480;i++) {
            fscanf(f, "%d", &d);
            depth[i] = (uint16_t) d;
        }
        
		got_depth = 1;
        printf("Depth information imported.\n");
    }
    
    // Start detector thread
    if (DETECTOR_ON && pthread_create(&detector_thread, NULL, detector, NULL)) {
        printf("Error creating detector thread\n");
    }
    
    // Start OpenCV things
    if (CV_ON) {
        cv_initialize();

        // Inicia a thread para exibir a imagem RGB
        if (RGB_ON && pthread_create(&cv_thread[0], NULL, cv_rgb_thread, NULL)) {
            printf("Error creating cv rgb thread\n");
        }
        
        if (RECONSTRUCTOR_ON && pthread_create(&cv_thread[1], NULL, cv_reconstructor_thread, NULL)) {
            printf("Error creating cv rgb thread\n");
        }
        
        if (DEPTH_ON) cv_depth();
//        if (DEPTH_ON && pthread_create(&cv_thread[0], NULL, cv_depth_thread, NULL)) {
//            printf("Error creating cv depth thread\n");
//        }
        
    }
    
	// Espera as threads acabarem
	if (kinect_mode && pthread_join(freenect_thread, NULL)) printf("--ERRO: pthread_join() \n");
    if (CV_ON & RGB_ON && pthread_join(cv_thread[0], NULL)) printf("Error joining cv rgb thread\n");
    if (RECONSTRUCTOR_ON && pthread_join(cv_thread[1], NULL)) printf("Error joining cv reconstructor thread\n");

	
	encerrar(0);
	
	printf("-- Exiting...\n");
    return 0;    
}


