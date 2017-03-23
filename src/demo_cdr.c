#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// opencv includes (just for reading the webcam data)
#include "cv.h"
#include "highgui.h"


//#include "mauricio.c"         // function to compute Mauricio's blur detection
//#include "harressian.c"       // computation of Harris keypoints
//#include "ransac.c"           // generic ransac algorithm
//#include "geometry.c"         // linear algebra and geometric computations

#include "fontu.c"            // bitmap font library
#include "seconds.c"          // function for computing running times
#include "xmalloc.c"          // retargetable malloc
//#include "drawing.c"          // functions for drawing figures on fRGB images


struct bitmap_font global_font; // globally visible fixed-width font (for HUD)


// parameters of the algorithm (global variables, for easy testing)

int    global_mode = 0;
char  *global_name = "(nothing)";
double global_alpha_gain = 3;
double global_sigma = 0.5;

int    global_pyramid = 0;

double global_harris_sigma = 1;    // s
double global_harris_k = 0.24;     // k
double global_harris_flat_th = 20; // t
//double global_harris_flat_th = 200; // t
int    global_harris_neigh = 1;    // n

int    global_ransac_ntrials = 300; // r
int    global_ransac_minliers = 22;    // i
double global_ransac_maxerr = 1.5;    // e

double global_mauricio_ssat = 200;
double global_mauricio_gth = 40.00;
int    global_mauricio_Cth = 3000;
double global_mauricio_Sth = 20.0;

double global_tracker_toggle = 1;

void rich_man_gaussian_filter(float *out, float *in, int w, int h, float sigma)
{
	if (sigma == 0)
	{
		for (int i = 0; i < w*h; i++)
			out[i] = in[i];
		return;
	}

	// build 5x5 approximation of gaussian kernel
	float k0 = 1;
	float k1 = exp(-1/(2*sigma*sigma));
	float k2 = exp(-4/(2*sigma*sigma));
	float kd = exp(-2/(2*sigma*sigma));
	float kD = exp(-8/(2*sigma*sigma));
	float kh = exp(-5/(2*sigma*sigma));
	float kn = k0 + 4*k1 + 4*k2 + 4*kd + 4*kD + 8*kh;
	float k[5][5] = {
		{kD, kh, k2, kh, kD},
		{kh, kd, k1, kd, kh},
		{k2, k1, k0, k1, k2},
		{kh, kd, k1, kd, kh},
		{kD, kh, k2, kh, kD},
	};

	// hand-made convolution
	for (int j = 2; j < h - 2; j ++)
	for (int i = 2; i < w - 2; i ++)
	{
		float ax = 0;
		for (int dj = 0; dj < 5; dj++)
		for (int di = 0; di < 5; di++)
		{
			int ii = i + di - 2;
			int jj = j + dj - 2;
			ax += k[dj][di] * in[ii+jj*w];
		}
		out[i+j*w] = ax / kn;
	}
}


// process one (float rgb) frame
static
void process_frgb_frames(float *out, float *in_A, float *in_P, int w, int h)
{
	double framerate = seconds();

	// convert image to gray (and put it into rafa's image structure)
	float *gray0_A = xmalloc_float(w*h);
	float *gray0_P = xmalloc_float(w*h);
	float *gray_A = xmalloc_float(w*h);
	float *gray_P = xmalloc_float(w*h);
	float *gray_out = xmalloc_float(w*h);
	float *color_out = xmalloc_float(3*w*h);
	for (int j = 0; j < h; j++)
	for (int i = 0; i < w; i++)
	{
		int idx = j*w + i;
		{
			float r = in_A[3*idx+0];
			float g = in_A[3*idx+1];
			float b = in_A[3*idx+2];
			gray0_A[idx] = (r + g + b) / 3;
		}
		{
			float r = in_P[3*idx+0];
			float g = in_P[3*idx+1];
			float b = in_P[3*idx+2];
			gray0_P[idx] = (r + g + b) / 3;
		}
	}
	rich_man_gaussian_filter(gray_A, gray0_A, w, h, global_sigma);
	rich_man_gaussian_filter(gray_P, gray0_P, w, h, global_sigma);

	float alpha = 1;
	float beta = 0;
	// gray_out = operator(gray_A, gray_P)
	if (false) { ;
	} else if (global_mode == 0) { // raw image
		global_name = "A";
		for (int i = 0; i < w*h; i++)
			gray_out[i] = gray_A[i];
	} else if (global_mode == 1) { // temporal derivative
		global_name = "At";
		for (int i = 0; i < w*h; i++)
			gray_out[i] = gray_A[i] - gray_P[i];
		beta = 127;
	} else if (global_mode == 2) { // x-derivative
		global_name = "Ax";
		for (int j = 0; j < h; j++)
		for (int i = 0; i < w-1; i++)
			gray_out[w*j+i] = gray_A[w*j+i+1] - gray_A[w*j+i];
		beta = 127;
	} else if (global_mode == 3) { // y-derivative
		global_name = "Ay";
		for (int j = 0; j < h-1; j++)
		for (int i = 0; i < w; i++)
			gray_out[w*j+i] = gray_A[w*(j+1)+i] - gray_A[w*j+i];
		beta = 127;
	} else if (global_mode == 4) { // gradient-norm
		global_name = "(Ax^2 + Ay^2)^(1/2)";
		for (int j = 0; j < h-1; j++)
		for (int i = 0; i < w-1; i++)
		{
			float dx = gray_A[w*j+i+1]   - gray_A[w*j+i];
			float dy = gray_A[w*(j+1)+i] - gray_A[w*j+i];
			gray_out[w*j+i] = hypot(dx, dy);
		}
		beta = 4;
	} else if (global_mode == 5) { // laplacian
		global_name = "Axx + Ayy";
		for (int j = 1; j < h-1; j++)
		for (int i = 1; i < w-1; i++)
		{
			float r =
				4*gray_A[w*(j+0)+(i+0)]
				- gray_A[w*(j+1)+(i+0)]
				- gray_A[w*(j+0)+(i+1)]
				- gray_A[w*(j-1)+(i+0)]
				- gray_A[w*(j+0)+(i-1)]
				;
			gray_out[w*j+i] = r;
		}
		beta = 127;
	} /*else if (global_mode == 6) { // gradient vector
		global_name = "(Ax, Ay)";
		for (int j = 1; j < h-1; j++)
		for (int i = 1; i < w-1; i++)
		{
			float r =
				4*gray_A[w*(j+0)+(i+0)]
				- gray_A[w*(j+1)+(i+0)]
				- gray_A[w*(j+0)+(i+1)]
				- gray_A[w*(j-1)+(i+0)]
				- gray_A[w*(j+0)+(i-1)]
				;
			gray_out[w*j+i] = r;
		}
		beta = 127;
	} else if (global_mode == 7) { // nagel enkelman local flow
		global_name = "(Ayy*Atx - Axy*Aty , Axx*Aty - Axy*Atx)/(Axx*Ayy-Axy^2)";
		for (int j = 1; j < h-1; j++)
		for (int i = 1; i < w-1; i++)
		{
			float r =
				4*gray_A[w*(j+0)+(i+0)]
				- gray_A[w*(j+1)+(i+0)]
				- gray_A[w*(j+0)+(i+1)]
				- gray_A[w*(j-1)+(i+0)]
				- gray_A[w*(j+0)+(i-1)]
				;
			gray_out[w*j+i] = r;
		}
		beta = 127;
	} */else if (global_mode == 6) { // temporal average
		global_name = "(A + B)/2";
		for (int i = 0; i < w*h; i++)
			gray_out[i] = 0.5*(gray_A[i] + gray_P[i]);
	}

	// fill-in output gray values (for visualization)
	if (global_mode != 0 && global_mode != 6)
		alpha *= global_alpha_gain;
	for (int j = 0; j < h; j++)
	for (int i = 0; i < w; i++)
	{
		int idx = j*w + i;
		//out[3*idx+0] = 127 + 0.5*(in_A[3*idx+0] - in_P[3*idx+0]);
		//out[3*idx+1] = 127 + 0.5*(in_A[3*idx+1] - in_P[3*idx+1]);
		//out[3*idx+2] = 127 + 0.5*(in_A[3*idx+2] - in_P[3*idx+2]);
		out[3*idx+0] = beta + alpha * gray_out[idx];
		out[3*idx+1] = beta + alpha * gray_out[idx];
		out[3*idx+2] = beta + alpha * gray_out[idx];
	}

	free(gray0_A);
	free(gray0_P);
	free(gray_A);
	free(gray_P);
	free(gray_out);
	free(color_out);

//	// draw HUD
	char buf[1000];
	//snprintf(buf, 1000, "id = %d\nmode = %s", global_mode, global_name");
	snprintf(buf, 1000, "%s", global_name);
//	snprintf(buf, 1000, "sigma = %g\nk=%g\nt=%g\nn=%d",
//			global_harris_sigma,
//			global_harris_k,
//			global_harris_flat_th,
//			global_harris_neigh);
	float fg[] = {0, 255, 0};//, red[] = {0, 0, 255};
	float fg2[] = {100, 0, 150};
	put_string_in_float_image(out,w,h,3, 5,5, fg, 0, &global_font, buf);
//	snprintf(buf, 1000, "ransac ntrials = %d\nransac minliers = %d\n"
//			"ransac maxerr = %g",
//			global_ransac_ntrials,
//			global_ransac_minliers,
//			global_ransac_maxerr);
//	put_string_in_float_image(out,w,h,3, 155,5, fg, 0, &global_font, buf);
//
////	//snprintf(buf, 1000, "mauricio: %s", mauricio?"sharp":"blurred");
////	put_string_in_float_image(out,w,h,3, 355,18,
////			mauricio?fg:red, 0, &global_font, "mauricio: disabled");
//
//	put_string_in_float_image(out,w,h,3, 355,31, fg, 0, &global_font,
//	       	global_tracker_toggle?"tracker: ENABLED":"tracker: disabled");
//
	framerate = seconds() - framerate;
	snprintf(buf, 1000, "%g Hz\nalpha = %g\nsigma = %g",
			1/framerate,
			global_alpha_gain,
			global_sigma
			);
	put_string_in_float_image(out,w,h,3, 355,5, fg2, 0, &global_font, buf);
}

#ifdef ENABLE_SCREENSHOTS
#include <sys/types.h>
#include <unistd.h>
#include "iio.h"
static void save_screenshot(float *x, int w, int h)
{
	static int idx = 0;
	int pid = getpid();
	char fname[FILENAME_MAX];
	snprintf(fname, FILENAME_MAX, "/tmp/camflow_shot_%d_%d.png", pid, idx);
	fprintf(stderr, "saving screenshot on file %s\n", fname);
	iio_save_image_float_vec(fname, x, w, h, 3);
	idx += 1;
}
#endif

static uint8_t float_to_byte(float x)
{
	float ix = round(x);
	if (ix < 0) return 0;
	if (ix > 255) return 255;
	return ix;
}

static int my_mod(int x, int m)
{
	int r = x % m;
	return r < 0 ? r + m : r;
}

int main( int argc, char *argv[] )
{
	if (argc != 2)
		return fprintf(stderr, "usage:\n\t%s <cam_id>\n", *argv);
	int cam_id = atoi(argv[1]);

	global_font = uncompress_font(*xfont_8x13); // prepare font for HUD

	// interactivity state
	CvCapture *capture = 0;
	int       key = 0;

	/* initialize camera */
	capture = cvCaptureFromCAM( cam_id );
	//cvCaptureFromCAM
	//capture.set(CVCAP_IMAGE_WIDTH, 1920);

	/* always check */
	if ( !capture )
		fail("could not get a capture");

	IplImage *frame = cvQueryFrame(capture);
	if (!frame) fail("did not get frame");
	int w = frame->width, W = 512;
	int h = frame->height, H = 512;
	int pd = frame->nChannels;
	int depth = frame->depth;
	fprintf(stderr, "%dx%d %d [%d]\n", w, h, pd, depth);
	if (w != 640 || h != 480 || pd != 3)
		fail("unexpected webcam size, "
				"please change some hard-coded numbers");

	//if (W > w || H > h) fail("bad crop");
	CvSize size;
	size.width = W;
	size.height = H;
	IplImage *frame_small = cvCreateImage(size, depth, pd);
	//IplImage *frame_big = cvCreateImage(size, depth, pd);
	//fprintf(stderr, "%dx%d %d [%d]\n", frame_big->width, frame_big->height, pd, depth);

	float *frgb_A = xmalloc_float(W*H*pd);
	float *frgb_P = xmalloc_float(W*H*pd);
	//float *frgb_in = xmalloc_float(W*H*pd);
	float *frgb_out = xmalloc_float(W*H*pd);
	for (int i = 0; i < W*H; i++) {
		int g = 0;//rand()%0x100;
		frgb_A[3*i+0] = frgb_P[3*i+0] = g;
		frgb_A[3*i+1] = frgb_P[3*i+1] = g;
		frgb_A[3*i+2] = frgb_P[3*i+2] = g;
	}

	/* create a window for the video */
	cvNamedWindow( "result", CV_WINDOW_FREERATIO );
	cvResizeWindow("result", W, H);

	while( key != 'q' ) {
		/* get a frame */
		frame = cvQueryFrame( capture );

		/* always check */
		if( !frame ) break;

		if (frame->width != w) fail("got bad width");
		if (frame->height != h) fail("got bad height");
		if (frame->nChannels != pd) fail("got bad nc");
		if (frame->depth != depth) fail("got bad depth");
		if (pd != 3) fail("pd is not 3");

		for (int i = 0; i < 3*W*H; i++)
			frgb_P[i] = frgb_A[i];
		for (int j = 0; j < 384; j++)
		for (int i = 0; i < 512; i++)
		for (int l = 0; l < 3; l++)
			frgb_A[((j+64)*512+i)*3+l] = (float)(unsigned char)
				frame->imageData[((j+48)*w+i+64)*3+l];

		process_frgb_frames(frgb_out, frgb_A, frgb_P, W, H);

		frgb_out[0]=frgb_out[1]=frgb_out[2]=0;
		frgb_out[3]=frgb_out[4]=frgb_out[5]=255;

		for (int i = 0; i < W * H * pd; i++)
			frame_small->imageData[i] = float_to_byte(frgb_out[i]);

		cvShowImage( "result", frame_small );

		/* exit if user press 'q' */
		key = cvWaitKey( 1 ) % 0x10000;
		double wheel_factor = 1.1;
		if (key == 's') global_sigma /= wheel_factor;
		if (key == 'S') {
			if (!global_sigma) global_sigma = 0.5;
			else global_sigma *= wheel_factor;
		}
		if (key == 'k') global_harris_k /= pow(wheel_factor,1/32.0);
		if (key == 'K') global_harris_k *= pow(wheel_factor,1/32.0);
		if (key == 'a') global_alpha_gain /= wheel_factor;
		if (key == 'A') global_alpha_gain *= wheel_factor;
		if (key == 't') global_harris_flat_th /= wheel_factor;
		if (key == 'T') global_harris_flat_th *= wheel_factor;
		if (key == 'n' && global_harris_neigh > 0)
			global_harris_neigh -= 1;
		if (key == 'N') global_harris_neigh += 1;
		if (key == 'r' && global_ransac_ntrials > 10)
			global_ransac_ntrials /= wheel_factor;
		if (key == 'R') global_ransac_ntrials *= wheel_factor;
		if (key == 'i' && global_ransac_minliers > 2)
			global_ransac_minliers -= 1;
		if (key == 'I') global_ransac_minliers += 1;
#ifdef ENABLE_SCREENSHOTS
		if (key == ',') save_screenshot(frgb_out, W, H);
#endif
		if (key == 'e') global_ransac_maxerr /= wheel_factor;
		if (key == 'E') global_ransac_maxerr *= wheel_factor;
		if (key == 'w') global_harris_k *= -1;
		if (key == 'p') global_pyramid = !global_pyramid;
		if (key == 'z') global_tracker_toggle = !global_tracker_toggle;
		if (key == ' ')   global_mode = my_mod(global_mode + 1, 7);
		if (key == 'm')   global_mode = my_mod(global_mode + 1, 7);
		if (key == 'M')   global_mode = my_mod(global_mode - 1, 7);
		if (key == 65288) global_mode = my_mod(global_mode - 1, 7);
		if (key == '1') {
			global_alpha_gain = 1;
			global_sigma = 0;
		}
		if (key == '0') global_sigma = 0;
		if (key > 0) printf("key = %d\n", key);
		//if (isalpha(key)) {
		//	printf("harris_sigma = %g\n", global_harris_sigma);
		//	printf("harris_k = %g\n", global_harris_k);
		//	printf("harris_t = %g\n", global_harris_flat_th);
		//	printf("harris_n = %d\n", global_harris_neigh);
		//	printf("\n");
		//}
	}

	/* free memory */
	cvDestroyWindow( "result" );
	cvReleaseCapture( &capture );

	return 0;
}
