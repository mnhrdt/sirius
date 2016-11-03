/**
 * Display video from webcam
 *
 * Author  Nash
 * License GPL
 * Website http://nashruddin.com
 */

#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "cv.h"
#include "highgui.h"



#include "harris/harris.c"
#include "harris/gauss.c"
#include "harris/image.c"
#include "harris/ntuple.c"
#include "harris/misc.c"

#define OMIT_MAIN_FONTU
#include "fontu.c"
#include "fonts/xfont_9x15.c"
#include "fonts/xfont_8x13.c"

#include "seconds.c"


struct bitmap_font global_font;

double global_harris_sigma = 1;    // s
double global_harris_k = 0.04;     // k
double global_harris_flat_th = 20; // t
int    global_harris_neigh = 3;    // n

#include "xmalloc.c"

// process one frame
static void process_tacu(float *out, float *in, int w, int h, int pd)
{
	// convert image to gray
	image_double in_gray = new_image_double(w, h);
	for (int j = 0; j < h; j++)
	for (int i = 0; i < w; i++)
	{
		int idx = j*w + i;
		float r = in[3*idx+0];
		float g = in[3*idx+1];
		float b = in[3*idx+2];
		in_gray->data[idx] = (r + g + b)/3;
	}

	// fill-in gray values (for visualization)
	for (int j = 0; j < h; j++)
	for (int i = 0; i < w; i++)
	{
		int idx = j*w + i;
		out[3*idx+0] = in_gray->data[idx];
		out[3*idx+1] = in_gray->data[idx];
		out[3*idx+2] = in_gray->data[idx];
	}

	// computi harris-hessian
	double tic = seconds();
	ntuple_list hp = harris2(in_gray,
			global_harris_sigma,
			global_harris_k,
			global_harris_flat_th,
			global_harris_neigh
			);
	tic = seconds() - tic;
	fprintf(stderr, "harris took %g milliseconds (%g hz)\n",
			tic*1000, 1/tic);

	// plot detected keypoints
	int n[][2] = {
		{0,0},
	       	{-1,0}, {0,-1}, {0,1}, {1,0}, // 5
		{-1,-1}, {-1,+1}, {1,-1}, {1,+1} // 9
	}, nn = 9;

	for (int i = 0; i < hp->size; i++)
	{
		assert(hp->dim == 2);
		int x = hp->values[2*i+0];
		int y = hp->values[2*i+1];
		for (int p = 0; p < nn; p++)
		{
			int xx = x + n[p][0];
			int yy = y + n[p][1];
			int idx = yy*w + xx;
			if (idx < 0 || idx >= w*h) continue;
			out[3*idx + 0] = 0;
			out[3*idx + 1] = 255;
			out[3*idx + 2] = 0;
		}
		//int idx = y*w + x;
		//if (idx < 0 || idx >= w*h) continue;
		//out[3*idx + 0] = 0;
		//out[3*idx + 1] = 0;
		//out[3*idx + 2] = 255;
	}

	free_ntuple_list(hp);
	free_image_double(in_gray);

	// draw HUD
	char buf[1000];
	snprintf(buf, 1000, "sigma = %g\nk=%g\nt=%g\nn=%d",
			global_harris_sigma,
			global_harris_k,
			global_harris_flat_th,
			global_harris_neigh);
	float fg[] = {0, 255, 0};
	put_string_in_float_image(out,w,h,3, 5,5, fg, 0, &global_font, buf);
}

int main( int argc, char *argv[] )
{
	if (argc != 1)
		return EXIT_FAILURE;

	global_font = *xfont_8x13;
	global_font = reformat_font(global_font, UNPACKED);

	CvCapture *capture = 0;
	int accum_index = 0;
	int       key = 0;

	/* initialize camera */
	capture = cvCaptureFromCAM( 1 );
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

	float *taccu_in = xmalloc(W*H*pd*sizeof*taccu_in);
	float *taccu_out = xmalloc(W*H*pd*sizeof*taccu_in);
	for (int i = 0; i < W*H; i++) {
		int g = 0;//rand()%0x100;
		taccu_in[3*i+0] = g;
		taccu_in[3*i+1] = g;
		taccu_in[3*i+2] = g;
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

		//for (int i = 0; i < W * H * pd; i++)
		//{
		//	taccu_in[i] = (float)(unsigned char)frame->imageData[i];
		//}
		for (int j = 0; j < 384; j++)
		for (int i = 0; i < 512; i++)
		for (int l = 0; l < pd; l++)
			taccu_in[((j+64)*512+i)*pd+l] = (float)(unsigned char)
				frame->imageData[((j+48)*w+i+64)*pd+l];

		process_tacu(taccu_out, taccu_in, W, H, pd);

		taccu_out[0]=taccu_out[1]=taccu_out[2]=0;
		taccu_out[3]=taccu_out[4]=taccu_out[5]=255;

		for (int i = 0; i < W * H * pd; i++)
			frame_small->imageData[i] = taccu_out[i];

		cvShowImage( "result", frame_small );

		/* exit if user press 'q' */
		key = cvWaitKey( 1 ) % 0x10000;
		double wheel_factor = 1.1;
		if (key == 's') global_harris_sigma /= wheel_factor;
		if (key == 'S') global_harris_sigma *= wheel_factor;
		if (key == 'k') global_harris_k /= wheel_factor;
		if (key == 'K') global_harris_k *= wheel_factor;
		if (key == 't') global_harris_flat_th /= wheel_factor;
		if (key == 'T') global_harris_flat_th *= wheel_factor;
		if (key == 'n' && global_harris_neigh > 1)
			global_harris_neigh -= 1;
		if (key == 'N') global_harris_neigh += 1;
		if (isalpha(key)) {
			printf("harris_sigma = %g\n", global_harris_sigma);
			printf("harris_k = %g\n", global_harris_k);
			printf("harris_t = %g\n", global_harris_flat_th);
			printf("harris_n = %d\n", global_harris_neigh);
			printf("\n");
		}
	}

	/* free memory */
	cvDestroyWindow( "result" );
	cvReleaseCapture( &capture );

	return 0;
}
