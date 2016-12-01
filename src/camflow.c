#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// opencv includes (just for reading the webcam data)
#include "cv.h"
#include "highgui.h"


#include "harressian.c"       // computation of Harris keypoints
#include "ransac.c"           // generic ransac algorithm
#include "geometry.c"         // linear algebra and geometric computations

#include "fontu.c"            // bitmap font library
#include "seconds.c"          // function for computing running times
#include "xmalloc.c"          // retargetable malloc

#include "drawing.c"          // functions for drawing figures on fRGB images

struct bitmap_font global_font; // globally visible fixed-width font (for HUD)


// parameters of the algorithm (global variables, for easy testing)

double global_harris_sigma = 1;    // s
double global_harris_k = 0.24;     // k
double global_harris_flat_th = 20; // t
int    global_harris_neigh = 1;    // n

int    global_ransac_ntrials = 1000; // r
int    global_ransac_minliers = 22;    // i
double global_ransac_maxerr = 1.5;    // e

double global_mauricio_ssat = 200;
double global_mauricio_gth = 40.00;
int    global_mauricio_Cth = 3000;
double global_mauricio_Sth = 20.0;
#include "mauricio.c"         // function to compute Mauricio's blur detection


int find_straight_line_by_ransac(int *out_mask, float line[3],
		float *points, int npoints,
		int ntrials, float max_err)
{
	if (npoints < 2)
		return 0;
	return ransac(out_mask, line, points, 2, npoints, 3,
			distance_of_point_to_straight_line,
			straight_line_through_two_points,
			2, ntrials, 3, max_err, NULL, NULL);
}

// process one (float rgb) frame
static void process_frgb_frame(float *out, float *in, int w, int h)
{
	double framerate = seconds();

	// convert image to gray (and put it into rafa's image structure)
	float *gray = xmalloc_float(w*h);
	for (int j = 0; j < h; j++)
	for (int i = 0; i < w; i++)
	{
		int idx = j*w + i;
		float r = in[3*idx+0];
		float g = in[3*idx+1];
		float b = in[3*idx+2];
		gray[idx] = (r + g + b) / 3;
	}

	// compute mauricio test
	bool mauricio = compute_mauricio(gray, w, h);

	// computi harris-hessian
	int max_keypoints = 2000;
	float point[3*max_keypoints];
	double tic = seconds();
	int npoints = harressian_ms(point, max_keypoints, gray, w, h,
			global_harris_sigma,
			global_harris_k,
			global_harris_flat_th);
	tic = seconds() - tic;
	int phist[30]; for (int i = 0; i < 30; i++) phist[i] = 0;
	for (int i = 0; i < npoints; i++) {
		int si = round(log2(point[3*i+2]));
		if (si < 30) phist[si] += 1;
	}
	fprintf(stderr, "npoints = %d:", npoints);
	for (int i = 0; i < 8; i++)
		fprintf(stderr, "\t%d", phist[i]);
	fprintf(stderr, "\n");
	//fprintf(stderr, "harris took %g milliseconds (%g hz)\n",
	//		tic*1000, 1/tic);

	// fill-in gray values (for visualization)
	for (int j = 0; j < h; j++)
	for (int i = 0; i < w; i++)
	{
		int idx = j*w + i;
		out[3*idx+0] = gray[idx];
		out[3*idx+1] = gray[idx];
		out[3*idx+2] = gray[idx];
	}

	// interior and exterior color for blob display
	float cin[3], cou[3];
	if (global_harris_k > 0) {
		cin[0] = 0; cin[1] = 255; cin[2] = 0; // green
		cou[0] = 0; cou[1] = 0; cou[2] = 255; // red
	} else {
		cin[0] = 255; cin[1] = 0; cin[2] = 0; // blue
		cou[0] = 0; cou[1] = 255; cou[2] = 0; // green
	}
	for (int i = 0; i < npoints; i++)
	{
		float x = point[3*i+0];
		float y = point[3*i+1];
		float radius = sqrt(2)*point[3*i+2];
		//overlay_rectangle_rgb(out,w,h, x-radius, y-radius,
		//		x+radius, y+radius, cin[0], cin[1], cin[2]);
		//overlay_rectangle_rgb(out,w,h, x-radius+1, y-radius+1,
		//		x+radius-1, y+radius-1, cou[0], cou[1], cou[2]);
		overlay_circle_rgb(out,w,h, x,y,radius+0, cin[0],cin[1],cin[2]);
		overlay_circle_rgb(out,w,h, x,y,radius+1, cou[0],cou[1],cou[2]);
	}

	// compute ransac
	if (npoints > 1)
	{
		// data for ransac
		int n = npoints;
		int *mask = xmalloc_int(n);
		float *keypoints = xmalloc_float(2 * n);
		for (int i = 0; i < n; i++)
		for (int l = 0; l < 2; l++)
			keypoints[2*i+l] = point[3*i+l];

		for (int i = 0; i < 10; i++)
		{
			// find line
			float line[3];
			int n_inliers = find_straight_line_by_ransac(mask, line,
					keypoints, n,
					global_ransac_ntrials,
					global_ransac_maxerr);
			if (n_inliers < global_ransac_minliers)
				break;
			double dline[3] = {line[0], line[1], line[2]};
			double rectangle[4] = {0, 0, w, h};
			double segment[4];
			cut_line_with_rectangle(segment, segment+2,
					dline, rectangle, rectangle+2);
			int ifrom[2] = {round(segment[0]), round(segment[1])};
			int ito[2] = {round(segment[2]), round(segment[3])};
			float fred[3] = {0, 0, 255};
			draw_segment_frgb(out, w, h, ifrom, ito, fred);
			// exclude the points of this segment
			int cx = 0;
			for (int i = 0; i < n; i++)
				if (!mask[i]) // keep only the unused points
				{
					keypoints[2*cx+0] = keypoints[2*i+0];
					keypoints[2*cx+1] = keypoints[2*i+1];
					cx++;
				}
			assert(cx + n_inliers == n);
			n = cx;
		}

		// cleanup
		free(keypoints);
		free(mask);
	}

	free(gray);

	// draw HUD
	char buf[1000];
	snprintf(buf, 1000, "sigma = %g\nk=%g\nt=%g\nn=%d",
			global_harris_sigma,
			global_harris_k,
			global_harris_flat_th,
			global_harris_neigh);
	float fg[] = {0, 255, 0}, red[] = {0, 0, 255};
	put_string_in_float_image(out,w,h,3, 5,5, fg, 0, &global_font, buf);
	snprintf(buf, 1000, "ransac ntrials = %d\nransac minliers = %d\n"
			"ransac maxerr = %g",
			global_ransac_ntrials,
			global_ransac_minliers,
			global_ransac_maxerr);
	put_string_in_float_image(out,w,h,3, 155,5, fg, 0, &global_font, buf);

	framerate = seconds() - framerate;
	snprintf(buf, 1000, "%g Hz", 1/framerate);
	put_string_in_float_image(out,w,h,3, 355,5, fg, 0, &global_font, buf);
	//snprintf(buf, 1000, "mauricio: %s", mauricio?"sharp":"blurred");
	put_string_in_float_image(out,w,h,3, 355,18,
			mauricio?fg:red, 0, &global_font, "mauricio: disabled");
}

#ifdef ENABLE_SCREENSHOTS
#include <sys/types.h>
#include <unistd.h>
#include <iio.h>
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

	float *frgb_in = xmalloc_float(W*H*pd);
	float *frgb_out = xmalloc_float(W*H*pd);
	for (int i = 0; i < W*H; i++) {
		int g = 0;//rand()%0x100;
		frgb_in[3*i+0] = g;
		frgb_in[3*i+1] = g;
		frgb_in[3*i+2] = g;
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

		for (int j = 0; j < 384; j++)
		for (int i = 0; i < 512; i++)
		for (int l = 0; l < pd; l++)
			frgb_in[((j+64)*512+i)*pd+l] = (float)(unsigned char)
				frame->imageData[((j+48)*w+i+64)*pd+l];

		process_frgb_frame(frgb_out, frgb_in, W, H);

		frgb_out[0]=frgb_out[1]=frgb_out[2]=0;
		frgb_out[3]=frgb_out[4]=frgb_out[5]=255;

		for (int i = 0; i < W * H * pd; i++)
			frame_small->imageData[i] = frgb_out[i];

		cvShowImage( "result", frame_small );

		/* exit if user press 'q' */
		key = cvWaitKey( 1 ) % 0x10000;
		double wheel_factor = 1.1;
		if (key == 's') global_harris_sigma /= wheel_factor;
		if (key == 'S') global_harris_sigma *= wheel_factor;
		if (key == 'k') global_harris_k /= pow(wheel_factor,1/32.0);
		if (key == 'K') global_harris_k *= pow(wheel_factor,1/32.0);
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
