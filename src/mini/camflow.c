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


struct bitmap_font global_font;

// parameters of the algorithm (for easy testing)

double global_harris_sigma = 1;    // s
double global_harris_k = 0.04;     // k
double global_harris_flat_th = 20; // t
int    global_harris_neigh = 1;    // n

int    global_ransac_ntrials = 1000; // r
int    global_ransac_minliers = 22;    // i
double global_ransac_maxerr = 1.5;    // e

double global_mauricio_ssat = 200;
double global_mauricio_gth = 40.00;
int    global_mauricio_Cth = 3000;
double global_mauricio_Sth = 20.0;


int find_straight_line_by_ransac(bool *out_mask, float line[3],
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

struct drawing_state {
	int w, h;
	float *color;
	float *frgb;
};

static void plot_frgb_pixel(int i, int j, void *ee)
{
	struct drawing_state *e = ee;
	if (!insideP(e->w, e->h, i, j))
		return;
	e->frgb[3*(j * e->w + i) + 0] = e->color[0];
	e->frgb[3*(j * e->w + i) + 1] = e->color[1];
	e->frgb[3*(j * e->w + i) + 2] = e->color[2];
}

static void draw_segment_frgb(float *frgb, int w, int h,
		int from[2], int to[2], float color[3])
{
	struct drawing_state e = {.w = w, .h = h, .frgb = frgb, .color = color};
	traverse_segment(from[0], from[1], to[0], to[1], plot_frgb_pixel, &e);
}

static bool compute_mauricio(float *x, int w, int h)
{
	// compute % of saturated pixels
	double sat_percent = 0;
	for (int i = 0; i < w*h; i++)
		if (x[i] > global_mauricio_ssat)
			sat_percent += 1;
	sat_percent /= w*h;
	sat_percent *= 100;

	// note: do not resample (simply pick the correct neighbors)
	// note: do not run expensive normalization (we'll see if it matters)

	// compute sobel statistics
	int num_cx_large = 0;
	int num_cy_large = 0;
	for (int j = 100; j < h-100; j += 2)
	for (int i = 100; i < w-100; i += 2)
	{
		// sobel neighborhood
		// Vmm V0m Vpm
		// Vm0 V00 Vp0
		// Vmp V0p Vpp
		double Vmm = x[(i-2) + (j-2)*w];
		double V0m = x[(i+0) + (j-2)*w];
		double Vpm = x[(i+2) + (j-2)*w];
		double Vm0 = x[(i-2) + (j+0)*w];
		//double V00 = x[(i+0) + (j+0)*w];
		double Vp0 = x[(i+2) + (j+0)*w];
		double Vmp = x[(i-2) + (j+2)*w];
		double V0p = x[(i+0) + (j+2)*w];
		double Vpp = x[(i+2) + (j+2)*w];
		double gx = Vpm - Vmm + 2*(Vp0 - Vm0) + Vpp - Vmp;
		double gy = Vmp - Vmm + 2*(V0p - V0m) + Vpp - Vpm;
		if (fabs(gx) > global_mauricio_gth) num_cx_large += 1;
		if (fabs(gy) > global_mauricio_gth) num_cy_large += 1;
	}
	//fprintf(stderr, "sat_percent, cxlarge, cylarge = %g %d %d\n",
	//		sat_percent, num_cx_large, num_cy_large);

	return
		(num_cx_large > global_mauricio_Cth) &&
		(num_cy_large > global_mauricio_Cth) &&
		(sat_percent < global_mauricio_Sth);
}

// process one (float rgb) frame
static void process_tacu(float *out, float *in, int w, int h)
{
	double framerate = seconds();

	// convert image to gray (and put it into rafa's image structure)
	float *gray = xmalloc(w*h*sizeof*gray);
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
	float point[2*max_keypoints];
	double tic = seconds();
	int npoints = harris(point, max_keypoints, gray, w, h,
			global_harris_sigma,
			global_harris_k,
			global_harris_flat_th, gray);
	fprintf(stderr, "npoints = %d\n", npoints);
	tic = seconds() - tic;
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

	// plot detected keypoints
	int n[][2] = {
		{0,0},
	       	{-1,0}, {0,-1}, {0,1}, {1,0}, // 5
		{-1,-1}, {-1,+1}, {1,-1}, {1,+1}, // 9
		{-2,0}, {2,0}, {0,-2}, {0,2}, // 13
		{-2,-1}, {2,-1}, {-1,-2}, {-1,2},
		{-2,1}, {2,1}, {1,-2}, {1,2} // 21
	}, nn = 21;

	for (int i = 0; i < npoints; i++)
	{
		int x = point[2*i+0];
		int y = point[2*i+1];
		for (int p = 0; p < nn; p++)
		{
			int xx = x + n[p][0];
			int yy = y + n[p][1];
			int idx = yy*w + xx;
			if (idx < 0 || idx >= w*h) continue;
			out[3*idx + 0] = 0;
			out[3*idx + 1] = p?255:0;
			out[3*idx + 2] = p?0:255;
		}
	}

	// compute ransac
	if (npoints > 1)
	{
		// data for ransac
		int n = npoints;
		bool *mask = xmalloc(n * sizeof*mask);

		for (int i = 0; i < 10; i++)
		{
			// find line
			float line[3];
			int n_inliers = find_straight_line_by_ransac(mask, line,
					point, n,
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
					point[2*cx+0] = point[2*i+0];
					point[2*cx+1] = point[2*i+1];
					cx++;
				}
			assert(cx + n_inliers == n);
			n = cx;
		}

		// cleanup
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
	snprintf(buf, 1000, "mauricio: %s", mauricio?"sharp":"blurred");
	put_string_in_float_image(out,w,h,3, 355,15,
			mauricio?fg:red, 0, &global_font, buf);
}

int main( int argc, char *argv[] )
{
	if (argc != 2)
		return fprintf(stderr, "usage:\n\t%s <cam_id>\n", *argv);
	int cam_id = atoi(argv[1]);

	global_font = *xfont_8x13;
	global_font = uncompress_font(global_font);

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

		process_tacu(taccu_out, taccu_in, W, H);

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
		if (key == 'n' && global_harris_neigh > 0)
			global_harris_neigh -= 1;
		if (key == 'N') global_harris_neigh += 1;
		if (key == 'r' && global_ransac_ntrials > 10)
			global_ransac_ntrials /= wheel_factor;
		if (key == 'R') global_ransac_ntrials *= wheel_factor;
		if (key == 'i' && global_ransac_minliers > 2)
			global_ransac_minliers -= 1;
		if (key == 'I') global_ransac_minliers += 1;
		if (key == 'e') global_ransac_maxerr /= wheel_factor;
		if (key == 'E') global_ransac_maxerr *= wheel_factor;
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
