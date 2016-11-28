// auxiliary functions for drawing figures on fRGB images

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


// paint a thin rectangle on a float rgb image
static void overlay_rectangle_rgb(float *out, int w, int h,
		int ax, int ay, int bx, int by, int c1, int c2, int c3)
{
	if (bx < ax) bx = ax;
	if (by < ay) by = ay;
	assert(ax <= bx);
	assert(ay <= by);
	int i, j;
	for (i = ax; i <= bx; i++) // horizontal edges
	{
		j = ay; // top
		if (insideP(w, h, i, j))
		{
			out[3*(j*w+i)+0] = c1;
			out[3*(j*w+i)+1] = c2;
			out[3*(j*w+i)+2] = c3;
		}
		j = by; // bottom
		if (insideP(w, h, i, j))
		{
			out[3*(j*w+i)+0] = c1;
			out[3*(j*w+i)+1] = c2;
			out[3*(j*w+i)+2] = c3;
		}
	}
	for (j = ay; j <= by; j++) // vertical edges
	{
		i = ax; // left
		if (insideP(w, h, i, j))
		{
			out[3*(j*w+i)+0] = c1;
			out[3*(j*w+i)+1] = c2;
			out[3*(j*w+i)+2] = c3;
		}
		i = bx; // right
		if (insideP(w, h, i, j))
		{
			out[3*(j*w+i)+0] = c1;
			out[3*(j*w+i)+1] = c2;
			out[3*(j*w+i)+2] = c3;
		}
	}
}

