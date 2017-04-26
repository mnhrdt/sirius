// auxiliary functions for drawing figures on fRGB images

// check wether a point is inside the image domain
static int insideP(int w, int h, int i, int j)
{
	return i >= 0 && j >= 0 && i < w && j < h;
}

// generic function to traverse a segment between two pixels
void traverse_segment(int px, int py, int qx, int qy,
		void (*f)(int,int,void*), void *e)
{
	if (px == qx && py == qy)
		f(px, py, e);
	else if (qx + qy < px + py) // bad quadrants
		traverse_segment(qx, qy, px, py, f, e);
	else {
		if (qx - px > qy - py || px - qx > qy - py) { // horizontal
			float slope = (qy - py)/(float)(qx - px);
			for (int i = 0; i < qx-px; i++)
				f(i+px, lrint(py + i*slope), e);
		} else { // vertical
			float slope = (qx - px)/(float)(qy - py);
			for (int j = 0; j <= qy-py; j++)
				f(lrint(px + j*slope), j+py, e);
		}
	}
}


struct drawing_state {
	int w, h;
	float *color;
	float *frgb;
};

static void plot_frgb_pixel(int i, int j, void *ee)
{
	struct drawing_state *e = (struct drawing_state *)ee;
	if (!insideP(e->w, e->h, i, j))
		return;
	e->frgb[3*(j * e->w + i) + 0] = e->color[0];
	e->frgb[3*(j * e->w + i) + 1] = e->color[1];
	e->frgb[3*(j * e->w + i) + 2] = e->color[2];
}

static void draw_segment_frgb(float *frgb, int w, int h,
		int from[2], int to[2], float color[3])
{
	struct drawing_state e;
	void *ee = (void*)&e;
	e.w = w;
	e.h = h;
	e.frgb = frgb;
	e.color = color;
	traverse_segment(from[0], from[1], to[0], to[1], plot_frgb_pixel, ee);
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

static void putcolor_frgb(float *x, int w, int h, int i, int j,
		float c1, float c2, float c3)
{
	if (insideP(w, h, i, j))
	{
		x[3*(j*w+i)+0] = c1;
		x[3*(j*w+i)+1] = c2;
		x[3*(j*w+i)+2] = c3;
	}
}

// paint a thin rectangle on a float rgb image
static void overlay_circle_rgb(float *out, int w, int h,
		float cx, float cy, float r, int c1, int c2, int c3)
{
	// top and bottom arcs
	for (int i = round(cx - r/sqrt(2)); i <= round(cx + r/sqrt(2)); i++)
	{
		float x = i + (cx-r/sqrt(2))-round(cx-r/sqrt(2));
		float ytop = cy + sqrt(r*r - (x - cx)*(x - cx));
		float ybot = cy - sqrt(r*r - (x - cx)*(x - cx));
		putcolor_frgb(out, w,h, i, round(ytop), c1,c2,c3);
		putcolor_frgb(out, w,h, i, round(ybot), c1,c2,c3);
	}
	// left and right arcs
	for (int j = round(cy - r/sqrt(2)); j <= round(cy + r/sqrt(2)); j++)
	{
		float y = j + (cy-r/sqrt(2))-round(cy-r/sqrt(2));
		float xleft = cx - sqrt(r*r - (y - cy)*(y - cy));
		float xrite = cx + sqrt(r*r - (y - cy)*(y - cy));
		putcolor_frgb(out, w, h, round(xleft), j, c1,c2,c3);
		putcolor_frgb(out, w, h, round(xrite), j, c1,c2,c3);
	}
}
