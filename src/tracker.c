#define MAX_NFRAMES 100
#define MAX_NPOINTS 1000
struct point_tracker {
	int last_frame;
	int nframes;
	int n[MAX_NFRAMES];
	float xyst[MAX_NFRAMES][MAX_NPOINTS][4];
};

void point_tracker_init(struct point_tracker *p, int nframes)
{
	assert(nframes <= MAX_NFRAMES);
	p->nframes = nframes;

	// init a whole cycle with no points
	for (int i = 0; i < p->nframes; i++)
		p->n[i] = 0;
	p->last_frame = 0;
}

#include <stdlib.h>

// API
void point_tracker_add_frame(struct point_tracker *p, float *xyst, int n)
{
	//fprintf(stderr, "ADD FRAME %d (%d) %d\n", p->last_frame, p->nframes, n);
	if (n > MAX_NPOINTS) {
		fprintf(stderr, "warning: tracker got %d points but "
				"MAX_NPOINTS is %d\n", n, MAX_NPOINTS);
		n = MAX_NPOINTS;
	}
	p->last_frame = (1 + p->last_frame) % p->nframes;
	p->n[p->last_frame] = n;
	for (int i = 0; i < n; i++)
	for (int k = 0; k < 4; k++)
		p->xyst[p->last_frame][i][k] = xyst[4*i+k];
}

static float dist(float *a, float *b)
{
	return hypot(a[0] - b[0], a[1] - b[1]);
}

static int comes_from_the_past_p(struct point_tracker *p, float *xyst)
{
	for (int i = 0; i < p->nframes; i++)
	for (int j = 0; j < p->n[i]; j++)
	if (i != p->last_frame)
	if (dist(xyst, p->xyst[i][j]) < 8.5)
		return true;
	return false;
}

static int comes_from_the_past(struct point_tracker *p, int idx)
{
	float *xyst = p->xyst[p->last_frame][idx];
	return comes_from_the_past_p(p, xyst);
}


// API
int point_tracker_extract_points_old(float *out_xyst, struct point_tracker *p,
		float hysteresis_hi /*, float ... */ )
{
	int n_out = 0;
	for (int i = 0; i < p->n[p->last_frame]; i++)
	{
		float *xyst = p->xyst[p->last_frame][i];
		if (xyst[3] > hysteresis_hi || comes_from_the_past(p, i))
		{
			for (int k = 0; k < 4; k++)
				out_xyst[4*n_out+k] = xyst[k];
			n_out += 1;
		} else {
			xyst[3] = -INFINITY;
		}
	}
	return n_out;
}

static int appears_later(struct point_tracker *p, int frame_idx, int point_idx)
{
	if (frame_idx == p->last_frame)
		return false;
	float *xyst = p->xyst[frame_idx][point_idx];
	for (int jj = 0; jj < p->nframes; jj++)
	{
		int j = (frame_idx + jj + 1) % p->nframes;
		for (int i = 0; i < p->n[j]; i++)
		{
			if (dist(xyst,p->xyst[j][i]) < 3.5*xyst[2])
				return true;
		}
		if (j == p->last_frame)
			break;
	}
	return false;
}

// API
void point_tracker_add_frame_t(struct point_tracker *p, float *xyst, int n,
		float hysteresis_hi)
{
	fprintf(stderr, "ADD FRAME %d (%d) %d {%g}\n", p->last_frame, p->nframes, n, hysteresis_hi);
	if (n > MAX_NPOINTS) {
		fprintf(stderr, "warning: tracker got %d points but "
				"MAX_NPOINTS is %d\n", n, MAX_NPOINTS);
		n = MAX_NPOINTS;
	}
	p->last_frame = (1 + p->last_frame) % p->nframes;

	int cx = 0;
	for (int i = 0; i < n; i++)
	{
		float *X = xyst + 4*i;
		if (X[3] > hysteresis_hi || comes_from_the_past_p(p, X))
		{
			for (int k = 0; k < 4; k++)
				p->xyst[p->last_frame][cx][k] = X[k];
			cx += 1;
		}
	}
	p->n[p->last_frame] = cx;
}



int point_tracker_extract_points(float *out_xyst, struct point_tracker *p)
{
	int n_out = 0;
	for (int j = 0; j < p->nframes; j++)
	{
		fprintf(stderr, "%d%c:%d ", j, j==p->last_frame?'\'':'_',
				p->n[j]);
		for (int i = 0; i < p->n[j]; i++)
		{
			if (!appears_later(p, j, i))
			{
				float *xyst = p->xyst[j][i];
				for (int k = 0; k < 4; k++)
					out_xyst[4*n_out+k] = xyst[k];
				n_out += 1;
			}
		}
	}
	fprintf(stderr, "\n");
	return n_out;
}
