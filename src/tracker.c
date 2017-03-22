#define MAX_NFRAMES 10
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

static int comes_from_the_past(struct point_tracker *p, int idx)
{
	float *xyst = p->xyst[p->last_frame][idx];
	for (int i = 0; i < p->nframes; i++)
	for (int j = 0; j < p->n[i]; j++)
	if (i != p->last_frame)
	if (dist(xyst, p->xyst[i][j]) < 2)
		return true;
	return false;
}

int point_tracker_extract_points(float *out_xyst, struct point_tracker *p,
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
		}
	}
	return n_out;
}
