// implementation of the multiscale "harris hessian" keypoint detector

// INTERFACE
int harressian( // return value: number of detected points
		float *out_xys,  // pre-allocated output buffer
		int max_npoints, // maximun number of points to compute
		float *x,        // input image data
		int w,           // input image width
		int h,           // input image height
		float sigma,     // parameter: pre-filtering size (e.g. 1.0)
		float kappa,     // parameter: roundness threshold (e.g. 0.24)
		float tau        // parameter: deepness threshold (e.g. 20.0)
		);
// note: the output is an array of triplets with (x,y,s) of each detected blob
// note 2: if kappa is negative, find "light blobs" instead of "dark blobs"


// IMPLEMENTATION
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

static float *xmalloc_float(int n) // utility function
{
	float *r = malloc(n * sizeof*r);
	if (!r) // if out of memory, force a segfault to ease debugging
		return r+*(volatile int*)0;
	return r;
}

static void gaussian_smoothing(float *y, float *x, int w, int h, float s)
{
	// build 3x3 approximation of gaussian kernel "k"
	float k0 = 1;
	float k1 = exp(-1/(2*s*s));
	float k2 = exp(-sqrt(2)/(2*s*s));
	float kn = k0 + 4*k1 + 4*k2;
	float k[3][3] = {{k2, k1, k2}, {k1, k0, k1}, {k2, k1, k2}};

	// compute y = k * x
	for (int j = 1; j < h - 1; j ++)
	for (int i = 1; i < w - 1; i ++)
	{
		float ax = 0;
		for (int dj = 0; dj < 3; dj++)
		for (int di = 0; di < 3; di++)
		{
			int ii = i + di - 1;
			int jj = j + dj - 1;
			ax += k[dj][di] * x[ii+jj*w];
		}
		y[i+j*w] = ax / kn;
	}
}

// extrapolate by nearest value
static float getpixel(float *I, int w, int h, int i, int j)
{
	if (i < 0) i = 0;
	if (j < 0) j = 0;
	if (i >= w) i = w-1;
	if (j >= h) j = h-1;
	return I[i+j*w];
}

static float getlaplacian(float *I, int w, int h, int i, int j)
{
	return -4 * getpixel(I, w, h, i  , j  )
	          + getpixel(I, w, h, i+1, j  )
	          + getpixel(I, w, h, i  , j+1)
	          + getpixel(I, w, h, i-1, j  )
	          + getpixel(I, w, h, i  , j-1);
}

static void downsample(float *out, int ow, int oh,
		float *in, int iw, int ih)
{
	if (!out || !in) return;
	assert(abs(2*ow-iw) < 2);
	assert(abs(2*oh-ih) < 2);
	for (int j = 0; j < oh; j++)
	for (int i = 0; i < ow; i++)
		out[ow*j+i] = getpixel(in, iw, ih, 2*i, 2*j);
}

#define MAX_LEVELS 20
struct pyramid {
	int n;                // number of levels
	int w[MAX_LEVELS];    // width of each level
	int h[MAX_LEVELS];    // height of each level
	float *x[MAX_LEVELS]; // data of each level
};

static float pyramidal_laplacian(struct pyramid *p, float x, float y, int o)
{
	if (o < 0 || o >= p->n)
		return -INFINITY;
	float *I = p->x[o];
	int w = p->w[o];
	int h = p->h[o];
	return getlaplacian(I, w, h, round(x), round(y));
}

static void fill_pyramid(struct pyramid *p, float *x, int w, int h)
{
	float S = 2.8 / 2; // magic value! do not change

	int i = 0;
	p->w[i] = w;
	p->h[i] = h;
	p->x[i] = xmalloc_float(w * h);
	memcpy(p->x[i], x, w*h*sizeof*x);
	while (1) {
		i += 1;
		if (i + 1 >= MAX_LEVELS) break;
		p->w[i] = ceil(p->w[i-1]/2);
		p->h[i] = ceil(p->h[i-1]/2);
		if (p->w[i] <= 1 && p->h[i] <= 1) break;
		p->x[i] = xmalloc_float(p->w[i] * p->h[i]);
		float *t = xmalloc_float(p->w[i-1] * p->h[i-1]);
		gaussian_smoothing(t, p->x[i-1], p->w[i-1], p->h[i-1], S);
		downsample(p->x[i], p->w[i], p->h[i], t, p->w[i-1], p->h[i-1]);
		free(t);
	}
	p->n = i;
}

static void free_pyramid(struct pyramid *p)
{
	for (int i = 0; i < p->n; i++)
		free(p->x[i]);
}

static float parabolic_minimum(float p, float q, float r)
{
	if (isfinite(p) && isfinite(q) && isfinite(r))
	{
		float alpha = (p + r) / 2 - q;
		float beta  = (p - r) / 2;
		float R = 0.5 * beta / alpha;
		if (fabs(R) > 0.5) {
			if (p < r) return -1;
			return 1;
		}
		assert(fabs(R) <= 0.5);
		return R;
	}
	if ( isfinite(p) &&  isfinite(q) && !isfinite(r)) return p < q ? -1 : 0;
	if (!isfinite(p) &&  isfinite(q) &&  isfinite(r)) return q < r ? 0 : 1;
	if (!isfinite(p) &&  isfinite(q) && !isfinite(r)) return 0;
	if ( isfinite(p) && !isfinite(q) && !isfinite(r)) return -1;
	if (!isfinite(p) && !isfinite(q) &&  isfinite(r)) return 1;
	if (!isfinite(p) && !isfinite(q) && !isfinite(r)) return NAN;
	return NAN; // isfinite(p) && !isfinite(q) && isfinite(r) // impossible
}

static int harressian_nogauss(float *out_xy, int max_npoints,
		float *x, int w, int h, float kappa, float tau)
{
	float sign = kappa > 0 ? 1 : -1;
	kappa = fabs(kappa);
	int n = 0;
	for (int j = 2; j < h - 2; j++)
	for (int i = 2; i < w - 2; i++)
	{
		// Vmm V0m Vpm
		// Vm0 V00 Vp0
		// Vmp V0p Vpp
		float Vmm = sign * x[(i-1) + (j-1)*w];
		float V0m = sign * x[(i+0) + (j-1)*w];
		float Vpm = sign * x[(i+1) + (j-1)*w];
		float Vm0 = sign * x[(i-1) + (j+0)*w];
		float V00 = sign * x[(i+0) + (j+0)*w];
		float Vp0 = sign * x[(i+1) + (j+0)*w];
		float Vmp = sign * x[(i-1) + (j+1)*w];
		float V0p = sign * x[(i+0) + (j+1)*w];
		float Vpp = sign * x[(i+1) + (j+1)*w];
		if (V0m<V00 || Vm0<V00 || V0p<V00 || Vp0<V00
				|| Vmm<V00 || Vpp<V00 || Vmp<V00 || Vpm<V00)
			continue;
		float dxx = Vm0 - 2*V00 + Vp0;
		float dyy = V0m - 2*V00 + V0p;
		float dxy =  (Vpp + Vmm - Vpm - Vmp)/4;
		float dyx = -(Vpm + Vmp - Vpp - Vmm)/4;
		float T = dxx + dyy;
		float D = dxx * dyy - dxy * dyx;
		float R0 = D - kappa * T * T;
		if (T > tau && R0 > 0) {
			out_xy[2*n+0] = i + parabolic_minimum(Vm0, V00, Vp0);
			out_xy[2*n+1] = j + parabolic_minimum(V0m, V00, V0p);
			n += 1;
		}
		if (n >= max_npoints - 1)
			goto done;
	}
done:
	assert(n < max_npoints);
	return n;
}

int harressian(float *out_xys, int max_npoints, float *x, int w, int h,
		float sigma, float kappa, float tau)
{
	// filter input image
	float *sx = xmalloc_float(w * h);
	gaussian_smoothing(sx, x, w, h, sigma);

	// create image pyramid
	struct pyramid p[1];
	fill_pyramid(p, sx, w, h);
	float *tab_xy = xmalloc_float(2 * max_npoints);

	// apply nongaussian harressian at each level of the pyramid
	int n = 0;
	for (int l = p->n - 1; l >= 0; l--)
	{
		int n_l = harressian_nogauss(tab_xy, max_npoints - n,
				p->x[l], p->w[l], p->h[l], kappa, tau);
		float factor = 1 << l;
		for (int i = 0; i < n_l; i++)
		{
			if (n >= max_npoints) break;
			float x = tab_xy[2*i+0];
			float y = tab_xy[2*i+1];

			// first-order scale localization
			float A = fabs(pyramidal_laplacian(p, x/2, y/2, l+1));
			float B = fabs(pyramidal_laplacian(p, x, y, l));
			float C = fabs(pyramidal_laplacian(p, x*2, y*2, l-1));
			if (l > 0 && C > B) continue;
			if (A > B) continue;
			float factor_scaling = parabolic_minimum(-A, -B, -C);

			// save this point
			out_xys[3*n+0] = factor * x;
			out_xys[3*n+1] = factor * y;
			out_xys[3*n+2] = factor * (3*factor_scaling + 5) / 4;;
			n++;
		}
	}
	assert(n <= max_npoints);

	// cleanup and exit
	free(tab_xy);
	free_pyramid(p);
	free(sx);
	return n;
}
