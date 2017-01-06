// implementation of the "harris hessian" keypoint detector

#include <assert.h>
#include <math.h>
#include <string.h>
#include "xmalloc.c"

static
void poor_man_gaussian_filter(float *out, float *in, int w, int h, float sigma)
{
	// build 3x3 approximation of gaussian kernel
	float k0 = 1;
	float k1 = exp(-1/(2*sigma*sigma));
	float k2 = exp(-sqrt(2)/(2*sigma*sigma));
	float kn = k0 + 4*k1 + 4*k2;
	float k[3][3] = {{k2, k1, k2}, {k1, k0, k1}, {k2, k1, k2}};

	// hand-made convolution
	for (int j = 1; j < h - 1; j ++)
	for (int i = 1; i < w - 1; i ++)
	{
		float ax = 0;
		for (int dj = 0; dj < 3; dj++)
		for (int di = 0; di < 3; di++)
		{
			int ii = i + di - 1;
			int jj = j + dj - 1;
			ax += k[dj][di] * in[ii+jj*w];
		}
		out[i+j*w] = ax / kn;
	}
}

// extrapolate by nearest value (useful for Neumann boundary conditions)
static float getpixel_1(float *I, int w, int h, int i, int j)
{
	if (i < 0) i = 0;
	if (j < 0) j = 0;
	if (i >= w) i = w-1;
	if (j >= h) j = h-1;
	return I[i+j*w];
}

static float getlaplacian_1(float *I, int w, int h, int i, int j)
{
	return -4 * getpixel_1(I, w, h, i  , j  )
	          + getpixel_1(I, w, h, i+1, j  )
	          + getpixel_1(I, w, h, i  , j+1)
	          + getpixel_1(I, w, h, i-1, j  )
	          + getpixel_1(I, w, h, i  , j-1);
}

static void downsample_by_factor_two(float *out, int ow, int oh,
		float *in, int iw, int ih)
{
	if (!out || !in) return;
	assert(abs(2*ow-iw) < 2);
	assert(abs(2*oh-ih) < 2);
	for (int j = 0; j < oh; j++)
	for (int i = 0; i < ow; i++)
		out[ow*j+i] = getpixel_1(in, iw, ih, 2*i, 2*j);
}

#define MAX_LEVELS 20
struct gray_image_pyramid {
	int n;                // number of levels
	int w[MAX_LEVELS];    // width of each level
	int h[MAX_LEVELS];    // height of each level
	float *x[MAX_LEVELS]; // data of each level
};

static
float pyramidal_laplacian(struct gray_image_pyramid *p, float x, float y, int o)
{
	if (o < 0 || o >= p->n)
		return -INFINITY;
	float *I = p->x[o];
	int w = p->w[o];
	int h = p->h[o];
	float a00 = getlaplacian_1(I, w, h, round(x  ), round(y  ));
	float a10 = getlaplacian_1(I, w, h, round(x+1), round(y  ));
	float a01 = getlaplacian_1(I, w, h, round(x  ), round(y+1));
	float am0 = getlaplacian_1(I, w, h, round(x-1), round(y  ));
	float a0m = getlaplacian_1(I, w, h, round(x  ), round(y-1));
	return fmax(a00, fmax(fmax(a10,a01),fmax(am0,a0m)));
}

static void fill_pyramid(struct gray_image_pyramid *p, float *x, int w, int h)
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
		float *tmp1 = xmalloc_float(p->w[i-1] * p->h[i-1]);
		float *tmp2 = xmalloc_float(p->w[i-1] * p->h[i-1]);
		//float *tmp3 = xmalloc_float(p->w[i-1] * p->h[i-1]);
		poor_man_gaussian_filter(tmp1,p->x[i-1],p->w[i-1],p->h[i-1], S);
		poor_man_gaussian_filter(tmp2,tmp1,p->w[i-1],p->h[i-1], S);
		//poor_man_gaussian_filter(tmp3,tmp2,p->w[i-1],p->h[i-1], S);
		downsample_by_factor_two(p->x[i], p->w[i], p->h[i],
			       	tmp2, p->w[i-1], p->h[i-1]);
		free(tmp1);
		free(tmp2);
		//free(tmp3);
	}
	p->n = i;
}

static void free_pyramid(struct gray_image_pyramid *p)
{
	for (int i = 0; i < p->n; i++)
		free(p->x[i]);
}

static float parabolic_minimum(float p, float q, float r)
{
	//return 0;
	if (isfinite(p) && isfinite(q) && isfinite(r))
	{
		//if (p < q && p < r) return -1;
		//if (q < p && q < r) return 0;
		//if (r < p && r < q) return 1;
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
	return NAN;
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
			break;
	}
	assert(n < max_npoints);
	return n;
}

int harressian(float *out_xys, int max_npoints, float *x, int w, int h,
		float sigma, float kappa, float tau)
{
	// filter input image
	float *sx = xmalloc_float(w * h);
	poor_man_gaussian_filter(sx, x, w, h, sigma);

	// create image pyramid
	struct gray_image_pyramid p[1];
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
			float new_factor = factor * (3*factor_scaling + 5) / 4;

			out_xys[3*n+0] = factor * x;
			out_xys[3*n+1] = factor * y;
			out_xys[3*n+2] = new_factor;
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
