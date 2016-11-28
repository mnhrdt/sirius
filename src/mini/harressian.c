// implementation of the "harris hessian" keypoint detector

#include "xmalloc.c"

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
		out[i+j*w] = ax/kn;
	}
}

// the type of a "getpixel" function
typedef float (*getpixel_operator)(float*,int,int,int,int);

// extrapolate by nearest value (useful for Neumann boundary conditions)
static float getpixel_1(float *I, int w, int h, int i, int j)
{
	if (i < 0) i = 0;
	if (j < 0) j = 0;
	if (i >= w) i = w-1;
	if (j >= h) j = h-1;
	return I[i+j*w];
}

static void zoom_out_by_factor_two(float *out, int ow, int oh,
		float *in, int iw, int ih)
{
	if (!out || !in) return;
	getpixel_operator p = getpixel_1;
	assert(abs(2*ow-iw) < 2);
	assert(abs(2*oh-ih) < 2);
	for (int j = 0; j < oh; j++)
	for (int i = 0; i < ow; i++)
	{
		float a[4];
		a[0] = p(in, iw, ih, 2*i, 2*j);
		a[1] = p(in, iw, ih, 2*i+1, 2*j);
		a[2] = p(in, iw, ih, 2*i, 2*j+1);
		a[3] = p(in, iw, ih, 2*i+1, 2*j+1);
		out[ow*j+i] = (a[0] + a[1] + a[2] + a[3]) / 4;
	}
}

#define MAX_LEVELS 20
struct gray_image_pyramid {
	int n;                // number of levels
	int w[MAX_LEVELS];    // width of each level
	int h[MAX_LEVELS];    // height of each level
	float *x[MAX_LEVELS]; // data of each level
};

static void fill_pyramid(struct gray_image_pyramid *p, float *x, int w, int h)
{
	int i = 0;
	p->w[i] = w;
	p->h[i] = h;
	p->x[i] = xmalloc(w * h * sizeof*x);
	poor_man_gaussian_filter(p->x[i], x, w, h, 1);
	while (1) {
		i += 1;
		if (i + 1 >= MAX_LEVELS) break;
		p->w[i] = ceil(p->w[i-1]/2);
		p->h[i] = ceil(p->h[i-1]/2);
		if (p->w[i] <= 1 && p->h[i] <= 1) break;
		p->x[i] = xmalloc(p->w[i] * p->h[i] * sizeof*x);
		float *tmp = xmalloc(p->w[i-1] * p->h[i-1] * sizeof*x);
		poor_man_gaussian_filter(tmp, p->x[i-1],p->w[i-1],p->h[i-1], 2.81);
		zoom_out_by_factor_two(p->x[i], p->w[i], p->h[i],
			       	tmp, p->w[i-1], p->h[i-1]);
		free(tmp);
	}
	p->n = i;
}

static void free_pyramid(struct gray_image_pyramid *p)
{
	for (int i = 0; i < p->n; i++)
		free(p->x[i]);
}

//int alloc_poor_man_multiscale(float **px, int (*wh)[2], float *x, int w, int h)
//{
//	if (w == 1 && h == 1) {
//		*px = NULL;
//		return 0;
//	}
//	int ws = ceil(w/2.0);
//	int hs = ceil(h/2.0);
//	float *xs = xmallow(ws * hs * 
//	float *my_x = *px;
//	int *my_wh = *wh;
//
//}
//
//void free_poor_man_multiscale(float **px, int n)
//{
//	for (int i = 0; i < n; i++)
//		free(px[i]);
//}

int harressian_nogauss(float *out_xy, int max_npoints,
		float *x, int w, int h, float kappa, float tau)
{
	float sign = kappa > 0 ? 1 : -1;
	kappa = fabs(kappa);
	int n = 0;
	for (int j = 1; j < h - 1; j++)
	for (int i = 1; i < w - 1; i++)
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
		//float dxy =  (2*V00 +Vpm +Vmp -Vm0 -V0m -Vp0 -V0p)/2;
		//float dyx = -(2*V00 +Vmm +Vpp -Vm0 -V0p -Vp0 -V0m)/2;
		//
		// XXX TODO : these schemes for dxy have directional aliasing!
		//
		float T = dxx + dyy;
		float D = dxx * dyy - dxy * dyx;
		float R0 = D - kappa * T * T;
		if (T > tau && R0 > 0)
		{
			// TODO: higher-order sub-pixel localisation
			float alpha_i = (Vp0 + Vm0)/2 - V00;
			float alpha_j = (V0p + V0m)/2 - V00;
			float beta_i  = (Vp0 - Vm0)/2;
			float beta_j  = (V0p - V0m)/2;
			out_xy[2*n + 1] = j - 0.5 * beta_j / alpha_j;
			n += 1;
		}
		if (n >= max_npoints)
			goto done;
	}
done:	assert(n <= max_npoints);
	return n;
}

int harressian_ms(float *out_xys, int max_npoints, float *x, int w, int h,
		float sigma, float kappa, float tau)
{
	// filter input image
	float *sx = xmalloc(w * h * sizeof*sx);
	poor_man_gaussian_filter(sx, x, w, h, sigma);

	// create image pyramid
	struct gray_image_pyramid p[1];
	fill_pyramid(p, sx, w, h);
	float *tab_xy = xmalloc(2 * max_npoints * sizeof*tab_xy);

	// apply nongaussian harressian at each level of the pyramid
	int n = 0;
	for (int l = p->n - 1; l >= 0; l--)
	{
		int n_l = harressian_nogauss(tab_xy, max_npoints - n,
				p->x[l], p->w[l], p->h[l], kappa, tau);
		float factor = 1 << l;
		for (int i = 0; i < n_l; i++)
		{
			out_xys[3*n+0] = factor * (tab_xy[2*i+0] + 0.48);
			out_xys[3*n+1] = factor * (tab_xy[2*i+1]);
			out_xys[3*n+2] = factor;
			n += 1;
		}
	}
	assert(n <= max_npoints);

	// cleanup and exit
	free(tab_xy);
	free_pyramid(p);
	free(sx);
	return n;
}

int harressian(float *out_xy, int max_npoints,
		float *x, int w, int h, float sigma, float kappa, float tau,
		float *outopt)
{
	float *sx = xmalloc(w * h * sizeof*sx);
	poor_man_gaussian_filter(sx, x, w, h, sigma);
	//poor_man_gaussian_filter(x, sx, w, h, sigma);
	//poor_man_gaussian_filter(sx, x, w, h, sigma);
	struct gray_image_pyramid pyr[1];
	fill_pyramid(pyr, sx, w, h);

	int n = harressian_nogauss(out_xy, max_npoints,
			pyr->x[1], pyr->w[1], pyr->h[1],
			kappa, tau);
	for (int i = 0; i < 2*n; i++)
		out_xy[i] *= 2;


	if (false && outopt) // this is just for debugging purposes
	            // all the code below must be removed
	{
		for (int i = 0; i < w*h; i++)
			outopt[i] = 0;//rand()%255;
		for (int j = 1; j < h - 1; j++)
		for (int i = 1; i < w - 1; i++)
		{
			float Vmm = sx[(i-1) + (j-1)*w];
			float V0m = sx[(i+0) + (j-1)*w];
			float Vpm = sx[(i+1) + (j-1)*w];
			float Vm0 = sx[(i-1) + (j+0)*w];
			float V00 = sx[(i+0) + (j+0)*w];
			float Vp0 = sx[(i+1) + (j+0)*w];
			float Vmp = sx[(i-1) + (j+1)*w];
			float V0p = sx[(i+0) + (j+1)*w];
			float Vpp = sx[(i+1) + (j+1)*w];
			float dxx = Vm0 - 2*V00 + Vp0;
			float dyy = V0m - 2*V00 + V0p;
			//float dxy = (Vpp + Vmm - Vpm - Vmp)/4;
			//float dyx = dxy;
			float dxy = (2*V00 +Vpm +Vmp -Vm0 -V0m -Vp0 -V0p)/2;
			float dyx = -(2*V00 +Vmm +Vpp -Vm0 -V0p -Vp0 -V0m)/2;
			float T = dxx + dyy;
			float D = dxx * dyy - dxy * dyx;
			float R0 = D - kappa * T * T;
			outopt[j*w+i] = 255 - fmin(254,8*fmax(0,R0));
		}
	}
	free(sx);
	free_pyramid(pyr);
	return n;
}
