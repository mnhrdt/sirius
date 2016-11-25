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

int harris(float *out_xy, int max_npoints,
		float *x, int w, int h, float sigma, float kappa, float tau,
		float *outopt)
{
	float *sx = xmalloc(w * h * sizeof*sx);
	poor_man_gaussian_filter(sx, x, w, h, sigma);
	//poor_man_gaussian_filter(x, sx, w, h, sigma);
	//poor_man_gaussian_filter(sx, x, w, h, sigma);

	int n = 0;
	for (int j = 1; j < h - 1; j++)
	for (int i = 1; i < w - 1; i++)
	{
		// Vmm V0m Vpm
		// Vm0 V00 Vp0
		// Vmp V0p Vpp
		float Vmm = sx[(i-1) + (j-1)*w];
		float V0m = sx[(i+0) + (j-1)*w];
		float Vpm = sx[(i+1) + (j-1)*w];
		float Vm0 = sx[(i-1) + (j+0)*w];
		float V00 = sx[(i+0) + (j+0)*w];
		float Vp0 = sx[(i+1) + (j+0)*w];
		float Vmp = sx[(i-1) + (j+1)*w];
		float V0p = sx[(i+0) + (j+1)*w];
		float Vpp = sx[(i+1) + (j+1)*w];
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
		// NOTE XXX TODO : this scheme for dxy has directional aliasing!
		//
		float T = dxx + dyy;
		float D = dxx * dyy - dxy * dyx;
		float R0 = D - kappa * T * T;
		if (T > tau && R0 > 0)
		{
			out_xy[2*n + 0] = i;
			out_xy[2*n + 1] = j;
			n += 1;
		}
		if (n >= max_npoints)
			goto done;
	}
done:	assert(n <= max_npoints);
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
			//if (
			//   V0m<V00 || Vm0<V00 || V0p<V00 || Vp0<V00
			//	//|| Vmm<V00 || Vpp<V00 || Vmp<V00 || Vpm<V00
			//	)
			//	outopt[j*w+i] = 0;
			//else
			//	outopt[j*w+i] = 255;
		}
		//{
		//	outopt[j*w+i] = x[j*w+i];
		//	//float Vmm = x[(i-1) + (j-1)*w];
		//	//float V0m = x[(i+0) + (j-1)*w];
		//	//float Vpm = x[(i+1) + (j-1)*w];
		//	//float Vm0 = x[(i-1) + (j+0)*w];
		//	//float V00 = x[(i+0) + (j+0)*w];
		//	//float Vp0 = x[(i+1) + (j+0)*w];
		//	//float Vmp = x[(i-1) + (j+1)*w];
		//	//float V0p = x[(i+0) + (j+1)*w];
		//	//float Vpp = x[(i+1) + (j+1)*w];
		//	//if (//i % 10 == 0
		//	//   V0m<V00 || Vm0<V00 || V0p<V00 || Vp0<V00
		//	//	|| Vmm<V00 || Vpp<V00 || Vmp<V00 || Vpm<V00
		//	//	)
		//	//	outopt[j*w+i] = x[j*w+i];
		//	//else
		//	//	outopt[j*w+i] = x[j*w+i];
		//}
	}
	free(sx);
	return n;
}
