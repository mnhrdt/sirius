#include "xmalloc.c"

void poor_man_gaussian_filter(float *out, float *in, int w, int h, float sigma)
{
	for (int j = 1; j < h - 1; j ++)
	for (int i = 1; i < w - 1; i ++)
	{
		float ax = 0;
		for (int dj = -1; dj <= 1; dj++)
		for (int di = -1; di <= 1; di++)
		{
			int ii = i + di;
			int jj = j + dj;
			ax += in[ii+jj*w];
		}
		out[i+j*w] = ax / 9;
	}
}

int harris(float *out_xy, int max_npoints,
		float *x, int w, int h, float sigma, float kappa, float tau)
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
		float dxy = Vpp + Vmm - Vpm - Vmp;
		float T = dxx + dyy;
		float D = dxx * dyy - dxy * dxy;
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
	free(sx);
	return n;
}
