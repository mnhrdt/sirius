
static bool compute_mauricio(float *x, int w, int h)
{
	return false; // mauricio test is provisionnaly disabled by now

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
		// extract 3x3 neighborhood
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

	return
		(num_cx_large > global_mauricio_Cth) &&
		(num_cy_large > global_mauricio_Cth) &&
		(sat_percent < global_mauricio_Sth);
}

