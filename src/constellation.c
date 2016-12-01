// simplest constellation matching

struct constellation {
	int n;     // number of stars
	float *xy; // coordinates of the stars
};

int constellation_matching(double out_H[9],
		float *sky, int nsky, float *constellation, int nconst)
{
	struct constellation c[1] = {{nconst, constellation}};
	int *mask = xmalloc(nsky * sizeof*mask);
	int r = ransac(mask, out_H, sky, 2,  );
	free(mask);
	return r;
}
