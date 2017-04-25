#include "harressian.c"
#include "iio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "pickopt.c"
#include "xfopen.c"
int main(int c, char *v[])
{
	// extract named options
	int maxpoints = atoi(pick_option(&c, &v, "m", "1000"));
	float param_s = atof(pick_option(&c, &v, "s", "1.0"));
	float param_k = atof(pick_option(&c, &v, "k", "0.24"));
	float param_t = atof(pick_option(&c, &v, "t", "30"));

	// process remaining positional arguments
	if (c > 3 || (c == 2 && !strcmp(v[1], "-h")))
		return fprintf(stderr, "usage:\n\t%s [in.png [out.txt]]\n", *v);
	char *filename_in  = c > 1 ? v[1] : "-";
	char *filename_out = c > 2 ? v[2] : "-";

	// read input image
	int w, h, pd;
	float *x = iio_read_image_float_vec(filename_in, &w, &h, &pd);

	// allocate space for output table
	float *y = malloc(maxpoints * 4 * sizeof*y);

	// run the algorithm
	int n = harressian(y, maxpoints, x, w, h, param_s, param_k, param_t);

	// write result
	FILE *f = xfopen(filename_out, "w");
	for (int i = 0; i < n; i++)
	{
		float *z = y + 4*i;
		fprintf(f, "%g %g %g %g\n", z[0], z[1], z[2], z[3]);
	}
	xfclose(f);

	// cleanup and exit
	free(x);
	free(y);
	return 0;
}
