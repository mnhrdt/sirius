#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "iio.h"      // functions for reading and writing images
#include "xmalloc.c"  // retargetable malloc
#include "drawing.c"  // functions for drawing figures on fRGB images
#include "pickopt.c"  // function for extracting command-line named options

int main(int c, char *v[])
{
	// extract named options
	char *filename_bg = pick_option(&c, &v, "bg", "");

	// process remaining positional arguments
	if (c != 3 && !(c == 1 && *filename_bg))
		return fprintf(stderr, "usage:\n\t%s w h <3cols >out.png\n",*v);
	int w = c > 1 ? atoi(v[1]) : 0;
	int h = c > 2 ? atoi(v[2]) : 0;

	// create background image
	float *x = NULL;
	if (*filename_bg)
		x = iio_read_image_float_rgb(filename_bg, &w, &h);
	else {
		x = xmalloc(3 * w * h * sizeof*x);
		for (int i = 0; i < 3 * w * h; i++)
			x[i] = 0;
	}

	// process input lines
	float t[3];
	while (3 == scanf("%g %g %g %*[^\n]\n", t, t+1, t+2))
	{
		overlay_circle_rgb(x, w, h, t[0], t[1], t[2]+1, 255, 0, 0);
		overlay_circle_rgb(x, w, h, t[0], t[1], t[2]+0, 0, 255, 0);
	}

	// write result
	uint8_t *y = xmalloc(3 * w * h);
	for (int i = 0; i < 3 * w * h; i++)
		y[i] = x[i];
	iio_write_image_uint8_vec("-", y, w, h, 3);

	// cleanup and exit
	free(x);
	free(y);
	return 0;
}
