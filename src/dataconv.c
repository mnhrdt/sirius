// simple data conversion routines
#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "xmalloc.c"

#define SETBIT(x,i) ((x)|=(1<<(i)))
#define GETBIT(x,i) (bool)((x)&(1<<(i)))


// unpack bytes into individual bits, thus enlarging the array eightfold
uint8_t *alloc_and_transform_from_RAW_to_BIT(uint8_t *x, int n, int *nout)
{
	*nout = 8*n;
	uint8_t *y = xmalloc(*nout+16);
	for (int i = 0; i < n; i++)
		for (int j = 0; j < 8; j++)
			y[8*i+j] = GETBIT(x[i], j);
	return y;
}

// base 85 decoding with quotable charset
uint8_t *alloc_and_transform_from_X85_to_RAW(uint8_t *x, int n, int *nout)
{
	assert(0 == n % 5);
	*nout = 4*n/5;
	uint32_t *y = xmalloc(*nout);
	for (int i = 0; i < n/5; i++)
	{
		y[i] = 0;
		for (int j = 0; j < 5; j++)
		{
			uint8_t q = x[5*i+j];
			assert(isprint(q));
			y[i] = y[i]*85 + q - (q < 92 ? 40 : 41);
		}
	}
	return (void*)y;
}

// PCX decoding
uint8_t *alloc_and_transform_from_RLE8_to_RAW(uint8_t *x, int n, int *nout)
{
	uint8_t *y = xmalloc(64*n);
	int r = 0, i = 0;
	while (i < n)
	{
		if (x[i] > 191) {
			assert(i+1 < n);
			int count = x[i] - 192 + 1;
			for (int j = 0; j < count; j++)
				y[r++] = x[i+1];
			i += 2;
		} else
			y[r++] = x[i++];
	}
	*nout = r;
	return y;
}
