#ifndef _XMALLOC_C
#define _XMALLOC_C

#include <stdlib.h>
#include <stdint.h>

#include "fail.c"

static void *xmalloc(size_t size)
{
#ifndef NDEBUG
	{
		double sm = size / (0x100000 * 1.0);
		if (sm > 1000)
			fprintf(stderr, "WARNING: large malloc"
					" %zu bytes (%gMB)\n", size, sm);
	}
#endif
	if (size == 0)
		fail("xmalloc: zero size");
	void *now = malloc(size);
	if (!now)
	{
		double sm = size / (0x100000 * 1.0);
		fail("xmalloc: out of memory when requesting "
			"%zu bytes (%gMB)",//:\"%s\"",
			size, sm);//, strerror(errno));
	}
	return now;
}

// note: due to C++ brain damage, the following silly functions are necessary
float *xmalloc_float(int n){return(float*)xmalloc(n*sizeof(float));}
int *xmalloc_int(int n){return(int*)xmalloc(n*sizeof(int));}
bool *xmalloc_bool(bool n){return(bool*)xmalloc(n*sizeof(bool));}
uint8_t *xmalloc_uint8(int n){return(uint8_t*)xmalloc(n*sizeof(uint8_t));}
uint32_t *xmalloc_uint32(int n){return(uint32_t*)xmalloc(n*sizeof(uint32_t));}
#endif//_XMALLOC_C
