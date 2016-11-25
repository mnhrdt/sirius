// BINARY DATA FORMATS
// -------------------
//
// RAW: bytes, or "packed bits"
// BIT: one bit on every byte, or "unpacked bits"
// RLE1: bits encoded into bytes by lenght of constant runs
// RLE8: run-length encoded bytes, à la PCX
// HUF8: canonically Huffman-encoded bytes
// HUF1: canonically Huffman-encoded runs of bits (i.e., RLE1 followed by HUF8)
// B64: base 64 encoding of bytes into printable characters
// A85: base 85 encoding of bytes into printable characters
// X85: base 85 encoding of bytes into printable characters (alternate charset)
// FAX: RLE1 + HUF8 + B64
// LZW: Lempel-Ziv (not implemented)

// implementation idea: do not think in terms of data conversions between pairs
// of formats, but in terms of data transformations, from whatever format into
// the target one.  That way, composition of formats is automatically supported.


///////////////
////  API  ////
///////////////
#include <stdint.h>


// 1. elementary transformations

// 1.1. unpacking and packing
uint8_t *alloc_and_transform_from_RAW_to_BIT(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_BIT_to_RAW(uint8_t *x, int n, int *nout);

// 1.2. run-length encoding and decoding
uint8_t *alloc_and_transform_from_BIT_to_RLE1(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_RLE1_to_BIT(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_RAW_to_RLE8(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_RLE8_to_RAW(uint8_t *x, int n, int *nout);

// 1.3. differential coding and decoding
uint8_t *alloc_and_transform_diff(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_undiff(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_xor(uint8_t *x, int n, int *nout);

// 1.3. canonical huffman encoding and decoding
uint8_t *alloc_and_transform_from_RAW_to_HUF8(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_HUF8_to_RAW(uint8_t *x, int n, int *nout);

// 1.4. base64 encoding
uint8_t *alloc_and_transform_from_RAW_to_B64(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_B64_to_RAW(uint8_t *x, int n, int *nout);

// 1.5. base85 encoding
uint8_t *alloc_and_transform_from_RAW_to_A85(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_A85_to_RAW(uint8_t *x, int n, int *nout);

// 1.6. lempel-ziv compression (will not implemented)
//uint8_t *alloc_and_transform_from_RAW_to_LZW(uint8_t *x, int n, int *nout);



// 2. composite transformations (as a form of API convenience)

// 2.1. BIT => RLE1 => HUF8
uint8_t *alloc_and_transform_from_BIT_to_HUF1(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_HUF1_to_BIT(uint8_t *x, int n, int *nout);

// 2.2. RAW => BIT => RLE1 => HUF8
uint8_t *alloc_and_transform_from_RAW_to_HUF1(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_HUF1_to_RAW(uint8_t *x, int n, int *nout);

// 2.3. BIT => RLE1 => HUF8 => B64
uint8_t *alloc_and_transform_from_BIT_to_B64(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_B64_to_BIT(uint8_t *x, int n, int *nout);

// 2.4. RAW => BIT => RLE1 => HUF8 => B64
uint8_t *alloc_and_transform_from_RAW_to_FAX(uint8_t *x, int n, int *nout);
uint8_t *alloc_and_transform_from_FAX_to_RAW(uint8_t *x, int n, int *nout);



//////////////////////////
////  IMPLEMENTATION  ////
//////////////////////////

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>

#include "xmalloc.c"
#include "fail.c"

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
