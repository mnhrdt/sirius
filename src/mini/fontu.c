#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xmalloc.c"
#include "fail.c"

// TODO: fix data leaks when callinc "alloc_and_transform" functions

enum font_data_format {
	UNPACKED, // array of chars with boolean values
	PACKED,   // array of chars with arbitrary values (with the same bitmap)
	RLE,      // binary run-length encoding
	PCX,      // binary run-length encoding
	RLET,      // binary run-length encoding, transposed
	//HUFFMAN,  // huffman code of "PACKED"
	//RLEH,     // huffman code of "RLE"
	X85,      // x85 encoding of "PACKED"
	X85RLE,   // x85 encoding of "RLE"
	X85RLET,   // x85 encoding of "RLE", transposed
	//X85HUF,   // x85 encoding of "HUFFMAN"
	//X85RLEH,  // x85 encoding of "RLEHUFFMAN" (efficient for C dumps)
	DIFF,
	XOR,
	RLEDIFF,
	RLEXOR,
	RLEPCX,
	RLEXORPCX,
	PCXX85,
	RLEPCXX85,
	RLEXORPCXX85,
};

// a font is a three-dimensional binary image
// its bits can be stored in different formats
struct bitmap_font {
	int number_of_glyphs;
	int width;
	int height;

	enum font_data_format packing;
	int ndata; // when UNPACKED: ndata = number_of_glyphs * width * height
	char *name; // optional field
	unsigned char *data;
};


#include "dataconv.c"

static char *packing_string(enum font_data_format p)
{
	switch(p) {
#define casepack(s) case s: return #s
	casepack(PACKED);
	casepack(UNPACKED);
	casepack(RLE);
	casepack(RLET);
	casepack(X85);
	casepack(X85RLE);
	casepack(X85RLET);
	casepack(DIFF);
	casepack(XOR);
	casepack(RLEDIFF);
	casepack(RLEXOR);
	casepack(PCX);
	casepack(RLEPCX);
	casepack(RLEXORPCX);
	casepack(PCXX85);
	casepack(RLEPCXX85);
	casepack(RLEXORPCXX85);
#undef casepack
	default: fail("impossible packing style");
	}
}

static struct bitmap_font reformat_font(struct bitmap_font f,
		enum font_data_format fmt)
{
	fprintf(stderr, "reformat_font(%s -> %s)\n",
			packing_string(f.packing), packing_string(fmt));
	uint8_t *(*transform)(uint8_t *t, int, int*) = NULL;

	if (fmt == f.packing) {
		return f;
	} else if (fmt == UNPACKED && f.packing == PACKED) {
		transform = alloc_and_transform_from_RAW_to_BIT;
	} else if (fmt == PACKED   && f.packing == UNPACKED) {
		transform = alloc_and_transform_from_BIT_to_RAW;
	} else if (fmt == RLE) {
		f = reformat_font(f, UNPACKED);
		transform = alloc_and_transform_from_BIT_to_RLE1;
	} else if (f.packing == RLE && fmt == UNPACKED) {
		transform = alloc_and_transform_from_RLE1_to_BIT;
	} else if (f.packing == PCXX85 && fmt == UNPACKED) {
		// PCXX85 -x85toraw-> PCX -pcxtoraw-> PACKED -rawtobit-> UNPACKED
	// TODO arrays of transforms (turn this function from code to data)
	f.data = alloc_and_transform_from_X85_to_RAW(f.data, f.ndata, &f.ndata);
	f.data = alloc_and_transform_from_RLE8_to_RAW(f.data, f.ndata, &f.ndata);
	f.data = alloc_and_transform_from_RAW_to_BIT(f.data, f.ndata, &f.ndata);
	f.packing = UNPACKED;
	return f;
	} else if (fmt == DIFF) {
		f = reformat_font(f, PACKED);
		transform = alloc_and_transform_diff;
	} else if (fmt == XOR) {
		f = reformat_font(f, PACKED);
		transform = alloc_and_transform_xor;
	} else if (fmt == RLEDIFF) {
		f = reformat_font(f, RLE);
		transform = alloc_and_transform_diff;
	} else if (fmt == RLEXOR) {
		f = reformat_font(f, RLE);
		transform = alloc_and_transform_xor;
	} else if (fmt == PCX) {
		f = reformat_font(f, PACKED);
		transform = alloc_and_transform_from_RAW_to_RLE8;
	} else if (fmt == RLEPCX) {
		f = reformat_font(f, RLE);
		transform = alloc_and_transform_from_RAW_to_RLE8;
	} else if (fmt == RLEXORPCX) {
		f = reformat_font(f, RLEXOR);
		transform = alloc_and_transform_from_RAW_to_RLE8;
	} else if (fmt == X85) {
		f = reformat_font(f, PACKED);
		transform = alloc_and_transform_from_RAW_to_X85;
	} else if (fmt == X85RLE) {
		f = reformat_font(f, RLE);
		transform = alloc_and_transform_from_RAW_to_X85;
	} else if (fmt == PCXX85) {
		f = reformat_font(f, PCX);
		transform = alloc_and_transform_from_RAW_to_X85;
	} else if (fmt == RLEPCXX85) {
		f = reformat_font(f, RLEPCX);
		transform = alloc_and_transform_from_RAW_to_X85;
	} else if (fmt == RLEXORPCXX85) {
		f = reformat_font(f, RLEXORPCX);
		transform = alloc_and_transform_from_RAW_to_X85;
	} else if (fmt == RLET) {
		f = reformat_font(f, UNPACKED);
		f.data = alloc_and_transpose_3d1(f.data, f.width, f.height,
				f.number_of_glyphs);
		transform = alloc_and_transform_from_BIT_to_RLE1;
	} else
		fail("unimplemented conversion \"%s\"=>\"%s\"\n",
				packing_string(f.packing), packing_string(fmt));

	if (transform)
		f.data = transform(f.data, f.ndata, &f.ndata);
	f.packing = fmt;
	return f;

	fail("unimplemented conversion \"%s\"\n", packing_string(fmt));
}

static int get_font_bit(struct bitmap_font *f, int c, int i, int j)
{
	assert(f->packing == UNPACKED);
	if (c < 0 || c > f->number_of_glyphs)
		return 0;
	return f->data[(c*f->height + j)*f->width + i];
}

static void put_pixel(float *x, int w, int h, int pd, int i, int j, float *c)
{
	//if (j*w + i < w*h)
	if (i>=0 && j>=0 && i<w && j<h)
		for (int l = 0; l < pd; l++)
			x[(w*j+i)*pd+l] = c[l];
}

static void put_pixel_rgb(uint8_t *x, int w, int h, int i, int j, uint8_t *c)
{
	if (c && i>=0 && j>=0 && i<w && j<h)
		for (int l = 0; l < 3; l++)
			x[(w*j+i)*3+l] = c[l];
}

static void put_string_in_float_image(float *x, int w, int h, int pd,
		int posx, int posy, float *color, int kerning,
		struct bitmap_font *font, char *string)
{
	int posx0 = posx;
	while (1)
	{
		int c = *string++;
		if (!c) break;
		if (c == '\n') {
			posx = posx0;
			posy += font->height;
			continue;
		}
		if (c > 0 && c < font->number_of_glyphs)
		{
			for (int i = 0; i < font->width; i++)
			for (int j = 0; j < font->height; j++)
				if (get_font_bit(font, c, i, j))
				{
					int ii = posx + i;
					int jj = posy + j;
					put_pixel(x, w, h, pd, ii, jj, color);
				}
		}
		posx += font->width + kerning;
	}
}

static void put_string_in_rgb_image(uint8_t *x, int w, int h,
		int posx, int posy, uint8_t *fg, uint8_t *bg, int kerning,
		struct bitmap_font *font, char *string)
{
	int posx0 = posx;
	while (1)
	{
		int c = *string++;
		if (!c) break;
		if (c == '\n') {
			posx = posx0;
			posy += font->height;
			continue;
		}
		if (c > 0 && c < font->number_of_glyphs)
		{
			for (int i = 0; i < font->width; i++)
			for (int j = 0; j < font->height; j++)
			{
				int ii = posx + i;
				int jj = posy + j;
				if (get_font_bit(font, c, i, j))
					put_pixel_rgb(x, w, h, ii, jj, fg);
				else
					put_pixel_rgb(x, w, h, ii, jj, bg);
			}
		}
		posx += font->width + kerning;
	}
}
