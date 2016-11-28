#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xmalloc.c"
#include "fail.c"

// TODO: fix data leaks when calling "alloc_and_transform" functions

enum font_data_format {
	UNPACKED, // array of chars with boolean values
	PACKED,   // array of chars with arbitrary values (with the same bitmap)
	RLE,      // binary run-length encoding
	PCX,      // binary run-length encoding
	RLET,      // binary run-length encoding, transposed
	X85,      // x85 encoding of "PACKED"
	X85RLE,   // x85 encoding of "RLE"
	X85RLET,   // x85 encoding of "RLE", transposed
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

unsigned char xfont_8x13_data[] =
"(1.)((.x[U{-I,jzzL.(zzL.(qzR0b((xYLgEyPbg*^^.PV.Gn(<x6(+Nmp81cvqL=qj28-He^i3*"
"nnvg+oDg3*ZofgEyQa2@@Sp*n>5pge[U3*n4_K*n5/p3XWfj3[/|P*n4TgjX}+R(`x,Rk@B=Q*[V1"
"T<n(OB)A3WegbGY[h+F7)gdi`k+;P8FghmGL=Naji)Bq>O(<r=m-7V<m=@zGO2@@BHg.JLg2TqSUg"
"7}J,g2+Omgh]>Ug*}f|g2+Om(<owkg-}(tg+Pro;@:?,gLA7,(<owkGYGuv2G1jQ*X^oh(<Zxdh(G"
"Do*[V1T+tn?h<XQa-)A3We<[o]Kg7}J/(=.Im-4zJF)A3We;@:?*2G-@7-41@`g0yg?J5hT<ghY}}"
"gIez}=Nm8C2{+i,2{,xxgeJ(oh.XXlgh]=mh+4=-geJ(o(tIIlPRPH4gSBS-(tO-d=@N>k=F:5L(<"
"q-Ugo`yq;wqQmghmGNh^G;Rge7rJ-oI`I-lmoT(<p3Pgo^c4GuYx9gMgYKg*^HDBN.KMgiB;S=O92"
"n;wqQgghoL/;@:?*BN.@w(<f}h<(u2T==c+S=@N>kg0ym?gh]>U*npKx=O@0Kg*^H?gIXqg(<n.^?"
"e(l9(<o(d.P*qbg.7Y(g*^HDh)`JePR>>s*Z?2KPRPDoheV>D(<ow,)@Q^Fg-{@x1_`0JgeII62y|"
"Agg80P<k;J=eOxIlwgl+aB:_l5}>YMxd;@:Hf=:D|qgEyQ@g5+a>hh}|EPY=4{gh]=mg1o=g)U1K2"
"g4F}c;@FRO(<]ki>XS]n(<y;HgcOTU(<y-3ggB.<1+_:H(tDzY-m<U}(<p3P;xe5Agh,Z?(=5Bz=N"
"t?YghYyD(=53n>YIY1g}lmA=DV8)<n4k?=wsDl(=36b7M]@8(<x}lgLQ2Dhe4FpE],0l36VF)h`(@"
"xg1WuBhbmKZg.KEPhf8xnGYGx:;G9BC2M`D6PR[wng6RJt-5I1]gn3u|gc2Ch-Hdp^g+Kg*(<n@`z"
"|=m3zzL.(zzL.(zzL.(*X^iIgc2=f5UHew1,KC:9}xt-gGFeAhI4Utg0|+ih.XY:3(r+M;x`Dl(<n"
"0agb7|c/egKag-1B*3[uxSg*^Dp1()33>@0w.1+Y{Z2C8J)(.KNn?4+`v-le7z(=Gd^iFc]s(=53+"
"g2Eor3*yFBPR>B2g-17y(=O..g165Gghm-2-6*IVhbE7q*ZPVNh`LPsk:V{eHr`I?(<dmG69)mm(="
"=T1-48-w)xDwJh{z=p.*D|^h}a:p3ZJJh*vmSZ({6QFI7tc@F{*zz({6QEANU(nNY3}o*uSJOHsC>"
"9F{*zz((qZI(`-mMgLA7+()eK@=N`wigL}/n((r3@=N`wigL}/n(+qIe=N`wigL}/n(-G;Q=N`wig"
"L}/n(+wHd=N`wigL}/ng.Iq|=N`wigiB8o+3]qk+3P4;;@:@L=:E/t((r3}PR7wjg-hbw(<i7lPR7"
"_jg-hbw(<i7lPR8_8g-hbw(<i7lPR8a7g-hbw(<i7l;wjqsgLQ>K((r3@;xe/r3[+U}*nQv3g*^E@"
"h.26M(<o}2=N`qm3({2@4:8v/=]AdqG?[[j)@FB0h.263(<fwk;wjr)gLS@/(+qIe<(l2U0/*G*=3"
"aymg*^E@h.26M(=,3n/z>Pi(<p3Rg5*Tt>ZAUy(<Zg(hBo6O(<fqjhBo6[(<fqjhBp6*(<fqjhBp8"
")(<fqjg*VhW*nGF*;w|h,gLSC.1((s`+<2X(:f]aC-4z@YOxIlwgl+aB()f>X=GB+d(<q-U;@4_KG"
"YEO4@LB|?<_n+,Eco+P3n.Q}OxIlwgOfWA/iRx}OxIlwhh}|E<)PY:hdxIxg1]=i-:U?d*X^cO=@N"
"<b;G71}*Z>(YPY=4{gh]=m(+qIe(m1O}(<owk;@4aJ=:B]-)@FE,gcOTU(<o}2+qoK@ghmGK()wdF"
";xe,>.*DYk*nG-U3XWdM;CWD[gh]@.(-G;Q=Nt?Y*X|5M=Nsvk-47]8ghYw0(<owk;@4_Kgh]@.(-"
"G;Q;G=yQ3n.Q}=Nsvkg*^K@(.]Ho(=3BdBP^iY(f-yL*X|5LE],0l*Z>(YE],0l3[+V(E],0l.a{l"
"mE],0l*Z>(YGYGx:;G9BC(t;tY=Nb3W(t=wm3n.Q{GYGx:;G9BC";
struct bitmap_font xfont_8x13[1] = {{256, 8, 13, PCXX85, 2130, "xfont_8x13", xfont_8x13_data}};


#include "dataconv.c"

static struct bitmap_font uncompress_font(struct bitmap_font f)
{
	assert(f.packing == PCXX85);
	f.data = alloc_and_transform_from_X85_to_RAW(f.data, f.ndata, &f.ndata);
	f.data = alloc_and_transform_from_RLE8_to_RAW(f.data,f.ndata, &f.ndata);
	f.data = alloc_and_transform_from_RAW_to_BIT(f.data, f.ndata, &f.ndata);
	f.packing = UNPACKED;
	return f;
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
	if (i>=0 && j>=0 && i<w && j<h)
		for (int l = 0; l < pd; l++)
			x[(w*j+i)*pd+l] = c[l];
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
