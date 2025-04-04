/*
	Color utility functions for embedded systems

	Copyright 2023 <>< Charles Lohr, under the MIT-x11 or NewBSD License, you choose!
*/

#ifndef _COLOR_UTILITIES_H
#define _COLOR_UTILITIES_H

// To stop warnings about unused functions.
static uint32_t EHSVtoHEX( uint8_t hue, uint8_t sat, uint8_t val ) __attribute__((used));
static uint32_t TweenHexColors( uint32_t hexa, uint32_t hexb, int tween ) __attribute__((used));

static uint32_t EHSVtoHEX( uint8_t hue, uint8_t sat, uint8_t val )
{
	#define SIXTH1 43
	#define SIXTH2 85
	#define SIXTH3 128
	#define SIXTH4 171
	#define SIXTH5 213

	uint16_t or = 0, og = 0, ob = 0;

	hue -= SIXTH1; //Off by 60 degrees.

	//TODO: There are colors that overlap here, consider 
	//tweaking this to make the best use of the colorspace.

	if( hue < SIXTH1 ) //Ok: Yellow->Red.
	{
		or = 255;
		og = 255 - ((uint16_t)hue * 255) / (SIXTH1);
	}
	else if( hue < SIXTH2 ) //Ok: Red->Purple
	{
		or = 255;
		ob = (uint16_t)hue*255 / SIXTH1 - 255;
	}
	else if( hue < SIXTH3 )  //Ok: Purple->Blue
	{
		ob = 255;
		or = ((SIXTH3-hue) * 255) / (SIXTH1);
	}
	else if( hue < SIXTH4 ) //Ok: Blue->Cyan
	{
		ob = 255;
		og = (hue - SIXTH3)*255 / SIXTH1;
	}
	else if( hue < SIXTH5 ) //Ok: Cyan->Green.
	{
		og = 255;
		ob = ((SIXTH5-hue)*255) / SIXTH1;
	}
	else //Green->Yellow
	{
		og = 255;
		or = (hue - SIXTH5) * 255 / SIXTH1;
	}

	uint16_t rv = val;
	if( rv > 128 ) rv++;
	uint16_t rs = sat;
	if( rs > 128 ) rs++;

	//or, og, ob range from 0...255 now.
	//Need to apply saturation and value.

	or = (or * val)>>8;
	og = (og * val)>>8;
	ob = (ob * val)>>8;

	//OR..OB == 0..65025
	or = or * rs + 255 * (256-rs);
	og = og * rs + 255 * (256-rs);
	ob = ob * rs + 255 * (256-rs);
//printf( "__%d %d %d =-> %d\n", or, og, ob, rs );

	or >>= 8;
	og >>= 8;
	ob >>= 8;

	return or | (og<<8) | ((uint32_t)ob<<16);
}

static const unsigned char huetable[] = {
	0x00, 0x06, 0x0c, 0x12, 0x18, 0x1e, 0x24, 0x2a, 0x30, 0x36, 0x3c, 0x42, 0x48, 0x4e, 0x54, 0x5a, 
	0x60, 0x66, 0x6c, 0x72, 0x78, 0x7e, 0x84, 0x8a, 0x90, 0x96, 0x9c, 0xa2, 0xa8, 0xae, 0xb4, 0xba, 
	0xc0, 0xc6, 0xcc, 0xd2, 0xd8, 0xde, 0xe4, 0xea, 0xf0, 0xf6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xf9, 0xf3, 0xed, 0xe7, 0xe1, 0xdb, 0xd5, 0xcf, 0xc9, 0xc3, 0xbd, 0xb7, 0xb1, 0xab, 0xa5, 
	0x9f, 0x99, 0x93, 0x8d, 0x87, 0x81, 0x7b, 0x75, 0x6f, 0x69, 0x63, 0x5d, 0x57, 0x51, 0x4b, 0x45, 
	0x3f, 0x39, 0x33, 0x2d, 0x27, 0x21, 0x1b, 0x15, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

static const unsigned char rands[] = {
	0x67, 0xc6, 0x69, 0x73, 0x51, 0xff, 0x4a, 0xec, 0x29, 0xcd, 0xba, 0xab, 0xf2, 0xfb, 0xe3, 0x46, 
	0x7c, 0xc2, 0x54, 0xf8, 0x1b, 0xe8, 0xe7, 0x8d, 0x76, 0x5a, 0x2e, 0x63, 0x33, 0x9f, 0xc9, 0x9a, 
	0x66, 0x32, 0x0d, 0xb7, 0x31, 0x58, 0xa3, 0x5a, 0x25, 0x5d, 0x05, 0x17, 0x58, 0xe9, 0x5e, 0xd4, 
	0xab, 0xb2, 0xcd, 0xc6, 0x9b, 0xb4, 0x54, 0x11, 0x0e, 0x82, 0x74, 0x41, 0x21, 0x3d, 0xdc, 0x87, 
	0x70, 0xe9, 0x3e, 0xa1, 0x41, 0xe1, 0xfc, 0x67, 0x3e, 0x01, 0x7e, 0x97, 0xea, 0xdc, 0x6b, 0x96, 
	0x8f, 0x38, 0x5c, 0x2a, 0xec, 0xb0, 0x3b, 0xfb, 0x32, 0xaf, 0x3c, 0x54, 0xec, 0x18, 0xdb, 0x5c, 
	0x02, 0x1a, 0xfe, 0x43, 0xfb, 0xfa, 0xaa, 0x3a, 0xfb, 0x29, 0xd1, 0xe6, 0x05, 0x3c, 0x7c, 0x94, 
	0x75, 0xd8, 0xbe, 0x61, 0x89, 0xf9, 0x5c, 0xbb, 0xa8, 0x99, 0x0f, 0x95, 0xb1, 0xeb, 0xf1, 0xb3, 
	0x05, 0xef, 0xf7, 0x00, 0xe9, 0xa1, 0x3a, 0xe5, 0xca, 0x0b, 0xcb, 0xd0, 0x48, 0x47, 0x64, 0xbd, 
	0x1f, 0x23, 0x1e, 0xa8, 0x1c, 0x7b, 0x64, 0xc5, 0x14, 0x73, 0x5a, 0xc5, 0x5e, 0x4b, 0x79, 0x63, 
	0x3b, 0x70, 0x64, 0x24, 0x11, 0x9e, 0x09, 0xdc, 0xaa, 0xd4, 0xac, 0xf2, 0x1b, 0x10, 0xaf, 0x3b, 
	0x33, 0xcd, 0xe3, 0x50, 0x48, 0x47, 0x15, 0x5c, 0xbb, 0x6f, 0x22, 0x19, 0xba, 0x9b, 0x7d, 0xf5, 
	0x0b, 0xe1, 0x1a, 0x1c, 0x7f, 0x23, 0xf8, 0x29, 0xf8, 0xa4, 0x1b, 0x13, 0xb5, 0xca, 0x4e, 0xe8, 
	0x98, 0x32, 0x38, 0xe0, 0x79, 0x4d, 0x3d, 0x34, 0xbc, 0x5f, 0x4e, 0x77, 0xfa, 0xcb, 0x6c, 0x05, 
	0xac, 0x86, 0x21, 0x2b, 0xaa, 0x1a, 0x55, 0xa2, 0xbe, 0x70, 0xb5, 0x73, 0x3b, 0x04, 0x5c, 0xd3, 
	0x36, 0x94, 0xb3, 0xaf, 0xe2, 0xf0, 0xe4, 0x9e, 0x4f, 0x32, 0x15, 0x49, 0xfd, 0x82, 0x4e, 0xa9, };

static const unsigned char sintable[] = {
	0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95, 0x99, 0x9c, 0x9f, 0xa2, 0xa5, 0xa8, 0xab, 0xad, 
	0xb0, 0xb3, 0xb6, 0xb9, 0xbc, 0xbe, 0xc1, 0xc4, 0xc6, 0xc9, 0xcb, 0xce, 0xd0, 0xd3, 0xd5, 0xd7, 
	0xda, 0xdc, 0xde, 0xe0, 0xe2, 0xe4, 0xe6, 0xe8, 0xe9, 0xeb, 0xed, 0xee, 0xf0, 0xf1, 0xf3, 0xf4, 
	0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfc, 0xfd, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfc, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf6, 
	0xf5, 0xf4, 0xf3, 0xf1, 0xf0, 0xee, 0xed, 0xeb, 0xe9, 0xe8, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdc, 
	0xda, 0xd7, 0xd5, 0xd3, 0xd0, 0xce, 0xcb, 0xc9, 0xc6, 0xc4, 0xc1, 0xbe, 0xbc, 0xb9, 0xb6, 0xb3, 
	0xb0, 0xad, 0xab, 0xa8, 0xa5, 0xa2, 0x9f, 0x9c, 0x99, 0x95, 0x92, 0x8f, 0x8c, 0x89, 0x86, 0x83, 
	0x80, 0x7d, 0x79, 0x76, 0x73, 0x70, 0x6d, 0x6a, 0x67, 0x64, 0x61, 0x5e, 0x5b, 0x58, 0x55, 0x52, 
	0x4f, 0x4c, 0x49, 0x47, 0x44, 0x41, 0x3e, 0x3c, 0x39, 0x36, 0x34, 0x31, 0x2f, 0x2d, 0x2a, 0x28, 
	0x26, 0x24, 0x21, 0x1f, 0x1d, 0x1b, 0x1a, 0x18, 0x16, 0x14, 0x13, 0x11, 0x10, 0x0e, 0x0d, 0x0b, 
	0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x04, 0x03, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 
	0x0a, 0x0b, 0x0d, 0x0e, 0x10, 0x11, 0x13, 0x14, 0x16, 0x18, 0x1a, 0x1b, 0x1d, 0x1f, 0x21, 0x24, 
	0x26, 0x28, 0x2a, 0x2d, 0x2f, 0x31, 0x34, 0x36, 0x39, 0x3c, 0x3e, 0x41, 0x44, 0x47, 0x49, 0x4c, 
	0x4f, 0x52, 0x55, 0x58, 0x5b, 0x5e, 0x61, 0x64, 0x67, 0x6a, 0x6d, 0x70, 0x73, 0x76, 0x79, 0x7d, };

static inline uint32_t FastMultiply( uint32_t big_num, uint32_t small_num ) __attribute__((section(".srodata")));
static inline uint32_t FastMultiply( uint32_t big_num, uint32_t small_num )
{
	// The CH32V003 is an EC core, so no hardware multiply. GCC's way multiply
	// is slow, so I wrote this.
	//
	// This basically does this:
	//	return small_num * big_num;
	//
	// Note: This does NOT check for zero to begin with, though this still
	// produces the correct results, it is a little weird that even if
	// small_num is zero it executes once.
	//
	// Additionally note, instead of the if( m&1 ) you can do the following:
	//  ret += multiplciant & neg(multiplicand & 1).
	//
	// BUT! Shockingly! That is slower than an extra branch! The CH32V003
	//  can branch unbelievably fast.
	//
	// This is functionally equivelent and much faster.
	//
	// Perf numbers, with small_num set to 180V.
	//  No multiply:         21.3% CPU Usage
	//  Assembly below:      42.4% CPU Usage  (1608 bytes for whole program)
	//  C version:           41.4% CPU Usage  (1600 bytes for whole program)
	//  Using GCC (__mulsi3) 65.4% CPU Usage  (1652 bytes for whole program)
	//
	// The multiply can be done manually:
	uint32_t ret = 0;
	uint32_t multiplicand = small_num;
	uint32_t mutliplicant = big_num;
	do
	{
		if( multiplicand & 1 )
			ret += mutliplicant;
		mutliplicant<<=1;
		multiplicand>>=1;
	} while( multiplicand );
	return ret;

	// Which is equivelent to the following assembly (If you were curious)
/*
	uint32_t ret = 0;
	asm volatile( "\n\
		.option   rvc;\n\
	1:	andi t0, %[small], 1\n\
		beqz t0, 2f\n\
		add %[ret], %[ret], %[big]\n\
	2:	srli %[small], %[small], 1\n\
		slli %[big], %[big], 1\n\
		bnez %[small], 1b\n\
	" :
		[ret]"=&r"(ret), [big]"+&r"(big_num), [small]"+&r"(small_num) : :
		"t0" );
	return ret;
*/
}

static uint32_t TweenHexColors( uint32_t hexa, uint32_t hexb, int tween )
{
	if( tween <= 0 ) return hexa;
	if( tween >= 255 ) return hexb;
	int32_t aamt = 255-tween;
	int32_t bamt = tween;
	int32_t hab = hexa & 0xff;
	int32_t har = (hexa>>8) & 0xff;
	int32_t hag = (hexa>>16) & 0xff;
	int32_t hbb = hexb & 0xff;
	int32_t hbr = (hexb>>8) & 0xff;
	int32_t hbg = (hexb>>16) & 0xff;
	int32_t b = (FastMultiply( hab, aamt ) + FastMultiply( hbb, bamt ) + 128) >> 8;
	int32_t r = (FastMultiply( har, aamt ) + FastMultiply( hbr, bamt ) + 128) >> 8;
	int32_t g = (FastMultiply( hag, aamt ) + FastMultiply( hbg, bamt ) + 128) >> 8;
	return b | (r<<8) | (g<<16);
}

#endif
