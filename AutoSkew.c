/*
 * AutoSkew.h
 *
 *  Created on: 2021Äê10ÔÂ25ÈÕ
 *      Author: Atlas
 */

#include "autoskew.h"

uint16_t
search_mid (
		uint16_t word_tmp
		)
{
	uint16_t mid_word = 0;

    if (word_tmp==0xffff) // 16
    	mid_word = 0x0180;
    else if (word_tmp==0x7fff) // 15
    	mid_word = 0x0080;
   	else if (word_tmp==0xfffe)
    	mid_word = 0x0100;
    else if ((word_tmp&0x3fff)==0x3fff) // 14
    	mid_word = 0x0040;
    else if ((word_tmp&0x7ffe)==0x7ffe)
    	mid_word = 0x0080;
    else if ((word_tmp&0xfffc)==0xfffc)
    	mid_word = 0x0200;
    else if ((word_tmp&0x1fff)==0x1fff) // 13
    	mid_word = 0x0040;
    else if ((word_tmp&0x3ffe)==0x3ffe)
    	mid_word = 0x0080;
    else if ((word_tmp&0x7ffc)==0x7ffc)
    	mid_word = 0x0100;
    else if ((word_tmp&0xfff8)==0xfff8)
    	mid_word = 0x0200;
    else if ((word_tmp&0x0fff)==0x0fff) // 12
    	mid_word = 0x0020;
    else if ((word_tmp&0x1ffe)==0x1ffe)
    	mid_word = 0x0040;
    else if ((word_tmp&0x3ffc)==0x3ffc)
    	mid_word = 0x0080;
    else if ((word_tmp&0x7ff8)==0x7ff8)
    	mid_word = 0x0100;
    else if ((word_tmp&0xfff0)==0xfff0)
    	mid_word = 0x0400;
    else if ((word_tmp&0x07ff)==0x07ff) // 11
    	mid_word = 0x0020;
    else if ((word_tmp&0x0ffe)==0x0ffe)
    	mid_word = 0x0040;
    else if ((word_tmp&0x1ffc)==0x1ffc)
    	mid_word = 0x0080;
    else if ((word_tmp&0x3ff8)==0x3ff8)
    	mid_word = 0x0100;
    else if ((word_tmp&0x7ff0)==0x7ff0)
    	mid_word = 0x0200;
    else if ((word_tmp&0xffe0)==0xffe0)
    	mid_word = 0x0400;
    else if ((word_tmp&0x03ff)==0x03ff) // 10
    	mid_word = 0x0010;
    else if ((word_tmp&0x07fe)==0x07fe)
    	mid_word = 0x0020;
    else if ((word_tmp&0x0ffc)==0x0ffc)
    	mid_word = 0x0040;
    else if ((word_tmp&0x1ff8)==0x1ff8)
    	mid_word = 0x0080;
    else if ((word_tmp&0x3ff0)==0x3ff0)
    	mid_word = 0x0100;
    else if ((word_tmp&0x7fe0)==0x7fe0)
    	mid_word = 0x0200;
    else if ((word_tmp&0xffc0)==0xffc0)
    	mid_word = 0x0800;
    else if ((word_tmp&0x01ff)==0x01ff) // 9
    	mid_word = 0x0010;
    else if ((word_tmp&0x03fe)==0x03fe)
    	mid_word = 0x0020;
    else if ((word_tmp&0x07fc)==0x07fc)
    	mid_word = 0x0040;
    else if ((word_tmp&0x0ff8)==0x0ff8)
    	mid_word = 0x0080;
    else if ((word_tmp&0x1ff0)==0x1ff0)
    	mid_word = 0x0100;
    else if ((word_tmp&0x3fe0)==0x3fe0)
    	mid_word = 0x0200;
    else if ((word_tmp&0x7fc0)==0x7fc0)
    	mid_word = 0x0400;
    else if ((word_tmp&0xff80)==0xff80)
    	mid_word = 0x0800;
    else if ((word_tmp&0x00ff)==0x00ff) // 8
    	mid_word = 0x0008;
    else if ((word_tmp&0x01fe)==0x01fe)
    	mid_word = 0x0010;
    else if ((word_tmp&0x03fc)==0x03fc)
    	mid_word = 0x0020;
    else if ((word_tmp&0x07f8)==0x07f8)
    	mid_word = 0x0040;
    else if ((word_tmp&0x0ff0)==0x0ff0)
    	mid_word = 0x0080;
    else if ((word_tmp&0x1fe0)==0x1fe0)
    	mid_word = 0x0100;
    else if ((word_tmp&0x3fc0)==0x3fc0)
    	mid_word = 0x0200;
    else if ((word_tmp&0x7f80)==0x7f80)
    	mid_word = 0x0400;
    else if ((word_tmp&0xff00)==0xff00)
    	mid_word = 0x0800;
    else if (((word_tmp&0x007f)==0x007f)|| // 7
    		 ((word_tmp&0x00fe)==0x00fe)||
    		 ((word_tmp&0x7f00)==0x7f00)||
    		 ((word_tmp&0xfe00)==0xfe00))
    	{
    	if ((word_tmp&0x007f)==0x007f) mid_word = 0x0008;
    	if ((word_tmp&0x00fe)==0x00fe) mid_word = 0x0010;
    	if ((word_tmp&0x7f00)==0x7f00) mid_word |= 0x0800;
    	if ((word_tmp&0xfe00)==0xfe00) mid_word |= 0x1000;
    	}
    else if ((word_tmp&0x01fc)==0x01fc)
    	mid_word = 0x0020;
    else if ((word_tmp&0x03f8)==0x03f8)
    	mid_word = 0x0040;
    else if ((word_tmp&0x07f0)==0x07f0)
    	mid_word = 0x0080;
    else if ((word_tmp&0x0fe0)==0x0fe0)
    	mid_word = 0x0100;
    else if ((word_tmp&0x1fc0)==0x1fc0)
    	mid_word = 0x0200;
    else if ((word_tmp&0x3f80)==0x3f80)
    	mid_word = 0x0400;
    else if (((word_tmp&0x003f)==0x003f)|| // 6
    		 ((word_tmp&0x007e)==0x007e)||
    		 ((word_tmp&0x00fc)==0x00fc)||
    		 ((word_tmp&0x01f8)==0x01f8)||
    		 ((word_tmp&0x1f80)==0x1f80)||
    		 ((word_tmp&0x3f00)==0x3f00)||
    		 ((word_tmp&0x7e00)==0x7e00)||
    		 ((word_tmp&0xfc00)==0xfc00))
    	{
    	if ((word_tmp&0x003f)==0x003f) mid_word = 0x0004;
    	if ((word_tmp&0x007e)==0x007e) mid_word = 0x0008;
    	if ((word_tmp&0x00fc)==0x00fc) mid_word = 0x0010;
    	if ((word_tmp&0x01f8)==0x01f8) mid_word = 0x0020;
    	if ((word_tmp&0x1f80)==0x1f80) mid_word |= 0x0200;
    	if ((word_tmp&0x3f00)==0x3f00) mid_word |= 0x0400;
    	if ((word_tmp&0x7e00)==0x7e00) mid_word |= 0x0800;
    	if ((word_tmp&0xfc00)==0xfc00) mid_word |= 0x2000;
    	}
    else if ((word_tmp&0x03f0)==0x03f0)
    	mid_word = 0x0040;
    else if ((word_tmp&0x07e0)==0x07e0)
    	mid_word = 0x0080;
    else if ((word_tmp&0x0fc0)==0x0fc0)
    	mid_word = 0x0100;
    else if (((word_tmp&0x001f)==0x001f)|| // 5
    		 ((word_tmp&0x003e)==0x003e)||
    		 ((word_tmp&0x007c)==0x007c)||
    		 ((word_tmp&0x00f8)==0x00f8)||
    		 ((word_tmp&0x01f0)==0x01f0)||
    		 ((word_tmp&0x03e0)==0x03e0)||
    		 ((word_tmp&0x07c0)==0x07c0)||
    		 ((word_tmp&0x0f80)==0x0f80)||
    		 ((word_tmp&0x1f00)==0x1f00)||
    		 ((word_tmp&0x3e00)==0x3e00)||
    		 ((word_tmp&0x7c00)==0x7c00)||
    		 ((word_tmp&0xf800)==0xf800))
    	{
    	if ((word_tmp&0x001f)==0x001f) mid_word = 0x0004;
    	if ((word_tmp&0x003e)==0x003e) mid_word = 0x0008;
    	if ((word_tmp&0x007c)==0x007c) mid_word = 0x0010;
    	if ((word_tmp&0x00f8)==0x00f8) mid_word = 0x0020;
    	if ((word_tmp&0x01f0)==0x01f0) mid_word = 0x0040;
    	if ((word_tmp&0x03e0)==0x03e0) mid_word = 0x0080;
    	if ((word_tmp&0x07c0)==0x07c0) mid_word |= 0x0100;
    	if ((word_tmp&0x0f80)==0x0f80) mid_word |= 0x0200;
    	if ((word_tmp&0x1f00)==0x1f00) mid_word |= 0x0400;
    	if ((word_tmp&0x3e00)==0x3e00) mid_word |= 0x0800;
    	if ((word_tmp&0x7c00)==0x7c00) mid_word |= 0x1000;
    	if ((word_tmp&0xf800)==0xf800) mid_word |= 0x2000;
    	}
    else if (((word_tmp&0x000f)==0x000f)|| // 4
    		 ((word_tmp&0x001e)==0x001e)||
    		 ((word_tmp&0x003c)==0x003c)||
    		 ((word_tmp&0x0078)==0x0078)||
    		 ((word_tmp&0x00f0)==0x00f0)||
    		 ((word_tmp&0x01e0)==0x01e0)||
    		 ((word_tmp&0x03c0)==0x03c0)||
    		 ((word_tmp&0x0780)==0x0780)||
    		 ((word_tmp&0x0f00)==0x0f00)||
    		 ((word_tmp&0x1e00)==0x1e00)||
    		 ((word_tmp&0x3c00)==0x3c00)||
    		 ((word_tmp&0x7800)==0x7800)||
    		 ((word_tmp&0xf000)==0xf000))
    	{
    	if ((word_tmp&0x001f)==0x000f) mid_word = 0x0002;
    	if ((word_tmp&0x003e)==0x001e) mid_word = 0x0004;
    	if ((word_tmp&0x007c)==0x003c) mid_word = 0x0008;
    	if ((word_tmp&0x00f8)==0x0078) mid_word = 0x0010;
    	if ((word_tmp&0x01f0)==0x00f0) mid_word = 0x0020;
    	if ((word_tmp&0x03e0)==0x01e0) mid_word |= 0x0040;
    	if ((word_tmp&0x03e0)==0x03c0) mid_word |= 0x0080;
    	if ((word_tmp&0x07c0)==0x0780) mid_word |= 0x0100;
    	if ((word_tmp&0x0f80)==0x0f00) mid_word |= 0x0200;
    	if ((word_tmp&0x1f00)==0x1e00) mid_word |= 0x0400;
    	if ((word_tmp&0x3e00)==0x3c00) mid_word |= 0x0800;
    	if ((word_tmp&0x7c00)==0x7800) mid_word |= 0x1000;
    	if ((word_tmp&0xf800)==0xf000) mid_word |= 0x4000;
    	}
    else if (((word_tmp&0x0007)==0x0007)|| // 3
    		 ((word_tmp&0x000e)==0x000e)||
    		 ((word_tmp&0x001c)==0x001c)||
    		 ((word_tmp&0x0038)==0x0038)||
    		 ((word_tmp&0x0070)==0x0070)||
    		 ((word_tmp&0x00e0)==0x00e0)||
    		 ((word_tmp&0x01c0)==0x01c0)||
    		 ((word_tmp&0x0380)==0x0380)||
    		 ((word_tmp&0x0700)==0x0700)||
    		 ((word_tmp&0x0e00)==0x0e00)||
    		 ((word_tmp&0x1c00)==0x1c00)||
    		 ((word_tmp&0x3800)==0x3800)||
    		 ((word_tmp&0x7000)==0x7000)||
    		 ((word_tmp&0xe000)==0xe000))
    	{
    	if ((word_tmp&0x0007)==0x0007) mid_word = 0x0002;
    	if ((word_tmp&0x000e)==0x000e) mid_word = 0x0004;
    	if ((word_tmp&0x001c)==0x001c) mid_word = 0x0008;
    	if ((word_tmp&0x0038)==0x0038) mid_word = 0x0010;
    	if ((word_tmp&0x0070)==0x0070) mid_word |= 0x0020;
    	if ((word_tmp&0x00e0)==0x00e0) mid_word |= 0x0040;
    	if ((word_tmp&0x01c0)==0x01c0) mid_word |= 0x0080;
    	if ((word_tmp&0x0380)==0x0380) mid_word |= 0x0100;
    	if ((word_tmp&0x0700)==0x0700) mid_word |= 0x0200;
    	if ((word_tmp&0x0e00)==0x0e00) mid_word |= 0x0400;
    	if ((word_tmp&0x1c00)==0x1c00) mid_word |= 0x0800;
    	if ((word_tmp&0x3800)==0x3800) mid_word |= 0x1000;
    	if ((word_tmp&0x7000)==0x7000) mid_word |= 0x2000;
    	if ((word_tmp&0xe000)==0xe000) mid_word |= 0x4000;
    	}
    else if (((word_tmp&0x0003)==0x0003)|| // 2
    		 ((word_tmp&0x0006)==0x0006)||
    		 ((word_tmp&0x000c)==0x000c)||
    		 ((word_tmp&0x0018)==0x0018)||
    		 ((word_tmp&0x0030)==0x0030)||
    		 ((word_tmp&0x0060)==0x0060)||
    		 ((word_tmp&0x00c0)==0x00c0)||
    		 ((word_tmp&0x0180)==0x0180)||
    		 ((word_tmp&0x0300)==0x0300)||
    		 ((word_tmp&0x0600)==0x0600)||
    		 ((word_tmp&0x0c00)==0x0c00)||
    		 ((word_tmp&0x1800)==0x1800)||
    		 ((word_tmp&0x3000)==0x3000)||
    		 ((word_tmp&0x6000)==0x6000)||
    		 ((word_tmp&0xc000)==0xc000))
    	{
    	if ((word_tmp&0x0003)==0x0003) mid_word = 0x0001;
    	if ((word_tmp&0x0006)==0x0006) mid_word = 0x0002;
    	if ((word_tmp&0x000c)==0x000c) mid_word = 0x0004;
    	if ((word_tmp&0x0018)==0x0018) mid_word |= 0x0008;
    	if ((word_tmp&0x0030)==0x0030) mid_word |= 0x0010;
    	if ((word_tmp&0x0060)==0x0060) mid_word |= 0x0020;
    	if ((word_tmp&0x00c0)==0x00c0) mid_word |= 0x0040;
    	if ((word_tmp&0x0180)==0x0180) mid_word |= 0x0080;
    	if ((word_tmp&0x0300)==0x0300) mid_word |= 0x0100;
    	if ((word_tmp&0x0600)==0x0600) mid_word |= 0x0200;
    	if ((word_tmp&0x0c00)==0x0c00) mid_word |= 0x0400;
    	if ((word_tmp&0x1800)==0x1800) mid_word |= 0x0800;
    	if ((word_tmp&0x3000)==0x3000) mid_word |= 0x1000;
    	if ((word_tmp&0x6000)==0x6000) mid_word |= 0x2000;
    	if ((word_tmp&0xc000)==0xc000) mid_word |= 0x8000;
    	}
    else
    	{
    	if ((word_tmp&0x0001)==0x0001) mid_word = 0x0001;
    	if ((word_tmp&0x0002)==0x0002) mid_word = 0x0002;
    	if ((word_tmp&0x0004)==0x0004) mid_word |= 0x0004;
    	if ((word_tmp&0x0008)==0x0008) mid_word |= 0x0008;
    	if ((word_tmp&0x0010)==0x0010) mid_word |= 0x0010;
    	if ((word_tmp&0x0020)==0x0020) mid_word |= 0x0020;
    	if ((word_tmp&0x0040)==0x0040) mid_word |= 0x0040;
    	if ((word_tmp&0x0080)==0x0080) mid_word |= 0x0080;
    	if ((word_tmp&0x0100)==0x0100) mid_word |= 0x0100;
    	if ((word_tmp&0x0200)==0x0200) mid_word |= 0x0200;
    	if ((word_tmp&0x0400)==0x0400) mid_word |= 0x0400;
    	if ((word_tmp&0x0800)==0x0800) mid_word |= 0x0800;
    	if ((word_tmp&0x1000)==0x1000) mid_word |= 0x1000;
    	if ((word_tmp&0x2000)==0x2000) mid_word |= 0x2000;
    	if ((word_tmp&0x4000)==0x4000) mid_word |= 0x4000;
    	if ((word_tmp&0x8000)==0x8000) mid_word |= 0x8000;
    	}

    return mid_word;
}
