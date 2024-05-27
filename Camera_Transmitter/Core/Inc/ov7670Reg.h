/*
 * ov7670Reg.h
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */

#ifndef OV7670_OV7670REG_H_
#define OV7670_OV7670REG_H_


#define OV7670_ADDR 	0x42

/* OV7670 REGISTERS */

#define GAIN				0x00
#define BLUE_GAIN			0x01
#define RED_GAIN			0x02
#define VREF				0x03
#define COM1				0x04
#define AECHH				0x07
#define COM2				0x09
#define PID					0x0A	// Product ID Number (MSB) - 0x76
#define VER					0x0A	// Product ID Number (LSB) - 0x73
#define COM3				0x0C
#define COM4				0x0D
#define COM5				0x0E
#define COM6				0x0F
#define CLKRC				0x11
#define COM7				0x12
#define COM8				0x13
#define COM9				0x14
#define COM10				0x15
#define HSTART				0x17
#define HSTOP				0x18
#define VSTART				0x19
#define VSTOP				0x1A
#define MVFP				0x1E
#define HREF				0x32
#define TSLB				0x3A
#define COM11				0x3B
#define COM12				0x3C
#define COM13				0x3D
#define COM14				0x3E
#define COM15				0x40
#define COM16				0x41
#define COM17				0x42
#define MTX1				0x4F
#define MTX2				0x50
#define MTX3				0x51
#define MTX4				0x52
#define MTX5				0x53
#define MTX6				0x54
#define BRIGHT				0x55
#define CONTRAST			0x56
#define MTXS				0x58
#define DBLV				0x6B
#define SCALING_XSC			0x70
#define SCALING_YSC			0x71
#define SCALING_DCWCTR		0x72
#define SCALING_PCLK_DIV	0x73
#define SLOP				0x7A
#define GAM1				0x7B
#define GAM2				0x7C
#define GAM3				0x7D
#define GAM4				0x7E
#define GAM5				0x7F
#define GAM6				0x80
#define GAM7				0x81
#define GAM8				0x82
#define GAM9				0x83
#define GAM10				0x84
#define GAM11				0x85
#define GAM12				0x86
#define GAM13				0x87
#define GAM14				0x88
#define GAM15				0x89
#define RGB444				0x8C
#define RSRVD				0xB0

#define REG_END 			0xFF


const uint8_t OV7670_reg[][2] = {

  /* COLOR MODES & RESOLUTION */
  {COM7	 , 0x14},   	// 0x14 = QVGA size(RGB); 0x8 = QCIF(YUV), 0x0C = QCIF (RGB)
  {RGB444, 0x00},   	// RGB444 Disable
  {COM15 , 0xD0},   	// RGB565, 00 - FF
  {TSLB  , 0x0C},   	// UYVY
  {COM13 , 0x80},   	// gamma enable, UV auto adjust, UYVY
  {RSRVD , 0x84}, 		// important

  /* CLOCK SETTINGS */
  {COM3		       , 0x04},  	// DCW enable
  {COM14	       , 0x19},  	// manual scaling, pclk/=2
  {SCALING_XSC	   , 0x3A},  	// scaling_xsc
  {SCALING_YSC	   , 0x35},  	// scaling_ysc
  {SCALING_DCWCTR  , 0x11}, 	// down sample by 2
  {SCALING_PCLK_DIV, 0xf1}, 	// DSP clock /= 2

  /* WINDOWING */
  {HSTART, 0x16},   // HSTART
  {HSTOP , 0x04},   // HSTOP
  {HREF  , 0x80},   // HREF
  {VSTART, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
  {VSTOP , 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
  {VREF  , 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

  /* COLOR MATRIX COEFFICIENT */
  {MTX1, 0x80},
  {MTX2, 0x80},
  {MTX3, 0x00},
  {MTX4, 0x22},
  {MTX5, 0x5e},
  {MTX6, 0x80},
  {MTXS, 0x9e},

  /* GAMMA CURVE */
  {COM16, 0x38},
  {GAM1 , 16  },
  {GAM2 , 30  },
  {GAM3 , 53  },
  {GAM4 , 90  },
  {GAM5 , 105 },
  {GAM6 , 118 },
  {GAM7 , 130 },
  {GAM8 , 140 },
  {GAM9 , 150 },
  {GAM10, 160 },
  {GAM11, 180 },
  {GAM12, 195 },
  {GAM13, 215 },
  {GAM14, 230 },
  {GAM15, 244 },
  {SLOP , 16  },

  /* FPS */
  {CLKRC, 0x00}, // pre-scalar = 1/1

  /* END OF THE REGISTERS */
  {REG_END, REG_END},
};





#endif /* OV7670_OV7670REG_H_ */
