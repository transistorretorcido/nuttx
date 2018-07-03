/**************************************************************************************
 * drivers/lcd/st7567.h
 *
 * Definitions for the ST7567 128x64 Dot Matrix LCD
 * Driver with C
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Manikandan <code@zilogic.com>
 *
 * Based on drivers/lcd/ssd1305.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: SSD1305.pdf, "Solomon Systech SSD1305 136x64 Dot Matrix OLED/PLED
 *             Segment/Common Driver with Controller," Solomon Systech Limited,
 *             http://www.solomon-systech.com, May, 2008.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************/

#ifndef __DRIVERS_LCD_ST7735_H
#define __DRIVERS_LCD_ST7735_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Fundamental Commands *****************************************************/

#define ST7735_EXIT_SOFTRST        0x01 /* 0x01: Software RESET */
#define ST7735_SLPIN               0x10 /* 0x10: Sleep In*/
#define ST7735_SLPOUT              0x11 /* 0x11: Sleep Out*/
#define ST7735_DISPNORMAL          0x20 /* 0x20: Normal display */
#define ST7735_DISPINVERSE         0x21 /* 0x21: Inverse display */
#define ST7735_DISPOFF             0x28 /* 0x28: Display OFF */
#define ST7735_DISPON              0x29 /* 0x29: Display ON in normal mode */
#define ST7735_SETCOL              0x2A /* 0x2A: Set column address */
#define ST7735_SETROW              0x2B /* 0x2B: Set row address */
#define ST7735_RAMWR               0x2C /* 0x2C: RAM Write */
#define ST7735_MADCTL              0x36 /* 0X36: Rotation  */
#define ST7735_0DEG                0X00 /* 0x60: 0 degrees*/
#define ST7735_90DEG               0X60 /* 0x60: 90 degrees*/
#define ST7735_180DEG              0XC0 /* 0xC0: 180 degrees*/
#define ST7735_270DEG              0XA0 /* 0xA0: 270 degrees*/
#define ST7735_COLMODE             0x3A /* 0x3A: Select mode color*/
#define ST7735_COLMODE16B          0x05 /*0x05 -> 16bit (RGB 5-6-5) rrrrggggggbbbbb (2 bytes)*/
#define ST7735_COLMODe18B          0x06 /*0x06 -> 18bit (RGB 6-6-6) rrrrrrxxggggggxxbbbbbbxx (3bytes)*/

#endif /* __DRIVERS_LCD_ST7735_H */
