/*
 * A driver for the SK6812 RGBW LEDs using the RMT peripheral on the ESP32.
 *
 * Further modifications done by Sebastius for the SK6812 RGBW
 * For the SHA2017 badge, https://wiki.sha2017.org/w/Projects:Badge
 *
 * Modifications Copyright (c) 2017 Martin F. Falatic
 *
 * Originally based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 */
/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SK6812_DRIVER_H
#define SK6812_DRIVER_H

#include <stdint.h>

typedef union {
  struct __attribute__ ((packed)) {
    uint8_t r, g, b, w;
  };
  uint32_t num;
} rgbwVal;

#define DEBUG_SK6812_DRIVER 0

#if DEBUG_sk6812_DRIVER
char *    sk6812_debugBuffer;
const int sk6812_debugBufferSz = 1024;
#endif

enum led_types {LED_SK6812};
extern int  sk6812_init(int gpioNum);
extern void sk6812_setColors(uint16_t length, rgbwVal *array);

inline rgbwVal makeRGBWVal(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  rgbwVal v;
  v.r = r;
  v.g = g;
  v.b = b;
  v.w = w;
  return v;
}

#endif /* SK6812_DRIVER_H */
