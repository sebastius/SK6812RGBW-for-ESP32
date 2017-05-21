/*
 * A driver for the SK6812 RGB LEDs using the RMT peripheral on the ESP32.
 *
 * Further modifications done by Sebastius (Sebastian Oort) for the SK6812 RGBW
 * For the SHA2017 badge, https://wiki.sha2017.org/w/Projects:Badge
 *
 * Modifications Copyright (c) 2017 Martin F. Falatic
 *
 * Originally based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * The RMT peripheral on the ESP32 provides very accurate timing of
 * signals sent to the SK6812 LEDs.
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

#include "sk6812rgbw.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ARDUINO)
  #include "esp32-hal.h"
  #include "esp_intr.h"
  #include "driver/gpio.h"
  #include "driver/periph_ctrl.h"
  #include "freertos/semphr.h"
  #include "soc/rmt_struct.h"
#elif defined(ESP_PLATFORM)
  #include <esp_intr.h>
  #include <driver/gpio.h>
  #include <freertos/FreeRTOS.h>
  #include <freertos/semphr.h>
  #include <soc/dport_reg.h>
  #include <soc/gpio_sig_map.h>
  #include <soc/rmt_struct.h>
  #include <stdio.h>
#endif

#ifdef __cplusplus
}
#endif

#define RMTCHANNEL          0 /* There are 8 possible channels */
#define DIVIDER             4 /* 8 still seems to work, but timings become marginal */
#define MAX_PULSES         32 /* A channel has a 64 "pulse" buffer - we use half per pass */
#define RMT_DURATION_NS  12.5 /* minimum time of a single RMT duration based on clock ns */

typedef struct {
  uint32_t T0H;
  uint32_t T1H;
  uint32_t T0L;
  uint32_t T1L;
  uint32_t TRS;
} timingParams;

timingParams ledParams;
timingParams ledParams_SK6812  = { .T0H = 300, .T1H = 600, .T0L = 900, .T1L = 600, .TRS =  80000};

typedef union {
  struct {
    uint32_t duration0:15;
    uint32_t level0:1;
    uint32_t duration1:15;
    uint32_t level1:1;
  };
  uint32_t val;
} rmtPulsePair;

static uint8_t *sk6812_buffer = NULL;
static uint16_t sk6812_pos, sk6812_len, sk6812_half, sk6812_bufIsDirty;
static xSemaphoreHandle sk6812_sem = NULL;
static intr_handle_t rmt_intr_handle = NULL;
static rmtPulsePair sk6812_bitval_to_rmt_map[2];

void initRMTChannel(int rmtChannel)
{
  RMT.apb_conf.fifo_mask = 1;  //enable memory access, instead of FIFO mode.
  RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer
  RMT.conf_ch[rmtChannel].conf0.div_cnt = DIVIDER;
  RMT.conf_ch[rmtChannel].conf0.mem_size = 1;
  RMT.conf_ch[rmtChannel].conf0.carrier_en = 0;
  RMT.conf_ch[rmtChannel].conf0.carrier_out_lv = 1;
  RMT.conf_ch[rmtChannel].conf0.mem_pd = 0;

  RMT.conf_ch[rmtChannel].conf1.rx_en = 0;
  RMT.conf_ch[rmtChannel].conf1.mem_owner = 0;
  RMT.conf_ch[rmtChannel].conf1.tx_conti_mode = 0;    //loop back mode.
  RMT.conf_ch[rmtChannel].conf1.ref_always_on = 1;    // use apb clock: 80M
  RMT.conf_ch[rmtChannel].conf1.idle_out_en = 1;
  RMT.conf_ch[rmtChannel].conf1.idle_out_lv = 0;

  return;
}

void copyToRmtBlock_half()
{
  // This fills half an RMT block
  // When wraparound is happening, we want to keep the inactive half of the RMT block filled
  uint16_t i, j, offset, len, byteval;

  offset = sk6812_half * MAX_PULSES;
  sk6812_half = !sk6812_half;

  len = sk6812_len - sk6812_pos;
  if (len > (MAX_PULSES / 8))
    len = (MAX_PULSES / 8);

  if (!len) {
    if (!sk6812_bufIsDirty) {
      return;
    }
    // Clear the channel's data block and return
    for (i = 0; i < MAX_PULSES; i++) {
      RMTMEM.chan[RMTCHANNEL].data32[i + offset].val = 0;
    }
    sk6812_bufIsDirty = 0;
    return;
  }
  sk6812_bufIsDirty = 1;

  for (i = 0; i < len; i++) {
    byteval = sk6812_buffer[i + sk6812_pos];

    #if DEBUG_SK6812_DRIVER
      snprintf(sk6812_debugBuffer, sk6812_debugBufferSz, "%s%d(", sk6812_debugBuffer, byteval);
    #endif

    // Shift bits out, MSB first, setting RMTMEM.chan[n].data32[x] to the rmtPulsePair value corresponding to the buffered bit value
    for (j = 0; j < 8; j++, byteval <<= 1) {
      int bitval = (byteval >> 7) & 0x01;
      int data32_idx = i * 8 + offset + j;
      RMTMEM.chan[RMTCHANNEL].data32[data32_idx].val = sk6812_bitval_to_rmt_map[bitval].val;
      #if DEBUG_SK6812_DRIVER
        snprintf(sk6812_debugBuffer, sk6812_debugBufferSz, "%s%d", sk6812_debugBuffer, bitval);
      #endif
    }
    #if DEBUG_SK6812_DRIVER
      snprintf(sk6812_debugBuffer, sk6812_debugBufferSz, "%s) ", sk6812_debugBuffer);
    #endif

    // Handle the reset bit by stretching duration1 for the final bit in the stream
    if (i + sk6812_pos == sk6812_len - 1) {
      RMTMEM.chan[RMTCHANNEL].data32[i * 8 + offset + 7].duration1 =
        ledParams.TRS / (RMT_DURATION_NS * DIVIDER);
      #if DEBUG_SK6812_DRIVER
        snprintf(sk6812_debugBuffer, sk6812_debugBufferSz, "%sRESET ", sk6812_debugBuffer);
      #endif
    }
  }

  // Clear the remainder of the channel's data not set above
  for (i *= 8; i < MAX_PULSES; i++) {
    RMTMEM.chan[RMTCHANNEL].data32[i + offset].val = 0;
  }

  sk6812_pos += len;

#if DEBUG_SK6812_DRIVER
  snprintf(sk6812_debugBuffer, sk6812_debugBufferSz, "%s ", sk6812_debugBuffer);
#endif

  return;
}


void sk6812_handleInterrupt(void *arg)
{
  portBASE_TYPE taskAwoken = 0;

  if (RMT.int_st.ch0_tx_thr_event) {
    copyToRmtBlock_half();
    RMT.int_clr.ch0_tx_thr_event = 1;
  }
  else if (RMT.int_st.ch0_tx_end && sk6812_sem) {
    xSemaphoreGiveFromISR(sk6812_sem, &taskAwoken);
    RMT.int_clr.ch0_tx_end = 1;
  }

  return;
}

int sk6812_init(int gpioNum)
{
  #if DEBUG_SK6812_DRIVER
    sk6812_debugBuffer = (char*)calloc(sk6812_debugBufferSz, sizeof(char));
  #endif
  ledParams = ledParams_SK6812;


  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);

  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpioNum], 2);
  gpio_matrix_out((gpio_num_t)gpioNum, RMT_SIG_OUT0_IDX + RMTCHANNEL, 0, 0);
  gpio_set_direction((gpio_num_t)gpioNum, GPIO_MODE_OUTPUT);

  initRMTChannel(RMTCHANNEL);

  RMT.tx_lim_ch[RMTCHANNEL].limit = MAX_PULSES;
  RMT.int_ena.ch0_tx_thr_event = 1;
  RMT.int_ena.ch0_tx_end = 1;

  // RMT config for SK6812 bit val 0
  sk6812_bitval_to_rmt_map[0].level0 = 1;
  sk6812_bitval_to_rmt_map[0].level1 = 0;
  sk6812_bitval_to_rmt_map[0].duration0 = ledParams.T0H / (RMT_DURATION_NS * DIVIDER);
  sk6812_bitval_to_rmt_map[0].duration1 = ledParams.T0L / (RMT_DURATION_NS * DIVIDER);

  // RMT config for SK6812 bit val 1
  sk6812_bitval_to_rmt_map[1].level0 = 1;
  sk6812_bitval_to_rmt_map[1].level1 = 0;
  sk6812_bitval_to_rmt_map[1].duration0 = ledParams.T1H / (RMT_DURATION_NS * DIVIDER);
  sk6812_bitval_to_rmt_map[1].duration1 = ledParams.T1L / (RMT_DURATION_NS * DIVIDER);

  esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, sk6812_handleInterrupt, NULL, &rmt_intr_handle);

  return 0;
}

void sk6812_setColors(uint16_t length, rgbwVal *array)
{
  uint16_t i;

  sk6812_len = (length * 4) * sizeof(uint8_t);
  sk6812_buffer = (uint8_t *) malloc(sk6812_len);

  for (i = 0; i < length; i++) {
    // Where color order is translated from RGB (e.g., SK6812 = GRBW)
    sk6812_buffer[0 + i * 4] = array[i].g;
    sk6812_buffer[1 + i * 4] = array[i].r;
    sk6812_buffer[2 + i * 4] = array[i].b;
    sk6812_buffer[3 + i * 4] = array[i].w;
  }

  sk6812_pos = 0;
  sk6812_half = 0;

  copyToRmtBlock_half();

  if (sk6812_pos < sk6812_len) {
    // Fill the other half of the buffer block
    #if DEBUG_SK6812_DRIVER
      snprintf(sk6812_debugBuffer, sk6812_debugBufferSz, "%s# ", sk6812_debugBuffer);
    #endif
    copyToRmtBlock_half();
  }


  sk6812_sem = xSemaphoreCreateBinary();

  RMT.conf_ch[RMTCHANNEL].conf1.mem_rd_rst = 1;
  RMT.conf_ch[RMTCHANNEL].conf1.tx_start = 1;

  xSemaphoreTake(sk6812_sem, portMAX_DELAY);
  vSemaphoreDelete(sk6812_sem);
  sk6812_sem = NULL;

  free(sk6812_buffer);

  return;
}
