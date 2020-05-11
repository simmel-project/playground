/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
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

/**
 * -# Receive start data packet.
 * -# Based on start packet, prepare NVM area to store received data.
 * -# Receive data packet.
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf_clock.h"
#include "nrfx.h"
#include "nrfx_config.h"
#include "nrfx_power.h"
#include "nrfx_pwm.h"

#include "nrf.h"

#include "nrfx_nvmc.h"
#include "nrfx_spi.h"

#include "i2s.h"
#include "spi.h"

__attribute__((used)) uint8_t id[3];
__attribute__((used)) float record_buffer_f[4096];
__attribute__((used)) int32_t record_buffer_i[4096];

static const struct i2s_pin_config i2s_config = {
    .data_pin_number = (32 + 9),
    .bit_clock_pin_number = (0 + 12),
    .word_select_pin_number = (0 + 8),
};

int main(void) {
  NRF_POWER->DCDCEN = 1UL;

  // Switch to the lfclk
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  NRF_CLOCK->TASKS_HFCLKSTOP = 1UL;

  spi_init();

  // Get the SPI ID, just to make sure things are working.
  spi_select();
  spi_xfer(0x9f);
  id[0] = spi_xfer(0xff);
  id[1] = spi_xfer(0xff);
  id[2] = spi_xfer(0xff);
  spi_deselect();

  // Issue "Deep Power Down"
  spi_select();
  spi_xfer(0xb9);
  spi_deselect();

  spi_deinit();

  const uint32_t sample_rate = 16000;
  memset(record_buffer_f, 0, sizeof(record_buffer_f));
  memset(record_buffer_i, 0, sizeof(record_buffer_i));
  record_to_buffer(&i2s_config, sample_rate, 'f', record_buffer_f,
                   sizeof(record_buffer_f) / sizeof(*record_buffer_f));
  record_to_buffer(&i2s_config, sample_rate, 'i', record_buffer_i,
                   sizeof(record_buffer_i) / sizeof(*record_buffer_i));
  while (1) {
    asm("bkpt");
  }
  NVIC_SystemReset();
}
