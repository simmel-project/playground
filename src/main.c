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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "nrfx_config.h"
#include "nrfx.h"
#include "nrf_clock.h"
#include "nrfx_power.h"
#include "nrfx_pwm.h"

#include "nrf.h"

#include "nrfx_nvmc.h"
#include "nrfx_spi.h"

__attribute__((noinline))
void spi_init(void) {
  NRF_SPI1->ENABLE = 1UL;
}

int main(void)
{
  // board_init();

  // // Reset Board
  // board_teardown();

  NRF_POWER->DCDCEN = 1UL;

  // Switch to the lfclk
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  spi_init();

  while (1) {
    asm("wfi");
  }
  NVIC_SystemReset();
}
