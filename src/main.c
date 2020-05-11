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

#define SPI_MOSI 9
#define SPI_MISO (4) // P1
#define SPI_SCK 10
#define SPI_CSn (6) // P1
void spi_init(void) {
  NRF_P1->OUTSET = (1 << SPI_CSn);
  NRF_P1->DIRSET = (1 << SPI_CSn);

  NRF_P0->DIRSET = (1 << SPI_SCK) | (1 << SPI_MOSI);
  NRF_P1->DIRCLR = (1 << SPI_MISO);

  NRF_SPI1->ENABLE = 1UL;
  NRF_SPI1->PSELSCK = SPI_SCK;
  NRF_SPI1->PSELMOSI = SPI_MOSI;
  NRF_SPI1->PSELMISO = 32+SPI_MISO;
}

void spi_deinit(void) {
  NRF_SPI1->ENABLE = 0;

  // unsigned int i;
  // for (i = 0; i < 32; i++) {
  //   NRF_P0->PIN_CNF[i] |= 0x80000000;
  //   NRF_P1->PIN_CNF[i] |= 0x80000000;
  // }
}

void spi_select(void) {
  NRF_P1->OUTCLR = (1 << SPI_CSn);
}

uint8_t spi_xfer(uint8_t data) {
  NRF_SPI1->TXD = data;
  while (NRF_SPI1->EVENTS_READY == 0)
    ;
  NRF_SPI1->EVENTS_READY = 0;
  return NRF_SPI1->RXD;
}

void spi_deselect(void) {
  NRF_P1->OUTSET = (1 << SPI_CSn);
}

__attribute__((used)) uint8_t id[3];

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

  // NRF_POWER->SYSTEMOFF = 1;
  while (1) {
    asm("wfi");
  }
  NVIC_SystemReset();
}
