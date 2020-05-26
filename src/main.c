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
#include "usb.h"

#include "printf.h"

#define BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS 0x0007E000
// This must match the value in `nrf52833_s140_v7.ld`
#define BOOTLOADER_REGION_START                                                \
    0x00076000 /**< This field should correspond to start address of the       \
                  bootloader, found in UICR.RESERVED, 0x10001014, register.    \
                  This value is used for sanity check, so the bootloader will  \
                  fail immediately if this value differs from runtime value.   \
                  The value is used to determine max application size for      \
                  updating. */
#define CODE_PAGE_SIZE                                                         \
    0x1000 /**< Size of a flash codepage. Used for size of the reserved flash  \
              space in the bootloader region. Will be runtime checked against  \
              NRF_UICR->CODEPAGESIZE to ensure the region is correct. */
uint8_t m_mbr_params_page[CODE_PAGE_SIZE] __attribute__((section(
    ".mbrParamsPage"))); /**< This variable reserves a codepage for mbr
                            parameters, to ensure the compiler doesn't locate
                            any code or variables at his location. */
volatile uint32_t m_uicr_mbr_params_page_address
    __attribute__((section(".uicrMbrParamsPageAddress"))) =
        BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS;

uint8_t m_boot_settings[CODE_PAGE_SIZE] __attribute__((
    section(".bootloaderSettings"))); /**< This variable reserves a codepage for
                                         bootloader specific settings, to ensure
                                         the compiler doesn't locate any code or
                                         variables at his location. */
volatile uint32_t m_uicr_bootloader_start_address __attribute__((
    section(".uicrBootStartAddress"))) =
    BOOTLOADER_REGION_START; /**< This variable ensures that the linker script
                                will write the bootloader start address to the
                                UICR register. This value will be written in the
                                HEX file and thus written to UICR when the
                                bootloader is flashed into the chip. */

__attribute__((used)) uint8_t id[3];
__attribute__((used)) float record_buffer_f[4096];
__attribute__((used)) int16_t record_buffer_i16[8192];
__attribute__((used)) int32_t record_buffer_i32[8192];

static const struct i2s_pin_config i2s_config = {
    .data_pin_number = (32 + 9),
    .bit_clock_pin_number = (0 + 12),
    .word_select_pin_number = (0 + 8),
};

static void background_tasks(void) {
    tud_task(); // tinyusb device task
    cdc_task();
}

int main(void) {
    int i;

    NRF_POWER->DCDCEN = 1UL;

    // Switch to the lfclk
    NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
    NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
    NRF_CLOCK->TASKS_HFCLKSTOP = 1UL;

    usb_init();
    tusb_init();

    while (!tud_cdc_n_connected(0)) {
        background_tasks();
    }

    // USB CDC is always unreliable during first connection. Use this cheesy
    // delay to work around that problem.
    for (i = 0; i < 16384; i++) {
        background_tasks();
    }

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

    printf("@SPI ID: %02x %02x %02x\n", id[0], id[1], id[2]);

    memset(record_buffer_f, 0, sizeof(record_buffer_f));
    memset(record_buffer_i16, 0, sizeof(record_buffer_i16));
    memset(record_buffer_i32, 0, sizeof(record_buffer_i32));
    record_to_buffer(&i2s_config, 'f', record_buffer_f,
                     sizeof(record_buffer_f) / sizeof(*record_buffer_f));
    record_to_buffer(&i2s_config, 'h', record_buffer_i16,
                     sizeof(record_buffer_i16) / sizeof(*record_buffer_i16));
    record_to_buffer(&i2s_config, 'i', record_buffer_i32,
                     sizeof(record_buffer_i32) / sizeof(*record_buffer_i32));


    while (1) {
        background_tasks();
    }

    NVIC_SystemReset();
}

// printf glue
void _putchar(char character) {
    if (character == '\n') {
        tud_cdc_n_write_char(0, '\r');
    }
    while (tud_cdc_n_write_char(0, character) < 1) {
        background_tasks();
    }
}
