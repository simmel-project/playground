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

// #define RECORD_TEST_16
// #define RECORD_TEST_32

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

#include "afsk.h"
#include "i2s.h"
#include "spi.h"
#include "usb.h"

#include "printf.h"

static const struct demod_config nus_cfg = {
    .sample_rate = 62500,

    // .baud_rate = 3564,
    // .f_lo = 10134,
    // .f_hi = 17097,
    // .filter_width = 9,

    .baud_rate = 4784,
    .f_lo = 6017,
    .f_hi = 11452,
    .filter_width = 8,

    // .baud_rate = 8000,
    // .f_lo = 8666,
    // .f_hi = 12500,
    // .filter_width = 9,
};

#define BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS 0x0007E000
// This must match the value in `nrf52833_s140_v7.ld`
#define BOOTLOADER_REGION_START                                                \
    0x00070000 /**< This field should correspond to start address of the       \
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
#if defined(RECORD_TEST_16) || defined(RECORD_TEST_32)
#define RECORD_BUFFER_SIZE 1024
#else
#define RECORD_BUFFER_SIZE 2048
#endif
__attribute__((used, aligned(4))) int8_t record_buffer[RECORD_BUFFER_SIZE];

static const struct i2s_pin_config i2s_config = {
    .data_pin_number = (32 + 9),
    .bit_clock_pin_number = (0 + 12),
    .word_select_pin_number = (0 + 8),
};

volatile uint32_t i2s_irqs;
volatile int16_t *i2s_buffer = NULL;
volatile bool i2s_ready = false;

#ifdef RECORD_TEST_32
static uint8_t data_buffer[81920];
static uint32_t data_buffer_offset;
#endif

#ifdef RECORD_TEST_16
#define DATA_BUFFER_ELEMENT_COUNT (81920 / 2)
static int16_t data_buffer[DATA_BUFFER_ELEMENT_COUNT];
static uint32_t data_buffer_offset;
#endif


static int32_t agc_run(int32_t current_peak) {
    static float agc_offset = 10;
    if (current_peak > agc_offset) {
        agc_offset *= 1.7;
    } else if ((current_peak < agc_offset - 400) && agc_offset > 101) {
        agc_offset *= 0.995;
    }

    // Target the audio to be +/- 16384 (this value / 2)
    int32_t agc_div = (agc_offset / 20000.0);
    if (agc_div < 1) {
        agc_div = 1;
    }
    return agc_div;
}

static void background_tasks(void) {
    tud_task(); // tinyusb device task
    cdc_task();

    if (i2s_ready) {
        i2s_ready = false;

        const size_t samples_per_loop =
            RECORD_BUFFER_SIZE / 2 / sizeof(int32_t) / 2;

        // Cast to 32-bit int pointer, since we need to swap the values.
        uint32_t *input_buffer = (uint32_t *)i2s_buffer;

        // Current peak value in this buffer. Always a positive value.
        int32_t current_peak = 0;
        unsigned int i;
        static int32_t agc_div;

#if defined(RECORD_TEST_16)
        static int16_t *output_buffer_ptr = &data_buffer[0];
        // Every other 32-bit word in `input_buffer` is 0, so skip over it.
        // Additionally, we're only interested in one half of the record
        // buffer, since the other half is being written to.
        for (i = 0; i < samples_per_loop; i++) {
            uint32_t unswapped = input_buffer[i * 2];
            int32_t swapped = (int32_t)((((unswapped >> 16) & 0xffff) |
                                         ((unswapped << 16) & 0xffff0000)));
            if (swapped > current_peak) {
                current_peak = swapped;
            }

            swapped = swapped / agc_div;
            *output_buffer_ptr++ = swapped;

            // Advance the output buffer pointer, wrapping when it reaches the
            // end.
            if (output_buffer_ptr >= &data_buffer[DATA_BUFFER_ELEMENT_COUNT])
                output_buffer_ptr = &data_buffer[0];
        }

        // Advance the master pointer to the output buffer, which we will use
        // in the next iteration of this loop.
        if (data_buffer_offset >= sizeof(data_buffer) / sizeof(*data_buffer)) {
            data_buffer_offset = 0;
        }

        if (current_peak > agc_peak) {
            agc_peak += 1024;
        } else if ((current_peak < agc_peak - 1024) && agc_peak > 256) {
            agc_peak -= 256;
        }
        // Target the audio to be +/- 16384 (this value / 2)
        agc_div = (agc_peak / 8192);
        if (agc_div < 1) {
            agc_div = 1;
        };

        static int loops = 0;
        loops++;
        if ((loops & 31) == 0) {
            printf("agc_div: %d  agc_peak: %d  current_peak: %d\n", agc_div,
                   agc_peak, current_peak);
        }
#elif defined(RECORD_TEST_32)
        // Used for sampling during development. Read out the buffer with:
        // ```
        // (gdb) dump binary memory tone.raw data_buffer
        // data_buffer+sizeof(data_buffer)
        // ```
        unsigned int i;
        uint32_t *output_buffer = (uint32_t *)&data_buffer[data_buffer_offset];
        uint32_t *input_buffer = (uint32_t *)i2s_buffer;
        for (i = 0; i < RECORD_BUFFER_SIZE / 2 / sizeof(uint32_t); i += 2) {
            uint32_t words = input_buffer[i];
            output_buffer[i / 2] =
                (((words >> 16) & 0xffff) | ((words << 16) & 0xffff0000)) << 8;
        }
        data_buffer_offset += RECORD_BUFFER_SIZE / 4;
        if (data_buffer_offset >= sizeof(data_buffer)) {
            data_buffer_offset = 0;
        }
#else
        // Every other 32-bit word in `input_buffer` is 0, so skip over it.
        // Additionally, we're only interested in one half of the record
        // buffer, since the other half is being written to.
        int16_t output_buffer[samples_per_loop];
        int16_t *output_buffer_ptr = output_buffer;
        uint32_t clipped = 0;
        for (i = 0; i < samples_per_loop; i++) {
            uint32_t unswapped = input_buffer[i * 2];
            int32_t swapped = (int32_t)((((unswapped >> 16) & 0xffff) |
                                         ((unswapped << 16) & 0xffff0000)));

            // Add samples that are greater than 0 to an AGC mask. We'll
            // determine how many leading zeroes there are to calculate how many
            // bits to shift by during the next run.
            if (swapped > current_peak) {
                current_peak = swapped;
            }

            swapped = swapped / agc_div;
            if ((swapped > 32768) || (swapped < -32767)) {
                clipped++;
            }
            *output_buffer_ptr++ = swapped;
        }
        if (clipped) {
            printf("Clipped %d samples\n", clipped);
        }
        afsk_run(output_buffer, sizeof(output_buffer) / sizeof(*output_buffer));
        agc_div = agc_run(current_peak);
#endif

        // unsigned int i;
        // uint16_t *output_buffer = (uint16_t
        // *)&data_buffer[data_buffer_offset]; uint16_t *input_buffer =
        // (uint16_t *)i2s_buffer; for (i = 0; i < RECORD_BUFFER_SIZE / 2 /
        // sizeof(uint16_t); i+=2) {
        //     output_buffer[i/2] = input_buffer[i];
        // }
        // data_buffer_offset += RECORD_BUFFER_SIZE / 4;

        // memcpy(&data_buffer[data_buffer_offset], (void *)i2s_buffer,
        //        RECORD_BUFFER_SIZE / 2);
        // data_buffer_offset += RECORD_BUFFER_SIZE / 2;

        // if (data_buffer_offset >= sizeof(data_buffer)) {
        //     data_buffer_offset = 0;
        // }
        if (i2s_ready) {
            printf("I2S UNDERRUN!!!\n");
        }
    }
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
    afsk_init(&nus_cfg);

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

    nus_init(&i2s_config, record_buffer, sizeof(record_buffer));
    nus_start();

    while (!tud_cdc_n_connected(0)) {
        background_tasks();
    }

    // USB CDC is always unreliable during first connection. Use this cheesy
    // delay to work around that problem.
    for (i = 0; i < 16384; i++) {
        background_tasks();
    }

    printf("SPI ID: %02x %02x %02x\n", id[0], id[1], id[2]);

    uint32_t usb_irqs = 0;
    uint32_t last_i2s_irqs = i2s_irqs;
    while (1) {
        background_tasks();
        usb_irqs++;
        if (i2s_irqs != last_i2s_irqs) {
            // printf("I2S IRQ count: %d  USB IRQ count: %d\n", i2s_irqs,
            // usb_irqs);
            last_i2s_irqs = i2s_irqs;
            usb_irqs = 0;
        }
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
