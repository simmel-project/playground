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

#include <math.h>
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

#include "bpsk.h"
#include "i2s.h"
#include "spi.h"
#include "usb.h"

#include "printf.h"

#define MEASURE_RUNTIME

__attribute__((used)) uint8_t id[3];
#if defined(RECORD_TEST_16) || defined(RECORD_TEST_32)
#define RECORD_BUFFER_SIZE 1024
#else
#define RECORD_BUFFER_SIZE 4096
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
#define DATA_BUFFER_ELEMENT_COUNT (81920 / 4)
static uint8_t data_buffer[DATA_BUFFER_ELEMENT_COUNT];
#endif

#ifdef RECORD_TEST_16
#define DATA_BUFFER_ELEMENT_COUNT (81920 / 2)
static int16_t data_buffer[DATA_BUFFER_ELEMENT_COUNT];
#endif

uint32_t i2s_runs = 0;
static void background_tasks(void) {
    tud_task(); // tinyusb device task
    cdc_task();

    if (i2s_ready) {
        i2s_ready = false;
        i2s_runs++;

        const size_t samples_per_loop =
            RECORD_BUFFER_SIZE / 2 / sizeof(uint32_t) / 2;

        // Cast to 32-bit int pointer, since we need to swap the values.
        uint32_t *input_buffer = (uint32_t *)i2s_buffer;
        unsigned int i;

#if defined(RECORD_TEST_16)
        static int16_t *output_buffer_ptr = &data_buffer[0];
        // Every other 32-bit word in `input_buffer` is 0, so skip over it.
        // Additionally, we're only interested in one half of the record
        // buffer, since the other half is being written to.
        for (i = 0; i < samples_per_loop; i++) {
            uint32_t unswapped = input_buffer[i * 2];
            int32_t swapped = (int32_t)((((unswapped >> 16) & 0xffff) |
                                         ((unswapped << 16) & 0xffff0000)));
            *output_buffer_ptr++ = swapped;

            // Advance the output buffer pointer, wrapping when it reaches the
            // end.
            if (output_buffer_ptr >= &data_buffer[DATA_BUFFER_ELEMENT_COUNT])
                output_buffer_ptr = &data_buffer[0];
        }
#elif defined(RECORD_TEST_32)
        // Used for sampling during development. Read out the buffer with:
        // ```
        // (gdb) dump binary memory tone.raw data_buffer
        // data_buffer+sizeof(data_buffer)
        // ```
        static int32_t *output_buffer_ptr = (int32_t *)&data_buffer[0];
        for (i = 0; i < samples_per_loop; i++) {
            uint32_t unswapped = input_buffer[i * 2];
            uint32_t neg_mask = (unswapped & (1 << 31)) ? 0xaa000055 : 0;
            int32_t swapped = (int32_t)((((unswapped >> 16) & 0xffff) |
                                         ((unswapped << 16) & 0xffff0000)) |
                                         neg_mask);
            *output_buffer_ptr++ = swapped;
            if (output_buffer_ptr >=
                (int32_t *)&data_buffer[DATA_BUFFER_ELEMENT_COUNT])
                output_buffer_ptr = (int32_t *)&data_buffer[0];
        }
#else
        // Every other 32-bit word in `input_buffer` is 0, so skip over it.
        // Additionally, we're only interested in one half of the record
        // buffer, since the other half is being written to.
        int32_t output_buffer[samples_per_loop];
        int32_t *output_buffer_ptr = output_buffer;
        for (i = 0; i < samples_per_loop; i++) {
            uint32_t unswapped = input_buffer[i * 2];
            int32_t swapped = (int32_t)((((unswapped >> 16) & 0xffff) |
                                         ((unswapped << 16) & 0xffff0000)));
            *output_buffer_ptr++ = swapped;
        }
        bpsk_run(output_buffer, sizeof(output_buffer) / sizeof(*output_buffer));
#endif

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
    bpsk_init();

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
    // uint32_t last_i2s_runs = 0;
    while (1) {
        background_tasks();
        usb_irqs++;
        if (i2s_irqs != last_i2s_irqs) {
            // printf("I2S IRQ count: %d  USB IRQ count: %d\n", i2s_irqs,
            // usb_irqs);
            last_i2s_irqs = i2s_irqs;
            usb_irqs = 0;
        }
        // if (((i2s_runs & 127) == 0) && (i2s_runs != last_i2s_runs)) {
        //     // printf("div: %d  tar: %5.0f  red: %f  ", agc_div, agc_target,
        //     //        agc_decrease);
        //     // tud_cdc_n_write_flush(0);
        //     // printf("inc: %f  off: %f\n", agc_increase, agc_offset);
        //     // last_i2s_runs = i2s_runs;
        //     // tud_cdc_n_write_flush(0);
        //     printf("div: %d  tar: %5.0f  dec: %f  "
        //            "inc: %f  off: %f  clip: %d  peak: %d  real: %d\n",
        //            _div, agc_target, agc_decrease, agc_increase,
        //            agc_offset, clipped_samples, current_peak, current_peak /
        //            agc_div);
        //     last_i2s_runs = i2s_runs;
        //     clipped_samples = 0;
        //     tud_cdc_n_write_flush(0);
        // }
    }

    NVIC_SystemReset();
}

// printf glue
void _putchar(char character) {
    int buffered = 1;
    if (character == '\n') {
        tud_cdc_n_write_char(0, '\r');
    }
    if (buffered) {
        while (tud_cdc_n_write_char(0, character) < 1) {
            tud_cdc_n_write_flush(0);
            background_tasks();
        }
    } else {
        tud_cdc_n_write_char(0, character);
    }
}
