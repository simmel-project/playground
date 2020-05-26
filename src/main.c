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

#include "tusb.h"

static void cdc_task(void);

// tinyusb function that handles power event (detected, ready, removed)
// We must call it within SD's SOC event handler, or set it as power event
// handler if SD is not enabled.
void tusb_hal_nrf_power_event(uint32_t event);

__attribute__((used)) uint8_t id[3];
__attribute__((used)) float record_buffer_f[4096];
__attribute__((used)) int16_t record_buffer_i16[8192];
__attribute__((used)) int32_t record_buffer_i32[8192];

static const struct i2s_pin_config i2s_config = {
    .data_pin_number = (32 + 9),
    .bit_clock_pin_number = (0 + 12),
    .word_select_pin_number = (0 + 8),
};

static void usb_init(void) {
    // Priorities 0, 1, 4 (nRF52) are reserved for SoftDevice
    // 2 is highest for application
    NVIC_SetPriority(USBD_IRQn, 2);

    // USB power may already be ready at this time -> no event generated
    // We need to invoke the handler based on the status initially
    uint32_t usb_reg;

    // Power module init
    const nrfx_power_config_t pwr_cfg = {0};
    nrfx_power_init(&pwr_cfg);

    // Register tusb function as USB power handler
    const nrfx_power_usbevt_config_t config = {
        .handler = (nrfx_power_usb_event_handler_t)tusb_hal_nrf_power_event};
    nrfx_power_usbevt_init(&config);

    nrfx_power_usbevt_enable();

    usb_reg = NRF_POWER->USBREGSTATUS;

    if (usb_reg & POWER_USBREGSTATUS_VBUSDETECT_Msk)
        tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_DETECTED);
    if (usb_reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk)
        tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_READY);
}

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

    memset(record_buffer_f, 0, sizeof(record_buffer_f));
    memset(record_buffer_i16, 0, sizeof(record_buffer_i16));
    memset(record_buffer_i32, 0, sizeof(record_buffer_i32));
    record_to_buffer(&i2s_config, 'f', record_buffer_f,
                     sizeof(record_buffer_f) / sizeof(*record_buffer_f));
    record_to_buffer(&i2s_config, 'h', record_buffer_i16,
                     sizeof(record_buffer_i16) / sizeof(*record_buffer_i16));
    record_to_buffer(&i2s_config, 'i', record_buffer_i32,
                     sizeof(record_buffer_i32) / sizeof(*record_buffer_i32));

    usb_init();
    tusb_init();

    while (1) {
        tud_task(); // tinyusb device task
        cdc_task();
    }

    NVIC_SystemReset();
}

// echo to either Serial0 or Serial1
// with Serial0 as all lower case, Serial1 as all upper case
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        if (itf == 0) {
            // echo back 1st port as lower case
            // if (isupper(buf[i])) buf[i] += 'a' - 'A';
            buf[i] += 'a' - 'A';
        } else {
            // echo back additional ports as upper case
            buf[i] -= 'a' - 'A';
            // if (islower(buf[i])) buf[i] -= 'a' - 'A';
        }

        tud_cdc_n_write_char(itf, buf[i]);

        if (buf[i] == '\r') tud_cdc_n_write_char(itf, '\n');
    }
    tud_cdc_n_write_flush(itf);
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
static void cdc_task(void) {
    uint8_t itf;

    for (itf = 0; itf < CFG_TUD_CDC; itf++) {
        if (tud_cdc_n_connected(itf)) {
            if (tud_cdc_n_available(itf)) {
                uint8_t buf[64];

                uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

                // echo back to both serial ports
                echo_serial_port(0, buf, count);
                echo_serial_port(1, buf, count);
            }
        }
    }
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USBD_IRQHandler(void)
{
  tud_int_handler(0);
}
