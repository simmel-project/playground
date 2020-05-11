#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "nrfx.h"
#include "nrfx_i2s.h"

#include "i2s.h"

struct {
  int16_t l, r;
} static_sample16 = {0x8000, 0x8000};
struct {
  uint8_t l1, r1, l2, r2;
} static_sample8 = {0x80, 0x80, 0x80, 0x80};

struct frequency_info {
  uint32_t RATIO;
  uint32_t MCKFREQ;
  int sample_rate;
  float abserr;
};
struct ratio_info {
  uint32_t RATIO;
  int16_t divisor;
  bool can_16bit;
  bool can_24bit;
};
struct ratio_info ratios[] = {
    {I2S_CONFIG_RATIO_RATIO_32X, 32, true, true},
    {I2S_CONFIG_RATIO_RATIO_48X, 48, false, false},
    {I2S_CONFIG_RATIO_RATIO_64X, 64, true, true},
    {I2S_CONFIG_RATIO_RATIO_96X, 96, true, false},
    {I2S_CONFIG_RATIO_RATIO_128X, 128, true, true},
    {I2S_CONFIG_RATIO_RATIO_192X, 192, true, true},
    {I2S_CONFIG_RATIO_RATIO_256X, 256, true, true},
    {I2S_CONFIG_RATIO_RATIO_384X, 384, true, true},
    {I2S_CONFIG_RATIO_RATIO_512X, 512, true, true},
};

struct mclk_info {
  uint32_t MCKFREQ;
  int divisor;
};
struct mclk_info mclks[] = {
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8, 8},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV10, 10},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11, 11},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV15, 15},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV16, 16},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21, 21},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV23, 23},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV31, 31},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV42, 42},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63, 63},
    {I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV125, 125},
};

static void construct(const struct i2s_pin_config *cfg) {
  NRF_I2S->PSEL.MCK = cfg->bit_clock_pin_number;
  NRF_I2S->PSEL.LRCK = cfg->word_select_pin_number;
  NRF_I2S->PSEL.SDOUT = cfg->data_pin_number;
  NRF_I2S->PSEL.SCK = (0+11);

  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Master;
  NRF_I2S->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Enabled;
  NRF_I2S->CONFIG.TXEN = I2S_CONFIG_TXEN_TXEN_Disabled;
  NRF_I2S->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Enabled;
  NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_64X;
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8;
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Left;

  NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_Left;
  NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S;
}

#define REC_BUFFER_LEN 16
int record_to_buffer(const struct i2s_pin_config *cfg,
                     uint8_t buf_typecode, void *buffer, size_t length) {
  float *buffer_f = (float *)buffer;
  uint32_t *buffer_u32 = (uint32_t *)buffer;
  int32_t *buffer_i32 = (int32_t *)buffer;

  construct(cfg);
  
  uint32_t stack_buffer[REC_BUFFER_LEN];
  uint32_t *sb_lo = stack_buffer;
  uint32_t *sb_hi = &stack_buffer[REC_BUFFER_LEN / 2];
  uint32_t sb_idx = 0;

  NRF_I2S->PSEL.SDOUT = 0xFFFFFFFF;
  NRF_I2S->PSEL.SDIN = cfg->data_pin_number;

  NRF_I2S->RXD.PTR = (uintptr_t)stack_buffer;
  NRF_I2S->TXD.PTR = 0xFFFFFFFF;
  // Turn on the interrupt to the NVIC but not within the NVIC itself. This
  // will wake the CPU and keep it awake until it is serviced without
  // triggering an interrupt handler.
  NRF_I2S->INTENSET = I2S_INTENSET_RXPTRUPD_Msk;
  NRF_I2S->ENABLE = I2S_ENABLE_ENABLE_Enabled;

  NRF_I2S->TASKS_START = 1;

  // The first event fires indicating the hardware accepted the buffer
  // and that it's started writing to it.
  // Wait for the hardware to read in the buffer
  do {
    uint32_t i;
    uint32_t sample_size = length;
    if (sample_size > REC_BUFFER_LEN / 2) {
      sample_size = REC_BUFFER_LEN / 2;
    }
    NRF_I2S->RXTXD.MAXCNT = sample_size;

    while (!NRF_I2S->EVENTS_RXPTRUPD) {
    //   asm("wfe");
    }
    NRF_I2S->EVENTS_RXPTRUPD = 0;
    uint32_t *current_sb = sb_idx & 1 ? sb_hi : sb_lo;
    NRF_I2S->RXD.PTR = (uintptr_t) (((++sb_idx) & 1) ? sb_hi : sb_lo);

    switch (buf_typecode) {
    case 'f':
      for (i = 0; i < sample_size; i++) {
        *buffer_f++ = current_sb[i];
      }
      break;
    case 'I':
    case 'L':
      for (i = 0; i < sample_size; i++) {
        *buffer_u32++ = current_sb[i] + 16777216;
      }
      break;
    case 'i':
    case 'l':
      for (i = 0; i < sample_size; i++) {
        *buffer_i32++ = ((int32_t *)current_sb)[i];
      }
      break;
    default:
      asm("bkpt");
    }

    length -= sample_size;
  } while (length != 0);
  //   NRF_I2S->RXTXD.MAXCNT = 0;

  // The second event fires indicating the buffer has filled
  // and that the I2S core has latched a second buffer.
  // Note that there is a race condition here, because the
  // I2S engine will begin writing over the buffer a second time.
  // Audio data is slow enough that this shouldn't matter, but there
  // is a potential for the first few samples to get overwritten
  // if the VM Hook Loop is very slow.
  while (!NRF_I2S->EVENTS_RXPTRUPD) {
    // asm("wfe");
  }

  // Stop the task as soon as possible to prevent it from overwriting the
  // buffer.
  NRF_I2S->TASKS_STOP = 1;
  NRF_I2S->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Disabled;
  NRF_I2S->EVENTS_RXPTRUPD = 0;
  NRF_I2S->RXTXD.MAXCNT = 0;

  NRF_I2S->CONFIG.TXEN = I2S_CONFIG_TXEN_TXEN_Enabled;
  NRF_I2S->PSEL.SDIN = 0xFFFFFFFF;
  NRF_I2S->PSEL.SDOUT = cfg->data_pin_number;

  NRF_I2S->INTENCLR = I2S_INTENSET_RXPTRUPD_Msk;
  NRF_I2S->ENABLE = I2S_ENABLE_ENABLE_Disabled;
  return length;
}
