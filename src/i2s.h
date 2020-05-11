#ifndef __I2S_H__
#define __I2S_H__

#include <stdint.h>

struct i2s_pin_config {
  uint32_t data_pin_number;        // SDIN / SDOUT
  uint32_t bit_clock_pin_number;   // SCK
  uint32_t word_select_pin_number; // LRCK
};

int record_to_buffer(const struct i2s_pin_config *cfg,
                     uint8_t buffer_typecode, void *buffer, size_t length);

#endif /* __I2S_H__*/
