#ifndef __I2S_H__
#define __I2S_H__

#include <stdint.h>

struct i2s_pin_config {
  uint32_t data_pin_number;        // SDIN / SDOUT
  uint32_t bit_clock_pin_number;   // MCK
  uint32_t word_select_pin_number; // LRCK
};

void nus_init(const struct i2s_pin_config *cfg, void *buffer, size_t length);
void nus_start(void);
void nus_stop(void);

struct modulate_cfg {
  uint32_t rate;
  uint32_t carrier;
  float baud;
  float pll_incr;
  void (*write)(void *arg, void *data, unsigned int count);
  void *write_arg;
  float omega;
  const char *string;
};

struct modulate_state {
  struct modulate_cfg cfg;
  float bit_pll;
  float baud_pll;
  int polarity;
  int str_pos;
  int varcode_pos;
  char varcode_str[16];
  int bitcount;
  int16_t high;
  int16_t low;
  int modulating;
};


#endif /* __I2S_H__*/
