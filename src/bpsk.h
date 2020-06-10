#ifndef __AFSK_H__
#define __AFSK_H__

#include <stdint.h>

struct demod_config {
    uint32_t sample_rate;
    uint32_t carrier_tone;
    uint32_t baud_rate;
};

void bpsk_init(void);
void bpsk_run(int16_t *samples, size_t nsamples);

#endif /* __AFSK_H__ */
