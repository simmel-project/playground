#ifndef __AFSK_H__
#define __AFSK_H__

#include <stdint.h>

struct demod_config {
    uint32_t sample_rate;
    uint32_t f_lo;
    uint32_t f_hi;
    uint32_t filter_width;
    uint32_t baud_rate;
};

void afsk_init(const struct demod_config *cfg);
void afsk_run(int16_t *samples, size_t nsamples);

#endif /* __AFSK_H__ */
