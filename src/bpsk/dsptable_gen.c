/* code to be run on the local computer to generate C files
   that contain constant tables for inclusion into the demodulator core */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>

#include "demod.h"

#ifdef M_PI
#undef M_PI
#endif
// Ensure M_PI is defined as a float, to avoid unnecessary conversion
#define M_PI 3.14159265358979323846f

void fsk_demod_generate_table(FSK_demod_const *fsk_table, uint32_t baud_rate,
                              uint32_t sample_rate, uint32_t f_lo,
                              uint32_t f_hi, uint32_t fsk_filter_size, uint8_t stuffing) {
    float phase;
    uint32_t i;

    assert(fsk_filter_size < FSK_FILTER_MAX_SIZE);
    fsk_table->f_lo = f_lo;
    fsk_table->f_hi = f_hi;
    fsk_table->baud_rate = baud_rate;
    fsk_table->sample_rate = sample_rate;
    fsk_table->filter_buf_size = fsk_filter_size * 2;
    fsk_table->filter_size = fsk_filter_size;
    fsk_table->stuffing = stuffing;

    /* compute the filters */
    for (i = 0; i < fsk_table->filter_size; i++) {
        phase = 2 * M_PI * fsk_table->f_lo * i / (float)fsk_table->sample_rate;
        fsk_table->filter_lo_i[i] = (int32_t)(cos(phase) * COS_BASE);
        fsk_table->filter_lo_q[i] = (int32_t)(sin(phase) * COS_BASE);

        phase = 2 * M_PI * fsk_table->f_hi * i / (float)fsk_table->sample_rate;
        fsk_table->filter_hi_i[i] = (int32_t)(cos(phase) * COS_BASE);
        fsk_table->filter_hi_q[i] = (int32_t)(sin(phase) * COS_BASE);
    }
}
