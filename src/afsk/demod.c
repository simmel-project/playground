#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "demod.h"
#include "mac.h"
#include "../printf.h"

static int32_t FSK_core(demod_sample_t *b, const FSK_demod_const *table) {
    uint32_t j;
    int32_t corrs[4] = {0, 0, 0, 0};
    int32_t sum = 0;

    for (j = 0; j < table->filter_size; j++) {
        corrs[0] += b[j] * table->filter_hi_i[j];
        corrs[1] += b[j] * table->filter_hi_q[j];
        corrs[2] += b[j] * table->filter_lo_i[j];
        corrs[3] += b[j] * table->filter_lo_q[j];
    }

    corrs[0] >>= COS_BITS;
    corrs[1] >>= COS_BITS;
    corrs[2] >>= COS_BITS;
    corrs[3] >>= COS_BITS;

    // printf("\n");
    // printf("hi_i: %d\n", corrs[0]);
    // printf("hi_q: %d\n", corrs[1]);
    // printf("lo_i: %d\n", corrs[2]);
    // printf("lo_q: %d\n", corrs[3]);

    // This should use saturating operations!
    sum += (corrs[0] * corrs[0]);
    sum -= (corrs[2] * corrs[2]);
    sum += (corrs[1] * corrs[1]);
    sum -= (corrs[3] * corrs[3]);

    return sum;
}

#ifdef CAPTURE_BUFFER
int16_t saved_samples[32768];
uint32_t saved_samples_ptr;
#endif

int fsk_demod(const FSK_demod_const *table, FSK_demod_state *state, int *bit,
              demod_sample_t *samples, size_t nb, size_t *ps) {
    int new_sample;
    int32_t sum;
    demod_sample_t *b;
    size_t processed_samples = 0;

    while (nb-- > 0) {
        processed_samples++;

#ifdef CAPTURE_BUFFER
        /* add a new sample in the demodulation filter */
        saved_samples[saved_samples_ptr++] = *samples;
        if (saved_samples_ptr >= 32768) {
            saved_samples_ptr = 0;
        }
#endif

        state->filter_buf[state->buf_offset++] = *samples++ >> state->shift;

        // Duplicate the top half of the table into the bottom half,
        // and move the pointer back to the middle.
        // When the pointer reaches the end, move the pointer back
        // to the buffer start, which lies in the middle of the
        // filter_buf array:
        //
        // Buffer start ----+
        //                  v
        // +----------------+----------------+
        // | Copy           | Active buffer  |
        // +----------------+----------------+
        //          \______________/
        //                 ^
        //                 |
        //      Example averaging window
        //
        // The core will operate on a sliding window into this buffer
        // equal to half the size of the buffer.
        if (state->buf_offset == table->filter_buf_size) {
            memmove(state->filter_buf,
                    state->filter_buf + table->filter_buf_size -
                        table->filter_size,
                    table->filter_size * sizeof(demod_sample_t));
            state->buf_offset = table->filter_size;
        }

        // Define the start of the sliding window
        b = state->filter_buf + state->buf_offset - table->filter_size;

        // Perform the dot products on the window
        sum = FSK_core(b, table);

        // printf("sum=%0.3f\n", sum / 65536.0);
        // If the resulting sum is > 0, then it's a `1`.  Otherwise, it's a `0`.
        new_sample = sum > 0;

        // The `baud_pll` runs from 0..1.  It should transition halfway
        // through the phase.  Adjust the PLL by some small value in order to
        // track variations in the transmitter and receiver clocks.
        if (state->last_sample != new_sample) {
            state->last_sample = new_sample;
            int32_t nudge;
            if (state->baud_pll <= 32768)
                nudge = state->baud_pll_adj;
            else
                nudge = -(int32_t)state->baud_pll_adj;
            // printf("Bit transition detected!  pll: %3.1f%% -> %3.1f%% Sum:
            // %0.3f  Bit:"
            //        "%d  Run: %d\n",
            //        state->baud_pll * 100.0, (state->baud_pll + nudge) *
            //        100.0, sum, new_sample, state->run_length);
            state->baud_pll += nudge;
            // if (run_length > 200) {
            //     state->baud_pll = 0.5 - state->baud_pll_adj;
            // }
            state->transition_count++;
            state->run_length = 0;
        } else {
            state->run_length++;
        }

        state->baud_pll += state->baud_incr;

        // When the PLL exceeds 1, this bit time has finished and we need to
        // move on to the next bit.
        if (state->baud_pll >= 65536) {
            state->baud_pll -= 65536;
            // if (run_length < 32)
            // printf("Finished bit.  Transition count: %d  baud=%3.1f%%
            // (%d)\n",
            //        state->transition_count, state->baud_pll * 100.0,
            //        state->last_sample);
            // if (transition_count >= 2) {
            // //     printf("Transition count was %d, so keeping previous bit
            // of %d\n", transition_count, last_bit);
            //     state->last_sample = last_bit;
            // }
            *bit = state->last_sample;
            state->transition_count = 0;
            // state->last_bit = state->last_sample;
            *ps = processed_samples;
            return 1;
        }
    }
    *ps = processed_samples;
    return 0;
}

void fsk_demod_init(const FSK_demod_const *table, FSK_demod_state *state) {
    int32_t a;

    state->baud_incr = table->baud_rate * 65536 / table->sample_rate;
    state->baud_pll = 0;
    state->baud_pll_adj = state->baud_incr / 4;

    assert(table->filter_buf_size < FSK_FILTER_BUF_MAX);
    memset(state->filter_buf, 0,
           table->filter_buf_size * sizeof(demod_sample_t));
    state->buf_offset = table->filter_size;
    state->last_sample = 0;

    state->shift = -2;
    a = table->filter_size;
    while (a != 0) {
        state->shift++;
        a /= 2;
    }

    // state->run_length = 0;
    state->transition_count = 0;
    // state->last_bit = 0;
    // printf("shift=%d\n", state->shift);
}
