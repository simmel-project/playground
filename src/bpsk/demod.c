#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "demod.h"
#include "mac.h"

#ifdef PLAYGROUND
#include "../printf.h"
#else
#include <stdio.h>
#endif

#include "fir_coefficients.h"
#include "lpf_coefficients.h"

struct nco_state {
    float32_t samplerate; // Hz
    float32_t freq;       // Hz

    float32_t phase; // rad
};

struct bpsk_state {
    /// Convert incoming q15 values into f32 values into this
    /// buffer, so that we can keep track of it between calls.
    float32_t current[SAMPLES_PER_PERIOD];
    uint32_t current_offset;

    /// If there aren't enough samples to convert data from q15
    /// to f32, store them in here and return.
    demod_sample_t cached[SAMPLES_PER_PERIOD];
    uint32_t cached_capacity;

    arm_fir_instance_f32 fir;
    float32_t fir_state[SAMPLES_PER_PERIOD + FIR_STAGES - 1];

    struct nco_state nco;
    float32_t nco_error;

    arm_fir_instance_f32 i_lpf;
    float32_t i_lpf_state[SAMPLES_PER_PERIOD + LPF_STAGES - 1];

    arm_fir_instance_f32 q_lpf;
    float32_t q_lpf_state[SAMPLES_PER_PERIOD + LPF_STAGES - 1];

    float32_t agc;
    float32_t agc_step;
    float32_t agc_target_hi;
    float32_t agc_target_low;
};

static struct bpsk_state bpsk_state;

extern char varcode_to_char(uint32_t c);
static void print_char(uint32_t c) {
    printf("%c", varcode_to_char(c >> 2));
}

static void nco(float32_t control, uint32_t timestep, float32_t *i,
                float32_t *q) {
    // control is a number from +1 to -1, which translates to -pi to +pi
    // additional phase per time step timestep is the current time step,
    // expressed in terms of samples since t=0 i is a pointer to the in-phase
    // return value q is a pointer to the quadrature return value

    bpsk_state.nco.phase += (control / PI);

    *i = arm_cos_f32((((float32_t)timestep) * bpsk_state.nco.freq * 2 * PI) /
                         bpsk_state.nco.samplerate +
                     bpsk_state.nco.phase);
    *q = arm_sin_f32((((float32_t)timestep) * bpsk_state.nco.freq * 2 * PI) /
                         bpsk_state.nco.samplerate +
                     bpsk_state.nco.phase);
}

void bpsk_demod_init(void) {
    arm_fir_init_f32(&bpsk_state.fir, FIR_STAGES, fir_coefficients,
                     bpsk_state.fir_state, SAMPLES_PER_PERIOD);

    bpsk_state.nco.samplerate = SAMPLE_RATE;
    bpsk_state.nco.freq = CARRIER_TONE;
    bpsk_state.nco.phase = 0.0;
    bpsk_state.nco_error = 0.0;

    bpsk_state.agc = 1.0;
    bpsk_state.agc_step = 0.05;
    bpsk_state.agc_target_hi = 0.5;
    bpsk_state.agc_target_low = 0.25;

    // Force a buffer refill for the first iteration
    bpsk_state.current_offset = SAMPLES_PER_PERIOD;

    bpsk_state.cached_capacity = 0;

    arm_fir_init_f32(&bpsk_state.i_lpf, LPF_STAGES, lpf_coefficients,
                     bpsk_state.i_lpf_state, SAMPLES_PER_PERIOD);
    arm_fir_init_f32(&bpsk_state.q_lpf, LPF_STAGES, lpf_coefficients,
                     bpsk_state.q_lpf_state, SAMPLES_PER_PERIOD);
}

// Dump this with:
// ```
//  dump binary memory sample.wav &sample_wave
//  ((uint32_t)&sample_wave)+sizeof(sample_wave)
// ```
#ifdef CAPTURE_BUFFER
#define CAPTURE_BUFFER_COUNT (32768)
struct sample_wave {
    uint8_t header[44];
    uint16_t saved_samples[CAPTURE_BUFFER_COUNT];
};
struct sample_wave sample_wave;
uint32_t saved_samples_ptr;
#endif

int bpsk_demod(int *bit, demod_sample_t *samples, size_t nb,
               size_t *processed_samples) {

    while (1) {

        // If we've run out of samples, fill the sample buffer
        if (bpsk_state.current_offset >= SAMPLES_PER_PERIOD) {
            bpsk_state.current_offset = 0;

            // If there's data in the cache buffer, use that as the source
            // for data.
            if (bpsk_state.cached_capacity > 0) {
                // If there won't be enough data to process, so copy the
                // remainder to the cache and return.
                if (bpsk_state.cached_capacity + nb < SAMPLES_PER_PERIOD) {
                    memcpy(bpsk_state.cached + bpsk_state.cached_capacity,
                           samples, nb * sizeof(*(bpsk_state.cached)));
                    bpsk_state.cached_capacity += nb;
                    *processed_samples += nb;
                    return 0;
                }

                memcpy(bpsk_state.cached + bpsk_state.cached_capacity, samples,
                       (SAMPLES_PER_PERIOD - bpsk_state.cached_capacity) *
                           sizeof(*(bpsk_state.cached)));

                // There is enough data, so convert it to f32
                arm_q15_to_float(bpsk_state.cached, bpsk_state.current,
                                 SAMPLES_PER_PERIOD);
                nb -= (SAMPLES_PER_PERIOD - bpsk_state.cached_capacity);
                samples += SAMPLES_PER_PERIOD - bpsk_state.cached_capacity;
                *processed_samples += SAMPLES_PER_PERIOD;
                bpsk_state.cached_capacity = 0;
            }
            // Otherwise, the cache is empty, so operate directly on sample data
            else {
                // If there isn't enough data to operate on, store it in the
                // cache.
                if (nb < SAMPLES_PER_PERIOD) {
                    memcpy(bpsk_state.cached + bpsk_state.cached_capacity,
                           samples, nb * sizeof(*(bpsk_state.cached)));
                    bpsk_state.cached_capacity = nb;
                    *processed_samples += nb;
                    return 0;
                }

                arm_q15_to_float(samples, bpsk_state.current,
                                 SAMPLES_PER_PERIOD);
                nb -= SAMPLES_PER_PERIOD;
                samples += SAMPLES_PER_PERIOD;
                *processed_samples += SAMPLES_PER_PERIOD;
            }
        }

#ifdef CAPTURE_BUFFER
#error "Unimplemented"
#endif

        // Perform an initial FIR filter on the samples. This creates a LPF.
        // This line may be omitted for testing.
        arm_fir_f32(&bpsk_state.fir, bpsk_state.current, bpsk_state.current,
                    SAMPLES_PER_PERIOD);

        float32_t loopwindow[SAMPLES_PER_PERIOD];
        memcpy(loopwindow, bpsk_state.current, sizeof(bpsk_state.current));

        // scan for agc value. note q15_to_float normalizes an int16_t to
        // +1.0/-1.0.
        int above_hi = 0;
        int above_low = 0;
        for (int i = 0; i < SAMPLES_PER_PERIOD; i++) {
            loopwindow[i] = loopwindow[i] * bpsk_state.agc; // compute the agc

            // then check if we're out of bounds
            if (loopwindow[i] > bpsk_state.agc_target_low) {
                above_low = 1;
            }
            if (loopwindow[i] > bpsk_state.agc_target_hi) {
                above_hi = 1;
            }
        }
        if (above_hi) {
            bpsk_state.agc = bpsk_state.agc * (1.0 - bpsk_state.agc_step);
        } else if (!above_low) {
            bpsk_state.agc = bpsk_state.agc * (1.0 + bpsk_state.agc_step);
        }

        float32_t i_samps[SAMPLES_PER_PERIOD];
        float32_t q_samps[SAMPLES_PER_PERIOD];
        for (int i = 0; i < SAMPLES_PER_PERIOD; i++) {
            nco(bpsk_state.nco_error, (uint32_t)i, &(i_samps[i]),
                &(q_samps[i]));
        }

        static float32_t i_mult_samps[SAMPLES_PER_PERIOD];
        static float32_t q_mult_samps[SAMPLES_PER_PERIOD];
        arm_mult_f32(loopwindow, i_samps, i_mult_samps, SAMPLES_PER_PERIOD);
        arm_mult_f32(loopwindow, q_samps, q_mult_samps, SAMPLES_PER_PERIOD);

        static float32_t i_lpf_samples[SAMPLES_PER_PERIOD];
        static float32_t q_lpf_samples[SAMPLES_PER_PERIOD];
        arm_fir_f32(&bpsk_state.i_lpf, i_mult_samps, i_lpf_samples, SAMPLES_PER_PERIOD);
        arm_fir_f32(&bpsk_state.q_lpf, q_mult_samps, q_lpf_samples, SAMPLES_PER_PERIOD);

        // arm_float_to_q15(i_lpf_samples, &(i_loop[sample_offset]),
        //                  SAMPLES_PER_PERIOD);
        // arm_float_to_q15(q_lpf_samples, &(q_loop[sample_offset]),
        //                  SAMPLES_PER_PERIOD);

        float32_t errorwindow[SAMPLES_PER_PERIOD];
        arm_mult_f32(i_lpf_samples, q_lpf_samples, errorwindow,
                     SAMPLES_PER_PERIOD);
        float32_t avg = 0;
        for (int i = 0; i < SAMPLES_PER_PERIOD; i++) {
            avg += errorwindow[i];
        }
        avg /= ((float32_t)SAMPLES_PER_PERIOD);
        bpsk_state.nco_error = -(avg);
        // printf("err: %0.04f\n", nco_error);

        static float bit_pll = 0;
        static const float pll_incr = (BAUD_RATE / (float)SAMPLE_RATE);
        for (unsigned int j = 0; j < SAMPLES_PER_PERIOD; j++) {
            if (bit_pll < 0.5 && (bit_pll + pll_incr) >= 0.5) {
                static int bit_acc = 0;
                static int last_state = 0;
                int state = i_lpf_samples[j] > 0.0;
                *bit = !(state ^ last_state);
                last_state = state;

                bit_acc = (bit_acc << 1) | *bit;
                if ((bit_acc & 3) == 0) {
                    print_char(bit_acc);
                    bit_acc = 0;
                }
            }
            bit_pll += pll_incr;
            if (bit_pll >= 1) {
                bit_pll -= 1;
            }
        }
    }
}
