#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "arm_math.h"

#include "demod.h"
#include "mac.h"

#ifdef PLAYGROUND
#include "../printf.h"
#else
#include <stdio.h>
#endif

#include "fir_coefficients.h"
#include "lpf_coefficients.h"

#define CAPTURE_BUFFER
#define LOG_QUADRATURE
// #define LOG_TRANSITION_PLLS

struct nco_state {
    float32_t samplerate; // Hz
    float32_t freq;       // Hz

    float32_t phase; // rad
    float32_t offset;
    float32_t error;
};

// Dump this with:
// ```
//  dump binary memory sample.wav &sample_wave
//  ((uint32_t)&sample_wave)+sizeof(sample_wave)
// ```
#ifdef CAPTURE_BUFFER
#define CAPTURE_BUFFER_COUNT (20000)
struct sample_wave {
    uint8_t header[44];
    demod_sample_t saved_samples[CAPTURE_BUFFER_COUNT];
};
struct sample_wave sample_wave;
uint32_t saved_samples_ptr;
#endif

// Log what the PLL is when a bit transition is encountered
#ifdef LOG_TRANSITION_PLLS
__attribute__((used)) float32_t transition_plls[64];
uint32_t transition_pll_offset;
#endif

struct bpsk_state {
    /// Convert incoming q15 values into f32 values into this
    /// buffer, so that we can keep track of it between calls.
    float32_t current[SAMPLES_PER_PERIOD];
    uint32_t current_offset;

    /// If there aren't enough samples to convert data from q15
    /// to f32, store them in here and return.
    demod_sample_t cache[SAMPLES_PER_PERIOD];
    uint32_t cache_capacity;

    arm_fir_instance_f32 fir;
    float32_t fir_state[SAMPLES_PER_PERIOD + FIR_STAGES - 1];

    struct nco_state nco;

    arm_fir_instance_f32 i_lpf;
    float32_t i_lpf_state[SAMPLES_PER_PERIOD + LPF_STAGES - 1];

    arm_fir_instance_f32 q_lpf;
    float32_t q_lpf_state[SAMPLES_PER_PERIOD + LPF_STAGES - 1];

    float32_t i_lpf_samples[SAMPLES_PER_PERIOD];

    float32_t agc;
    float32_t agc_step;
    float32_t agc_target_hi;
    float32_t agc_target_low;

    // Signal decoding
    float32_t bit_pll;
    float32_t pll_incr;
    float32_t pll_incr_nudge;
    int bit_acc;
    int last_state;
    int last_bit_state;
};

static struct bpsk_state bpsk_state;

static void make_nco(float32_t *i, float32_t *q) {
    // control is a number from +1 to -1, which translates to -pi to +pi
    // additional phase per time step timestep is the current time step,
    // expressed in terms of samples since t=0 i is a pointer to the in-phase
    // return value q is a pointer to the quadrature return value

    float32_t timestep = bpsk_state.nco.offset;
    while (timestep < SAMPLES_PER_PERIOD + bpsk_state.nco.offset) {
        bpsk_state.nco.phase += (bpsk_state.nco.error / PI);
        *i++ = arm_cos_f32((timestep * bpsk_state.nco.freq * 2 * PI) /
                               bpsk_state.nco.samplerate +
                           bpsk_state.nco.phase);
        *q++ = arm_sin_f32((timestep * bpsk_state.nco.freq * 2 * PI) /
                               bpsk_state.nco.samplerate +
                           bpsk_state.nco.phase);
        timestep += 1;
    }

    // XXX MAKE SURE TO DEAL WITH TIMESTEP WRPAPING
    if (timestep > bpsk_state.nco.samplerate)
        timestep -= bpsk_state.nco.samplerate;

    bpsk_state.nco.offset = timestep;
}

void bpsk_demod_init(void) {
    arm_fir_init_f32(&bpsk_state.fir, FIR_STAGES, fir_coefficients,
                     bpsk_state.fir_state, SAMPLES_PER_PERIOD);

    bpsk_state.nco.samplerate = SAMPLE_RATE;
    bpsk_state.nco.freq = CARRIER_TONE;
    bpsk_state.nco.phase = 0.0;
    bpsk_state.nco.error = 0.0;
    bpsk_state.nco.offset = 0;

    bpsk_state.agc = 1.0;
    bpsk_state.agc_step = 0.001;
    bpsk_state.agc_target_hi = 0.5;
    bpsk_state.agc_target_low = 0.25;

    // Force a buffer refill for the first iteration
    bpsk_state.current_offset = SAMPLES_PER_PERIOD;

    bpsk_state.cache_capacity = 0;

    arm_fir_init_f32(&bpsk_state.i_lpf, LPF_STAGES, lpf_coefficients,
                     bpsk_state.i_lpf_state, SAMPLES_PER_PERIOD);
    arm_fir_init_f32(&bpsk_state.q_lpf, LPF_STAGES, lpf_coefficients,
                     bpsk_state.q_lpf_state, SAMPLES_PER_PERIOD);

    bpsk_state.bit_pll = 0.0;
    bpsk_state.pll_incr = ((float)BAUD_RATE / (float)SAMPLE_RATE);
    bpsk_state.pll_incr_nudge = bpsk_state.pll_incr / 8.0;
    bpsk_state.bit_acc = 0;
    bpsk_state.last_state = 0;

#ifdef CAPTURE_BUFFER
    const uint8_t wav_header[] = {
        0x52, 0x49, 0x46, 0x46, // chunkId 'RIFF'
        0x1c, 0x12, 0x05, 0x00, // Chunk size
        0x57, 0x41, 0x56, 0x45, // 'WAVE'
        0x66, 0x6d, 0x74, 0x20, // 'fmt '
        0x10, 0x00, 0x00, 0x00, // Sub chunk 1 size (chunk is 16 bytes)
        0x01, 0x00,             // Audio format (1 = pcm)
        0x02, 0x00,             // Numer of channels (1 = mono)
        0x11, 0x2b, 0x00, 0x00, // Sample rate
        0x22, 0x56, 0x00, 0x00, // Byte rate
        0x02, 0x00,             // Block alignment
        16,   0x00,             // Bits per sample
        0x64, 0x61, 0x74, 0x61, // 'data'
        0xf8, 0x11, 0x05, 0x00, // chunk size
    };
    uint32_t rate = 62500;
    uint32_t len = sizeof(sample_wave.saved_samples);
    memcpy(sample_wave.header, wav_header, sizeof(wav_header));
    memcpy(&sample_wave.header[24], &rate, sizeof(rate));
    memcpy(&sample_wave.header[40], &len, sizeof(len));
#endif
}

#ifdef CAPTURE_BUFFER
#ifndef LOG_QUADRATURE
static void append_to_capture_buffer(demod_sample_t *samples) {
    uint32_t sample_count = 0;
    while (sample_count++ < SAMPLES_PER_PERIOD) {
        sample_wave.saved_samples[saved_samples_ptr++] = *samples++;
        if (saved_samples_ptr > CAPTURE_BUFFER_COUNT) {
            saved_samples_ptr = 0;
        }
    }
}
#endif
static void append_to_capture_buffer_stereo(int16_t *left, int16_t *right) {
    uint32_t sample_count = 0;
    while (sample_count++ < SAMPLES_PER_PERIOD) {
        sample_wave.saved_samples[saved_samples_ptr++] = *left++ | ((*right++) << 16);
        if (saved_samples_ptr > CAPTURE_BUFFER_COUNT) {
            saved_samples_ptr = 0;
        }
    }
}
#endif

static void bpsk_core(void) {
    // Perform an initial FIR filter on the samples. This creates a LPF.
    // This line may be omitted for testing.
    arm_fir_f32(&bpsk_state.fir, bpsk_state.current, bpsk_state.current,
                SAMPLES_PER_PERIOD);

    // Scan for agc value. note values are normalized to +1.0/-1.0.
    int above_hi = 0;
    int above_low = 0;
    for (int i = 0; i < SAMPLES_PER_PERIOD; i++) {
        bpsk_state.current[i] =
            bpsk_state.current[i] * bpsk_state.agc; // compute the agc

        // then check if we're out of bounds
        if (bpsk_state.current[i] > bpsk_state.agc_target_low) {
            above_low = 1;
        }
        if (bpsk_state.current[i] > bpsk_state.agc_target_hi) {
            above_hi = 1;
        }
    }
    if (above_hi) {
        bpsk_state.agc = bpsk_state.agc * (1.0 - bpsk_state.agc_step);
    } else if (!above_low) {
        bpsk_state.agc = bpsk_state.agc * (1.0 + bpsk_state.agc_step);
    }

    // Generate Numerically-Controlled Oscillator values for the
    // current timestamp.
    float32_t i_samps[SAMPLES_PER_PERIOD];
    float32_t q_samps[SAMPLES_PER_PERIOD];
    make_nco(i_samps, q_samps);

    static float32_t i_mult_samps[SAMPLES_PER_PERIOD];
    static float32_t q_mult_samps[SAMPLES_PER_PERIOD];
    arm_mult_f32(bpsk_state.current, i_samps, i_mult_samps, SAMPLES_PER_PERIOD);
    arm_mult_f32(bpsk_state.current, q_samps, q_mult_samps, SAMPLES_PER_PERIOD);

    // static float32_t bpsk_state.i_lpf_samples[SAMPLES_PER_PERIOD];
    static float32_t q_lpf_samples[SAMPLES_PER_PERIOD];
    arm_fir_f32(&bpsk_state.i_lpf, i_mult_samps, bpsk_state.i_lpf_samples,
                SAMPLES_PER_PERIOD);
    arm_fir_f32(&bpsk_state.q_lpf, q_mult_samps, q_lpf_samples,
                SAMPLES_PER_PERIOD);

#ifdef LOG_QUADRATURE
    int16_t i_loop[SAMPLES_PER_PERIOD];
    int16_t q_loop[SAMPLES_PER_PERIOD];
    arm_float_to_q15(bpsk_state.i_lpf_samples, i_loop, SAMPLES_PER_PERIOD);
    arm_float_to_q15(q_lpf_samples, q_loop, SAMPLES_PER_PERIOD);
    append_to_capture_buffer_stereo(i_loop, q_loop);
    //write_wav_stereo(i_loop, q_loop, SAMPLES_PER_PERIOD,
    //"quadrature_loop.wav");
#endif

    float32_t errorwindow[SAMPLES_PER_PERIOD];
    arm_mult_f32(bpsk_state.i_lpf_samples, q_lpf_samples, errorwindow,
                 SAMPLES_PER_PERIOD);
    float32_t avg = 0;
    for (int i = 0; i < SAMPLES_PER_PERIOD; i++) {
        avg += errorwindow[i];
    }
    avg /= ((float32_t)SAMPLES_PER_PERIOD);
    bpsk_state.nco.error = -(avg)/(0.25);
    // printf("err: %0.04f\n", bpsk_state.nco.error);
}

/// Attempt to fill the `bpsk_state.current` buffer with `SAMPLES_PER_PERIOD`
/// samples of data. If there is data in the `bpsk_state.cache` buffer, pull
/// samples from there. Otherwise, pull samples from the `samples` array.
/// Returns -1 if there isn't enough data to process, or >=0 indicating the
/// number of samples removed from `samples`.
static int bpsk_fill_buffer(demod_sample_t *samples, uint32_t nb,
                            uint32_t *processed_samples) {
    // If there aren't any bytes remaining to process, return.
    if (nb == 0) {
        return -1;
    }
    // If there's data in the cache buffer, use that as the source
    // for data.
    else if (bpsk_state.cache_capacity > 0) {
        // If there won't be enough data to process, copy the
        // remainder to the cache and return.
        if (bpsk_state.cache_capacity + nb < SAMPLES_PER_PERIOD) {
            printf("Buffer not big enough, stashing in cache\n");
            memcpy(&bpsk_state.cache[bpsk_state.cache_capacity], samples,
                   nb * sizeof(demod_sample_t));
            bpsk_state.cache_capacity += nb;
            *processed_samples += nb;
            return -1;
        }

        // There is enough data if we combine the cache with fresh data,
        // so determine how many samples to take.
        uint32_t samples_to_take =
            (SAMPLES_PER_PERIOD - bpsk_state.cache_capacity);

        // printf("Pulling %d samples from cache and adding %d samples
        // from pool of %d\n",
        //     bpsk_state.cache_capacity,
        //     samples_to_take, nb);
        memcpy(&bpsk_state.cache[bpsk_state.cache_capacity], samples,
               samples_to_take * sizeof(demod_sample_t));

#ifdef CAPTURE_BUFFER
	//        append_to_capture_buffer(bpsk_state.cache);
#endif

        // There is enough data, so convert it to f32 and clear the
        // cache
        arm_q31_to_float(bpsk_state.cache, bpsk_state.current,
                         SAMPLES_PER_PERIOD);
        bpsk_state.cache_capacity = 0;

        nb -= samples_to_take;
        samples += samples_to_take;
        (*processed_samples) += samples_to_take;
        return samples_to_take;
    }
    // Otherwise, the cache is empty, so operate directly on sample data
    else {
        // If there isn't enough data to operate on, store it in the
        // cache.
        if (nb < SAMPLES_PER_PERIOD) {
            // printf("Only %d samples left, stashing in cache\n", nb);
            memcpy(bpsk_state.cache, samples, nb * sizeof(demod_sample_t));
            bpsk_state.cache_capacity = nb;
            (*processed_samples) += nb;
            return -1;
        }

#ifdef CAPTURE_BUFFER
	//        append_to_capture_buffer(samples);
#endif
        // Directly convert the sample data to f32
        arm_q31_to_float(samples, bpsk_state.current, SAMPLES_PER_PERIOD);
        nb -= SAMPLES_PER_PERIOD;
        samples += SAMPLES_PER_PERIOD;
        (*processed_samples) += SAMPLES_PER_PERIOD;
    }

    // Indicate we successfully filled the buffer
    return SAMPLES_PER_PERIOD;
}

#define HYSTERESIS (32.0/32768.0)

int bpsk_demod(uint32_t *bit, demod_sample_t *samples, uint32_t nb,
               uint32_t *processed_samples) {
    *processed_samples = 0;
    while (1) {

        // Advance to the next demodualted sample
        bpsk_state.current_offset += 1;

        // If we've run out of demodulated signal data, fill the sample buffer
        if (bpsk_state.current_offset >= SAMPLES_PER_PERIOD) {

            // Remove samples from the internal cache and/or the samples
            // provided to us in the `samples` array. If there is not enough
            // data to continue, this function will store all remaining data in
            // a cache and return `-1`.
            int samples_removed =
                bpsk_fill_buffer(samples, nb, processed_samples);
            if (samples_removed == -1) {
                return 0;
            }
            nb -= samples_removed;
            samples += samples_removed;

            // Process the contents of the bpsk_state. This reads in the samples
            // and generates output data, primarily the `i_lpf_samples` array
            // which contains the demodulated signal data.
            bpsk_core();
            bpsk_state.current_offset = 0;
        }

	int state;
	if( bpsk_state.last_state == 0 ) {
	  if( bpsk_state.i_lpf_samples[bpsk_state.current_offset] > HYSTERESIS )
	    state = 1;
	  else
	    state = 0;
	} else {
	  if( bpsk_state.i_lpf_samples[bpsk_state.current_offset] < -HYSTERESIS )
	    state = 0;
	  else
	    state = 1;
	}

        // If the state has transitioned, nudge the PLL towards the middle of
        // the bit in order to keep locked on.
        if (state != bpsk_state.last_state) {
#ifdef LOG_TRANSITION_PLLS
            transition_plls[transition_pll_offset++] = bpsk_state.bit_pll;
            if (transition_pll_offset >= 64) transition_pll_offset = 0;
#endif
            // Perform the nudge depending on whether it's in the first
            // half of the pulse or the second.
            if (bpsk_state.bit_pll < 0.5) {
                bpsk_state.bit_pll += bpsk_state.pll_incr_nudge;
            } else {
                bpsk_state.bit_pll -= bpsk_state.pll_incr_nudge;
            }
            bpsk_state.last_state = state;
        }

        // Advance the PLL. When it hits 1.0, it means we've read an entire bit.
        // Return `1` indicating that `*bit` is valid, so the caller can handle
        // it.
        bpsk_state.bit_pll += bpsk_state.pll_incr;
        if (bpsk_state.bit_pll >= 1) {
            bpsk_state.bit_pll -= 1;

            // A transition (e.g. from high-to-low or low-to-high) is
            // a logical `0`, whereas no transition is a `1`. Calculate
            // this, and stash the current state for the next loop.
            *bit = !(state ^ bpsk_state.last_bit_state);
            bpsk_state.last_bit_state = state;
            return 1;
        }
    }
}
