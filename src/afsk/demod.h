#ifndef __ORCHARD_DEMOD__
#define __ORCHARD_DEMOD__

#ifndef FSK_FILTER_MAX_SIZE
#define FSK_FILTER_MAX_SIZE 256
#endif

#ifndef FSK_FILTER_BUF_MAX
#define FSK_FILTER_BUF_MAX 256
#endif

// Scale our sin/cos tables so they fit in a signed 16-bit int
#define COS_BITS   14
#define COS_BASE   (1 << COS_BITS)

typedef int16_t demod_sample_t;

typedef struct {
    demod_sample_t filter_buf[FSK_FILTER_BUF_MAX];
    uint32_t buf_offset;

    /// The position inside of the current bit. Range (0,65536). A new bit is
    /// complete when this exceeds 65536.
    uint32_t baud_pll;

    /// The amount to increment the baud for each sample. This is dependent on
    /// the sample rate and baud rate.
    uint32_t baud_incr;

    /// How much to nudge the PLL by when we encounter a bit transition
    uint32_t baud_pll_adj;

    /// The last bit, 0 or 1
    int last_sample;

    int16_t shift;
    uint32_t run_length;
    int transition_count;
    int last_bit;
} FSK_demod_state;

typedef struct {
    /* parameters */
    uint32_t f_lo, f_hi;
    uint32_t sample_rate;
    uint32_t baud_rate;
    uint32_t filter_buf_size;

    /* local variables */
    uint32_t filter_size;
    int32_t filter_lo_i[FSK_FILTER_MAX_SIZE];
    int32_t filter_lo_q[FSK_FILTER_MAX_SIZE];

    int32_t filter_hi_i[FSK_FILTER_MAX_SIZE];
    int32_t filter_hi_q[FSK_FILTER_MAX_SIZE];
} FSK_demod_const;

/// Generate filter table constants.  This only needs to be done
/// once per set of parameters.
void fsk_demod_generate_table(FSK_demod_const *fsk_table, uint32_t baud_rate,
                              uint32_t sample_rate, uint32_t f_lo,
                              uint32_t f_hi, uint32_t gen_filter_tab);
void fsk_demod_init(const FSK_demod_const *fsk_table,
                    FSK_demod_state *fsk_state);
int fsk_demod(const FSK_demod_const *fsk_table, FSK_demod_state *fsk_state,
              int *bit, demod_sample_t *samples, size_t nb, size_t *processed_samples);

#endif
