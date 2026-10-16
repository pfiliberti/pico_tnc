/*
 * hf_demod.h - 300 baud HF packet demodulator (1600/1800 Hz AFSK)
 *
 * Several decoders run side by side, each tuned a little differently
 * (like Direwolf's "7@30", but 50 Hz apart), so a station that is off frequency still
 * decodes. A frame that passes the checksum in any decoder is kept,
 * copies of it from the other decoders are dropped.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define HF_MARK_HZ 1600
#define HF_SPACE_HZ 1800
#ifndef HF_DEC_N
#define HF_DEC_N 7              // number of decoders
#endif
#define HF_DEC_SPACING_HZ 50    // tuning step between decoders (7 x 50 Hz decodes stations up to about +-180 Hz off)
#define HF_CORR_N 44            // correlation window: one bit at 13200 Hz sampling
#define HF_HYST_SHIFT 5         // hysteresis = 1/32 of total tone energy
#define HF_DATA_LEN 1024        // longest frame
#define HF_MIN_LEN 17           // 2 addresses + control + FCS
#define HF_DUP_TICKS 300        // drop copies of a frame seen within 3 s (10 ms ticks)

typedef struct HF_DEC {
    // tone correlators
    uint32_t mark_ph, space_ph;
    uint32_t mark_step, space_step;
    int32_t corr[4][HF_CORR_N];     // mark i, mark q, space i, space q
    int32_t sum[4];
    uint8_t bit;

    // bit clock PLL (Direwolf style)
    int32_t pll_counter;
    int32_t pll_quality;
    uint8_t pval;
    uint8_t nrzi;

    // HDLC framing
    uint8_t state;
    uint8_t flag;
    uint8_t data_byte;
    int data_bit_cnt;
    int data_cnt;
    uint8_t data[HF_DATA_LEN];
} hf_dec_t;

typedef struct HF_DEMOD {
    hf_dec_t dec[HF_DEC_N];
    int idx;                        // position in the correlation window
    uint32_t pll_step;

    // duplicate filter
    uint16_t last_fcs;
    int last_len;
    uint32_t last_time;

    // statistics
    unsigned int good;              // frames passed on
    unsigned int dup;               // copies dropped
    unsigned int bad_long;          // center decoder: long frame, bad checksum
    unsigned int hits[HF_DEC_N];    // which decoder delivered each good frame
} hf_demod_t;

// sampling_rate in Hz, samples_per_bit = sampling_rate / baud
void hf_demod_init(hf_demod_t *h, int sampling_rate, int samples_per_bit);

// one band-pass filtered sample; now = time in 10 ms ticks (for the duplicate filter)
// returns true while any decoder is locked inside a frame (data carrier detect)
bool hf_demod_sample(hf_demod_t *h, int val, uint32_t now);

// drop any frame in progress (no signal, or transmitting)
void hf_demod_abort(hf_demod_t *h);

// called for every good, new frame (data includes the 2 FCS bytes)
void hf_frame_out(const uint8_t *data, int len);

// optional hook: called when the center decoder enters a frame (start != 0)
// and when a frame ends, good or bad (start == 0)
void hf_frame_event(bool start, bool good, int len);
