/*
 * hf_demod.c - 300 baud HF packet demodulator, several decoders side by side
 *
 * Each decoder compares the energy of its mark and space tones over one
 * bit, recovers the bit clock with a PLL and unframes HDLC. Decoder i is
 * tuned (i - HF_DEC_N/2) * HF_DEC_SPACING_HZ away from 1600/1800 Hz.
 */

#include <string.h>
#include "hf_demod.h"

enum { HF_FLAG, HF_DATA };

// one cycle of sine, Q15
static const int16_t sin_tab[256] = {
         0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
      6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
     12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
     18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
     23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
     27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
     30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
     32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
     32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
     32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
     30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
     27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
     23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
     18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
     12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
      6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
         0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
     -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
    -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
    -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
    -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
     -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804
};

static uint32_t hz_step(int hz, int sampling_rate)
{
    return (uint32_t)(((uint64_t)hz << 32) / sampling_rate);
}

void hf_demod_init(hf_demod_t *h, int sampling_rate, int samples_per_bit)
{
    memset(h, 0, sizeof(*h));
    h->pll_step = (uint32_t)(((1ULL << 32) + samples_per_bit / 2) / samples_per_bit);
    for (int i = 0; i < HF_DEC_N; i++) {
        int offset = (i - HF_DEC_N / 2) * HF_DEC_SPACING_HZ;
        h->dec[i].mark_step = hz_step(HF_MARK_HZ + offset, sampling_rate);
        h->dec[i].space_step = hz_step(HF_SPACE_HZ + offset, sampling_rate);
    }
}

void hf_demod_abort(hf_demod_t *h)
{
    for (int i = 0; i < HF_DEC_N; i++) h->dec[i].state = HF_FLAG;
}

__attribute__((weak)) void hf_frame_event(bool start, bool good, int len)
{
    (void)start; (void)good; (void)len;
}

// AX.25 FCS over the whole frame including the FCS bytes leaves this residue
static bool fcs_ok(const uint8_t *p, int len)
{
    uint16_t crc = 0xffff;
    for (int i = 0; i < len; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc == 0xf0b8;
}

static void frame_end(hf_demod_t *h, int n, uint32_t now)
{
    hf_dec_t *d = &h->dec[n];
    int len = d->data_cnt;
    bool center = (n == HF_DEC_N / 2);

    if (len < HF_MIN_LEN) return;

    if (!fcs_ok(d->data, len)) {
        if (center && len >= 40) {
            h->bad_long++;
            hf_frame_event(false, false, len);
        }
        return;
    }

    // the same frame usually arrives from several decoders
    uint16_t fcs = d->data[len - 2] | (d->data[len - 1] << 8);
    if (fcs == h->last_fcs && len == h->last_len && now - h->last_time < HF_DUP_TICKS) {
        h->dup++;
        return;
    }
    h->last_fcs = fcs;
    h->last_len = len;
    h->last_time = now;

    h->good++;
    h->hits[n]++;
    hf_frame_event(false, true, len);
    hf_frame_out(d->data, len);
}

static void hdlc_bit(hf_demod_t *h, int n, int bit, uint32_t now)
{
    hf_dec_t *d = &h->dec[n];

    d->flag = (d->flag << 1) | bit;

    if (d->state == HF_FLAG) {
        if (d->flag == 0x7e) { // found flag
            d->state = HF_DATA;
            d->data_cnt = 0;
            d->data_bit_cnt = 0;
            if (n == HF_DEC_N / 2) hf_frame_event(true, false, 0);
        }
        return;
    }

    if ((d->flag & 0x3f) == 0x3f) { // six "1" bits: flag or abort, end of frame
        frame_end(h, n, now);
        d->state = HF_FLAG;
        return;
    }

    if ((d->flag & 0x3f) == 0x3e) return; // drop bit stuffing "0"

    d->data_byte = (d->data_byte >> 1) | (bit << 7);
    if (++d->data_bit_cnt >= 8) {
        if (d->data_cnt < HF_DATA_LEN) {
            d->data[d->data_cnt++] = d->data_byte;
        } else {
            d->state = HF_FLAG; // too long
            return;
        }
        d->data_bit_cnt = 0;
    }
}

static inline int32_t slide(hf_dec_t *d, int k, int idx, int32_t v)
{
    d->sum[k] += v - d->corr[k][idx];
    d->corr[k][idx] = v;
    return d->sum[k] >> 12;
}

bool hf_demod_sample(hf_demod_t *h, int val, uint32_t now)
{
    bool locked = false;
    int idx = h->idx;

    for (int n = 0; n < HF_DEC_N; n++) {
        hf_dec_t *d = &h->dec[n];

        // tone energy, mark vs space, over the last bit
        int mp = d->mark_ph >> 24;
        int sp = d->space_ph >> 24;
        d->mark_ph += d->mark_step;
        d->space_ph += d->space_step;

        int32_t mi = slide(d, 0, idx, (val * sin_tab[mp]) >> 8);
        int32_t mq = slide(d, 1, idx, (val * sin_tab[(mp + 64) & 255]) >> 8);
        int32_t si = slide(d, 2, idx, (val * sin_tab[sp]) >> 8);
        int32_t sq = slide(d, 3, idx, (val * sin_tab[(sp + 64) & 255]) >> 8);

        int32_t mark = mi * mi + mq * mq;
        int32_t space = si * si + sq * sq;
        int32_t diff = mark - space;
        int32_t hyst = (mark >> HF_HYST_SHIFT) + (space >> HF_HYST_SHIFT);

        if (diff > hyst) d->bit = 1;
        else if (diff < -hyst) d->bit = 0;

        // bit clock PLL, sample in the middle of each bit
        int32_t prev = d->pll_counter;
        d->pll_counter = (int32_t)((uint32_t)d->pll_counter + h->pll_step);
        if (prev >= 0 && d->pll_counter < 0) { // wrapped: bit time
            hdlc_bit(h, n, d->bit == d->nrzi, now); // NRZI
            d->nrzi = d->bit;
            if (d->pll_quality > 0) d->pll_quality--;
        }
        if (d->bit != d->pval) { // tone change: pull the clock toward it
            int32_t adjust = d->pll_counter >> 2;
            d->pll_counter -= adjust;
            if (adjust >= (1 << 28) && d->pll_quality < 100) d->pll_quality += 5; // lock error
            d->pval = d->bit;
        }

        if (d->pll_quality == 0 && d->state == HF_DATA) locked = true;
    }

    if (++idx >= HF_CORR_N) idx = 0;
    h->idx = idx;

    return locked;
}
