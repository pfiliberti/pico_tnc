/*
Copyright (c) 2021, Kazuhisa Yokota, JN1DFF
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, 
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.
* Neither the name of the <organization> nor the names of its contributors 
  may be used to endorse or promote products derived from this software 
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"

//#include "timer.h"
#include "tnc.h"
#include "bell202.h"
#include "ax25.h"
#include "usb_output.h"
#include "tty.h"

#define FCS_OK 0x0f47
//#define FCS_OK (0x0f47 ^ 0xffff)
#define MIN_LEN (7 * 2 + 1 /*+ 1*/ + 2) // Address field *2, Control, (OPTIONAL!)PID, FCS

#define STR_LEN 64

static uint8_t str[STR_LEN];

static void display_packet(tty_t *ttyp, tnc_t *tp)
{
    int i;
    int in_addr = 1;
    int len = tp->data_cnt;
    uint8_t *data = tp->data;
    int size;

#if PORT_N > 1
    size = snprintf(str, STR_LEN, "(%d) %d:%d:", tnc_time(), tp->port, tp->pkt_cnt);
    tty_write(ttyp, str, size);
#endif

    for (i = 0; i < len - 2; i++) {
	    int c;

        if (i < 7) c = data[i + 7];       // src addr
        else if (i < 14) c = data[i - 7]; // dst addr
        else c = data[i];

	    if (in_addr) {
	        int d = c >> 1;

	        if (i % 7 == 6) { // SSID

	            if (i >= 7) in_addr = !(data[i] & 1);  // check address extension bit

		        if (d & 0x0f) { // SSID
                    size = snprintf(str, STR_LEN, "-%d", d & 0x0f);
                    tty_write(ttyp, str, size);
                }
                if (i >= 14 && (c & 0x80)) tty_write_char(ttyp, '*'); // H bit
		        if (i == 6) tty_write_char(ttyp, '>');
                else tty_write_char(ttyp, in_addr ? ',' : ':');

	        } else { // CALLSIGN

		        if (d >= '0' && d <= '9') tty_write_char(ttyp, d);
		        else if (d >= 'A' && d <= 'Z') tty_write_char(ttyp, d);
		        else if (d != ' ') {
                    size = snprintf(str, STR_LEN, "<%02x>", d);
                    tty_write(ttyp, str, size);
                }

	        }
	    } else {
	        if (c >= ' ' && c <= '~') tty_write_char(ttyp, c);
	        else {
                size = snprintf(str, STR_LEN, "<%02x>", c);
                tty_write(ttyp, str, size);
            }
	    }
    }
#if 1
    tty_write_str(ttyp, "\r\n");
#else
    size = snprintf(str, STR_LEN, "<%02x%02x>\r\n", data[len-1], data[len-2]);
    tty_write(ttyp, str, size);
#endif
}

static void output_packet(tnc_t *tp)
{
    int len = tp->data_cnt;
    uint8_t *data = tp->data;

    if (len < MIN_LEN) return;
 
    // FCS check
    if (ax25_fcs(0, data, len) != FCS_OK) return;
    //printf("\n*** Pllq=%d,%d\n",tp->pll_quality,tp->flag_count);
 
    // Here check if ax25 Input Queue has room and if so insert
    if(ax25_InQ_HasRoom()) ax25_InQ_Insert(data, len);
#if CODEX_RX_DIAGNOSTICS
    else {
        // Added by Codex
        extern struct {
        unsigned int rx_queue_inserted;
        unsigned int rx_queue_dropped_full;
        unsigned int rx_queue_started;
        unsigned int rx_queue_completed;
        unsigned int rx_queue_restarts;
        unsigned int rx_queue_max_depth;
        unsigned int rx_packet_max_len;
        unsigned int adc_dma_overruns;
        unsigned int tx_out_overflows;
        } codex_diag;
        codex_diag.rx_queue_dropped_full++;
    }
#endif

    for (int i = TTY_USB; i <= TTY_UART0; i++) {
        tty_t *ttyp = &tty[i];

        ttyp->montype = MON_OFF;
        // TNC MONitor command
        switch (ttyp->montype) {
            case MON_ALL:
                display_packet(ttyp, tp);
                break;
            
            default:
                break;
        }
    }
}

#define AX25_FLAG 0x7e

static void decode_bit(tnc_t *tp, int bit)
{
    tp->flag <<= 1;
    tp->flag |= bit;

    switch (tp->state)
    {
        case FLAG:
	    if (tp->flag == AX25_FLAG) // found flag
        {
	        tp->state = DATA;
	        tp->data_cnt = 0;
	        tp->data_bit_cnt = 0;
	        //fprintf(stderr, "found AX25_FALG\n");
	    }
	    break;

        case DATA:
        if ((tp->flag & 0x3f) == 0x3f) { // AX.25 flag, end of packet, six continuous "1" bits
	        output_packet(tp);
	        tp->state = FLAG;
	        break;
	    }

        // case DATA:
        // // AX.25 flag, end of packet, six continuous "1" bits followed by a 0!
	    // if (tp->flag == AX25_FLAG) 
        // {
	    //     output_packet(tp);
        //     /* there could be another frame coming in so continue receive */
        //     tp->state = FLAG;
	    //     tp->data_cnt = 0;
	    //     tp->data_bit_cnt = 0;
	    //     break;
	    // }

        /* Check for 5 consecutive 1s followed by a 0 (bit stuffing and drop stuffing bit if so )*/
        if ((tp->flag & 0x3f) == 0x3e) break; // ignore bit stuffing bit

        /* shift bit into byte msbit */
	    tp->data_byte >>= 1;
	    tp->data_byte |= (bit << 7);

	    tp->data_bit_cnt++;
        if (tp->data_bit_cnt >= 8)  /* got 8 bits for a byte? */
        {
	        if (tp->data_cnt < DATA_LEN) 
            {
                tp->data[tp->data_cnt++] = tp->data_byte;
            }
            else /* abort packet too long! */
            {
                printf("packet too long > %d\n", tp->data_cnt);
                tp->state = FLAG;
                break;
            }
	        tp->data_bit_cnt = 0; /* reset bit counter for next byte */
	    }
    }
}

static void decode(tnc_t *tp, int val)
{
    tp->edge++;
    if (val != tp->pval) {
      //	int bits = (edge + SAMPLING_N/2) / SAMPLING_N;
        int bits = (tp->edge * BAUD_RATE*2 + SAMPLING_RATE) / (SAMPLING_RATE * 2);

	    //printf("%d,", bits);

	    decode_bit(tp, 0);      // NRZI
	    while (--bits > 0) {
	        decode_bit(tp, 1);
	    }

	    tp->edge = 0;
	    tp->pval = val;
    }
}

#define PLL_STEP ((int)(((1ULL << 32) + SAMPLING_N/2) / SAMPLING_N))

// DireWolf PLL
static void decode2(tnc_t *tp, int val)
{
    int32_t prev_pll = tp->pll_counter >> 31; // compiler bug workaround

    tp->pll_counter += PLL_STEP;

    if ((tp->pll_counter >> 31) < prev_pll) { // overflow

        decode_bit(tp, val == tp->nrzi);    // decode NRZI
        tp->nrzi = val;

#ifdef OPEN_SQUELCH

        if(tp->pll_quality > 0)
        {
            tp->pll_quality--;
            if(tp->cdt)
            {
                gpio_put(tp->cdt_pin, 0);
                tp->cdt = false;
            }
        }

        if(tp->pll_quality == 0 && tp->state == DATA)
        {
            if(!tp->cdt)
            {
                gpio_put(tp->cdt_pin, 1);
                tp->cdt = true;
            }
        }      
#endif
    }

    if (val != tp->pval) {
        int32_t adjust = tp->pll_counter >> 2; // phase correction amount
        tp->pll_counter -= adjust; // adjust PLL counter

#ifdef OPEN_SQUELCH
        // update lock confidence
        if (adjust >= (1 << 28)) // lock error so penalze quality
        {
            if(tp->pll_quality < 100) tp->pll_quality+=5; // penalize lock error
        }
#endif
        tp->pval = val;
    }
}

void demodulator(tnc_t *tp, int adc)
{
    int val;
    int bit;

    //printf("%d,", adc);

    //dac_output_voltage(DAC_CHANNEL_1, adc >> 4);

#define AVERAGE_MUL 256 
#define AVERAGE_N 64
#define AVERAGE_SHIFT 6
#define CDT_AVG_N 128
#define CDT_MUL 256
#define CDT_SHIFT 6

    // update average value
    tp->avg += (adc * AVERAGE_MUL - tp->avg) >> AVERAGE_SHIFT;
    val = adc - (tp->avg + AVERAGE_MUL/2) / AVERAGE_MUL;

    // carrier detect
    tp->cdt_lvl += (val * val * CDT_MUL - tp->cdt_lvl) >> CDT_SHIFT;

#if 0
    static int count = 0;
    if ((++count & ((1 << 16) - 1)) == 0) {
        printf("(%u) decode: adc: %d, cdt_lvl: %d, avg: %d, port = %d\n", tnc_time(), adc, tp->cdt_lvl, tp->avg, tp->port);
    }
#endif

#define CDT_THR_LOW 1024
#define CDT_THR_HIGH (CDT_THR_LOW * 2) // low +6dB

#ifdef OPEN_SQUELCH
    // If no signal energy revert receiver state to FLAG and cdt=false
    if (tp->cdt && tp->cdt_lvl < CDT_THR_LOW)
    {
        tp->state = FLAG;
        tp->cdt = false;
        gpio_put(tp->cdt_pin, 0);
    }
#else
    if (!tp->cdt && tp->cdt_lvl > CDT_THR_HIGH) { // CDT on

        gpio_put(tp->cdt_pin, 1);
        tp->cdt = true;
    //     //printf("(%u) decode: CDT on, adc: %d, cdt_lvl: %d, avg: %d, port = %d\n", tnc_time(), adc, tp->cdt_lvl, tp->avg, tp->port);
    //     //printf("(%u) decode: cdt on, port = %d\n", tnc_time(), tp->port);

    } else if (tp->cdt && tp->cdt_lvl < CDT_THR_LOW) { // CDT off

        gpio_put(tp->cdt_pin, 0);
        tp->cdt = false;
    //     //printf("(%u) decode: CDT off, adc: %d, cdt_lvl: %d, avg: %d, port = %d\n", tnc_time(), adc, tp->cdt_lvl, tp->avg, tp->port);
    //     //printf("(%u) decode: cdt off, port = %d\n", tnc_time(), tp->port);

    }

    if (!tp->cdt) return;
#endif

#if 0
	sum += adc;
	if (++count >= AVERAGE_N) {
	    average = sum / AVERAGE_N;
	    //ESP_LOGI(TAG, "average adc value = %d",  average);
	    sum = 0;
	    count = 0;
	}
#else
	//tp->average = (tp->average * (AVERAGE_N - 1) + adc + AVERAGE_N/2) / AVERAGE_N;
	/*
	if (count >= 13200) {
		printf("average = %d\n", average);
		count = 0;
	}
	*/
#endif

#define LPF_N SAMPLING_N


	//val = bell202_decode((int)adc - average);
	//val = bell202_decode((int)adc - 2048);
	//val = bell202_decode(tp, adc); // delayed decode
#ifdef BELL202_SYNC
	val = bell202_decode2(tp, adc); // sync decode
#else
	val = bell202_decode(tp, adc); // delayed decode
#endif
	//lpf = (lpf * (LPF_N-1) + val) / LPF_N;

	//printf("demodulator(%d) = %d\n", adc, val);
#if 0
    static uint32_t count = 0;
    if (count < 132) {
        printf("%d, %d\n", adc, val);
    }
    if (++count >= 13200 * 10) {
        count = 0;
        printf("----------------\n");
    }
#endif

#define LPF_THRESHOLD (1 << 12)

	//printf("bit val = %d\n", val);

	if (val < -LPF_THRESHOLD) tp->bit = 1;
	else if (val >= LPF_THRESHOLD) tp->bit = 0;

	//dac_output_voltage(DAC_CHANNEL_2, bit * 128);
	//printf("%d\n", bit);
#ifdef DECODE_PLL
	decode2(tp, tp->bit);   // Direwolf PLL
#else
	decode(tp, tp->bit);    // bit length
#endif
}
