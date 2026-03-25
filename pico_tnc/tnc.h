/**
 * Copyright (c) 2021 JN1DFF
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once
#include "pico/util/queue.h"

#include "filter.h"
#include "ax25.h"
//#include "cmd.h"

#include "sio.h"

// Added to support opentnc hardware layout (mpvano)
// NOTE make sure this file (tnc.h) is included in send.c and receive.c
//	send.c and receive.c will use presence of the following definitions only
//	that way a simple command line -D OPEN_TNC_GPIO wil enable them in all files
//#define OPEN_TNC_GPIO	1
#ifdef OPEN_TNC_GPIO
//	GPIO Number definitions for OpenTNC hardware
#define OPEN_TNC_PWM		20	// physical pin 26
#define OPEN_TNC_PTT_OUT	16	// physical pin 21

//	OpenTNC Leds are (Left to Right): RUN, CONNECT, CARRIER, PACKET, PTT
#define OPEN_TNC_CARRIER	11	// physical pin 15
#define OPEN_TNC_CONNECT	12	// physical pin 16
#define OPEN_TNC_STATION	13	// physical pin 17
#define OPEN_TNC_PTT_LED	14	// physical pin 19

// CAN'T USE GPIO 19 (ISR_PIN) physical pin 25
#define OPEN_TNC_SW0		18	// physical pin 24
#define OPEN_TNC_SW1		21	// physical pin 27
#define OPEN_TNC_SW2		22	// physical pin 29
#endif

// Added by Codex
// Set to 1 to enable extra runtime diagnostics around RX/TNC handoff.
#ifndef CODEX_RX_DIAGNOSTICS
#define CODEX_RX_DIAGNOSTICS 1
#endif

// Set to 1 to enable extra TNC EMULATION diagnostics.
//#ifndef TNCEMUDEBUG
//#define TNCEMUDEBUG 1
//#endif

// number of ports
#define PORT_N 1    // number of ports, 1..3

#define BAUD_RATE 1200
#define SAMPLING_N 11
//#define DELAY_N 3
//#define SAMPLING_RATE ((1000000*DELAY_N+DELAY_US/2)/DELAY_US)
#define SAMPLING_RATE (BAUD_RATE * SAMPLING_N)
#define DELAY_US 446 // 446us
#define DELAYED_N ((SAMPLING_RATE * DELAY_US + 500000) / 1000000)

#define ADC_SAMPLING_RATE (SAMPLING_RATE * PORT_N)

#define DATA_LEN 1024                   // packet receive buffer size

#define FIR_LPF_N 27
#define FIR_BPF_N 25

#define ADC_BIT 8       // adc bits 8 or 12
//#define ADC_BIT 12      // adc bits 8 or 12

//#define BELL202_SYNC 1  // sync decode
#define DECODE_PLL 1    // use PLL
#define OPEN_SQUELCH    // if using open squelch requires pll

#define CONTROL_N 10
#define DAC_QUEUE_LEN 64
#define DAC_BLOCK_LEN (DAC_QUEUE_LEN + 1)

#define SEND_QUEUE_LEN (1024 * 16)

#define AX25_FLAG 0x7e

#define BUSY_PIN 22

#define KISS_PACKET_LEN 1024                // kiss packet length
#define TTY_N 3                             // number of serial
#define CMD_BUF_LEN 255

/* Additional Z80_STATE status flag to request emulation termination. */
#define FLAG_STOP_EMULATION     (1 << 31)

/* Emulation Defines */
#define Z80_CPU_SPEED           8195200   /* In Hz. */
#define CYCLES_PER_PASS         (Z80_CPU_SPEED / 400)
#define CYCLES_PER_INT		    (CYCLES_PER_PASS / 10) /*Cycles to run for each int processing */
//#define DEFAULT_BBS_MSG "Happy if u post msg"
#define DEFAULT_BBS_MSG   "New? Read INFO msg."
#define TIME_1SECOND 100 // 10ms * 100

/* Rom image is externally linked in. */
extern unsigned char _binary_hk21rom_bin_start;
extern unsigned char _binary_hk21rom_bin_end;
extern unsigned char _binary_hk21rom_bin_size;

/* Rom Image for Tnc Emulator */
extern unsigned char *Rom;


/* Dip switch setting optoons */
enum DIP_OPTIONS {
    CON_KIS = 0,    /* USB attached to TNC Console, Serial is KISS PORT */
    KIS_CON,        /* USB is KISS PORT, Serial attach to TNC Console */
    CON_CON,        /* Both USB and Serial attach to TNC Console */
    KIS_KIS,        /* Both USB and Serial are Kiss Ports */
    CON_MSG,        /* USB attached to TNC Console, new bbs msgs dumped to serial */
    MSG_CON,        /* New bbs msgs dumped to USB, serial to TNC Console */
    SPARE,          /* Future Use */
    BOOTLOAD        /* Enter Bootloader */
};

enum STATE {
	FLAG,
	DATA
};

typedef struct {
  int low_i;
  int low_q;
  int high_i;
  int high_q;
} values_t;

typedef struct TTY tty_t;

typedef struct TNC {
    uint8_t port;

    // receive

    // demodulator
    uint8_t bit;

    // decode_bit
    uint16_t data_cnt;
    uint8_t data[DATA_LEN];
    uint8_t state;
    uint8_t flag;
    uint8_t data_byte;
    uint8_t data_bit_cnt;

    // decode
    uint8_t edge;
    
    // decode2
    int32_t pll_counter;
    int32_t pll_quality; // PLL lock confidence
    uint8_t pval;
    uint8_t nrzi;

    // bell202_decode
    int delayed[DELAYED_N];
    int delay_idx;
    int cdt;
    int cdt_lvl;
    int avg;
    uint8_t cdt_pin;

    // bell202_decode2
    int sum_low_i;
    int sum_low_q;
    int sum_high_i;
    int sum_high_q;
    int low_idx;
    int high_idx;
    values_t values[SAMPLING_N];
    int values_idx;
    filter_t lpf;
    filter_t bpf;

    // send

/* Kiss Parameter Offset defines */
#define NUMKISSPARMS 5
#define KISS_TXDELAY 0
#define KISS_P 1
#define KISS_SLOT 2
#define KISS_TXTAIL 3
#define KISS_FULLDUPLEX 4

/* TncEmu emulation requires a minimum tx delay of 55ms */
#define MIN_TNCEMU_TXDELAY 55

/* Define IO Ports */
#ifdef OPEN_TNC_GPIO
#define DIP_SWITCH_0 OPEN_TNC_SW0
#define DIP_SWITCH_1 OPEN_TNC_SW1
#define DIP_SWITCH_2 OPEN_TNC_SW2
#else
#define DIP_SWITCH_0 16
#define DIP_SWITCH_1 17
#define DIP_SWITCH_2 18
#endif

#define CON_LED_GPIO 14 /* IO for Console LED */
#define STA_LED_GPIO 15 /* IO for Station LED */
#define TNC_EMULATING_LED_PIN PICO_DEFAULT_LED_PIN

#define DEFAULT_ACTIVITY_COUNT 1000

    uint8_t ax25_parms[NUMKISSPARMS];

    // dac queue
    queue_t dac_queue;

    // DAC, PTT pin
    uint8_t ptt_pin;
    uint8_t pwm_pin;
    uint8_t pwm_slice;

    // DMA channels
    uint8_t ctrl_chan;
    uint8_t data_chan;
    uint32_t data_chan_mask;
    uint8_t busy;

    // Bell202 wave generator
    int next;
    int phase;
    int level;
    int cnt_one;

    // wave buffer for DMA
    uint32_t const *dma_blocks[DAC_BLOCK_LEN][CONTROL_N + 1];

    // send data queue
    queue_t send_queue;
    int send_time;
    int send_len;
    int send_state;
    int send_data;

    // calibrate
    uint8_t cal_data;
    bool do_nrzi;
    uint32_t cal_time;
    tty_t *ttyp;

    // station and console leds
    uint8_t conled_pin;
    uint8_t staled_pin;

    int active_timeout;
    uint conSpeed;
    uint8_t ctc2_control;
    uint8_t ctc2_tc;


} tnc_t;

extern tnc_t tnc[];
extern uint32_t __tnc_time;

void tnc_init(void);
void tnc_emulate(void);
int IO_in (int);
void IO_out (int, int);
void SIO_Reset( IC_SIO *);
int SIO_Cmd_Read( IC_SIO *);
unsigned int Memory_Read_Byte(unsigned int);
unsigned int Memory_Read_Word(unsigned int);
void Memory_Write_Byte(unsigned int, unsigned int);
void Memory_Write_Word(unsigned int, unsigned int);
int kbhit(void);
char tobcd(unsigned int);
char frombcd(unsigned int bcd);
void RewriteBbsMsg(int addr, char *txt );
unsigned int GetNextBbsMsgNo(void);
bool consolePeek(void);
int consoleInput(void);
void consoleOutput(uint8_t c);
void consoleOutputStr(uint8_t const *str);
void Con_Serial_ParmChange(void);

inline uint32_t tnc_time(void)
{
    return __tnc_time;
}

// TNC command
enum MONITOR {
    MON_ALL = 0,
    MON_OFF,
};

// tty
enum TTY_MODE {
    TTY_TERMINAL = 0,
    TTY_GPS,
};

enum TTY_SERIAL {
    TTY_USB = 0,
    TTY_UART0,
    TTY_UART1,
};

typedef struct TTY {
    uint8_t kiss_buf[KISS_PACKET_LEN];
    int kiss_idx;
    bool kiss_mode;  // kiss mode
    uint8_t kiss_state; // kiss state
    uint32_t kiss_timeout; // kiss timer
    bool con_mode; // console attached to tnc

    uint8_t input_buf[CMD_BUF_LEN + 1];
    int inp_head;
    int inp_tail;

    uint8_t num;        // index of tty[]

    uint8_t tty_mode;   // terminal or GPS
    uint8_t tty_serial; // USB, UART0, UART1

    // Decode Monitor
    uint8_t montype;

    tnc_t *tp;          // input/output port No.
} tty_t;

extern tty_t tty[];

// send process state
enum SEND_STATE {
    SP_IDLE = 0,
    SP_WAIT_CLR_CH,
    SP_P_PERSISTENCE,
    SP_WAIT_SLOTTIME,
    SP_PTT_ON,
    SP_SEND_FLAGS,
    SP_DATA_START,
    SP_DATA,
    SP_ERROR,
    SP_CALIBRATE,
    SP_CALIBRATE_OFF,
};
