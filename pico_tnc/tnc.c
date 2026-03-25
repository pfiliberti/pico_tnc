/*
Copyright (c) 2021, JN1DFF
All rights reserved.
Copyright (c) 2025, KF7PSM

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
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"

#include "tnc.h"
#include "z80emu.h"

#include "ax25.h"
#include "flash.h"
#include "tty.h"
#include "send.h"
#include "kiss.h"
#include "receive.h" // added by Codex

uint32_t __tnc_time;

tnc_t tnc[PORT_N];

double  cycles,timer_int,sio_int;
double  total;

#if CODEX_RX_DIAGNOSTICS
// Added by Codex
struct {
  unsigned int rx_queue_inserted;
  unsigned int rx_queue_dropped_full;
  unsigned int rx_queue_started;
  unsigned int rx_queue_completed;
  unsigned int rx_queue_restarts;
  unsigned int rx_queue_max_depth;
  unsigned int rx_packet_max_len;
  unsigned int adc_dma_overruns;
  unsigned int tx_out_overflows;
  unsigned int receive_backlog_max;      // Added by Codex
  unsigned int receive_drain_loops;      // Added by Codex
  unsigned int receive_drain_max;        // Added by Codex
  unsigned int emulate_long_passes;      // Added by Codex
  unsigned int emulate_max_cycles;       // Added by Codex
  unsigned int tx_bytes_from_emu;      // Added by Codex
  unsigned int tx_ptt_asserts;         // Added by Codex
  unsigned int tx_feedflag_sets;       // Added by Codex
  unsigned int tx_extstat_interrupts;  // Added by Codex
  unsigned int tx_sendpacket_calls;    // Added by Codex
  unsigned int tx_sendpacket_fails;    // Added by Codex
  unsigned char last_sioa_cmd;         // Added by Codex
} codex_diag = {0};
#endif

/* tnc emulator */
unsigned char flop,oldptt;
unsigned int RxCharIn_Idx=0; // int so packets can't wrap at 255!
unsigned char ax25rdy=0;
unsigned char feedflag=0;
unsigned char abortflag=0;
unsigned char txundr_count=0;
unsigned short int mycrc;
int rxcnt;

/* Output Buffer */
unsigned char Ax25_Out[BUFLEN];
unsigned int  Ax25_Out_Cnt;
unsigned int  Ax25_In_Dly = 0;

unsigned int PrevbbsMsgNo;
unsigned int clock_address = 0; /* Clock stucture in TNC Ram */
unsigned int bbsmsg_address = 0;

uint32_t parm_check_time = 0;
bool newMsg = false;
bool newMsgFlashState = false;

/* for snprintf */
uint8_t msgbuf[25];

/* Locations in ram where z80 code stores these parameters */
unsigned int ax25_parm_location[NUMKISSPARMS-1]= {0x3FDB, 0x4033, 0x4035, 0x3FD7};

/* The Emulated TNC has 32k of RAM and 32k of ROM.
   Rom is addressed starting at 0 and Ram at 0x8000 */
unsigned char *Rom = &_binary_hk21rom_bin_start;

/* Emulated TNC Ram Space */
unsigned char   Ram[1 << 15];

/* Declare struct vars for SIO channels */
IC_SIO	sioa;
IC_SIO	siob;
Z80_STATE       state;

void tnc_init(void)
{
  int x;
  /* Init ax25 Receive Q pointers */
  ax25_init_Q();

  rtc_init(); // Initialize the RTC

  clock_address = 0x4f0a; /* Where tnc keeps time */
  bbsmsg_address = 0x4f06; /* where tnc stores msg count */

  /* Patch for rom we can manually patch later, Needed? */
//  Rom[0x5032] = Rom[0x5041];
  Rom[0x5032] = 0x3e;

  /* Stuff NOPs to disable strange obfuscation of text */
  for(int x=0; x< 11; x++) Rom[0x47f7+x] = 0;
  Rom[0x47f7+12] = 0;

  /* Read GPIO ? and if set clear ram */
  if(0)
  {
    for(x=0; x<sizeof(Ram); x++)
    {
        Ram[x]=0;
    }
  }
  else // read saved ram memory from flash
  {
      consoleOutputStr("TNCEMU:Reading Ram Data from Flash ");
      int slot = flash_read(Ram, sizeof(Ram));
      if (slot >= 0) 
      {
          snprintf(msgbuf,sizeof(msgbuf),"slot %d\n", slot);
      } else 
      {
          snprintf(msgbuf,sizeof(msgbuf),".\nRead failed!\n");
      }
      consoleOutputStr(msgbuf);
  }

  char NewBBsMsg[] = DEFAULT_BBS_MSG;
  /* Throw some custom text into eprom for when user logs into bbs */
  /* Replaces "Heath System" */
  RewriteBbsMsg(0x2dad, NewBBsMsg);

  /* initialize the previous bbs msg # to current in memory
  for later comparison to see if a msg was added */
  PrevbbsMsgNo = GetNextBbsMsgNo();

  // Set a dummy time if the RTC is not already set (optional, for testing)
  datetime_t initial_time = {
      .year = 2000,
      .month = 8,
      .day = 6,
      .dotw = 3, // Wednesday
      .hour = 12,
      .min = 0,
      .sec = 0
  };
  rtc_set_datetime(&initial_time);
  Ram[clock_address+5] = 0x20; // Set year in tnc ram

  // filter initialization
  // LPF
  static const filter_param_t flt_lpf = {
      .size = FIR_LPF_N,
      .sampling_freq = SAMPLING_RATE,
      .pass_freq = 0,
      .cutoff_freq = 1200,
  };
  int16_t *lpf_an, *bpf_an;

  lpf_an = filter_coeff(&flt_lpf);

#if 0
  printf("LPF coeffient\n");
  for (int i = 0; i < flt_lpf.size; i++) {
      printf("%d\n", lpf_an[i]);
  }
#endif
  // BPF
  static const filter_param_t flt_bpf = {
      .size = FIR_BPF_N,
      .sampling_freq = SAMPLING_RATE,
      .pass_freq = 900,
      .cutoff_freq = 2500,
  };
  bpf_an = filter_coeff(&flt_bpf);
#if 0
  printf("BPF coeffient\n");
  for (int i = 0; i < flt_bpf.size; i++) {
      printf("%d\n", bpf_an[i]);
  }
#endif
  // PORT initialization
  for (x = 0; x < PORT_N; x++) {
      tnc_t *tp = &tnc[x];

      // Console and station leds set gpio pins
      tp->conled_pin = CON_LED_GPIO;
      tp->staled_pin = STA_LED_GPIO;

      gpio_init(tp->conled_pin);
      gpio_set_dir(tp->conled_pin, GPIO_OUT);
      gpio_init(tp->staled_pin);
      gpio_set_dir(tp->staled_pin, GPIO_OUT);

#ifdef TNC_EMULATING_LED_PIN
      gpio_init(TNC_EMULATING_LED_PIN);
      gpio_set_dir(TNC_EMULATING_LED_PIN, GPIO_OUT);
      gpio_put(TNC_EMULATING_LED_PIN,0);
#endif

      // receive
      tp->port = x;
      tp->state = FLAG;
      filter_init(&tp->lpf, lpf_an, FIR_LPF_N);
      filter_init(&tp->bpf, bpf_an, FIR_BPF_N);

      // send queue
      queue_init(&tp->send_queue, sizeof(uint8_t), SEND_QUEUE_LEN);
      tp->send_state = SP_IDLE;

      tp->cdt = 0;
      tp->ax25_parms[KISS_TXDELAY] = 50;
      tp->ax25_parms[KISS_P] = 63;
      tp->ax25_parms[KISS_SLOT] = 10;
      tp->ax25_parms[KISS_TXTAIL] = 0;
      tp->ax25_parms[KISS_FULLDUPLEX] = 0;

      // calibrate
      tp->do_nrzi = true;
  }

  //printf("%d ports support\n", PORT_N);
  //printf("DELAYED_N = %d\n", DELAYED_N);

  tnc_t *tp = &tnc[0];

  /* Init ctc2 emulated ctc reg values */
  tp->ctc2_control = 0;
  tp->ctc2_tc = 0;
  tp->conSpeed = 75; /* 75 baud gets translated to 115200 in Con_Serial_ParmChange*/

  /* Memorize certain ax25 parms to detect any changes later */
  for(x=0; x< NUMKISSPARMS-1; x++) /* start 1 skip txdelay for now */
  {
    tp->ax25_parms[x] = Ram[ax25_parm_location[x]];
  }

    /* Reset emulated SIO state machines */
    SIO_Reset(&sioa); /* Reset Emulated Serial i/o a */
    SIO_Reset(&siob); /* Reset Emulated Serial i/o b */

    /* Reset z80 emulator */
    Z80Reset(&state);

    /* Init z80 cycle time counters */
    total = timer_int = sio_int = 0;

    parm_check_time = tnc_time();
}

/* Run some cycles of emulated tnc */
void tnc_emulate(void)
{
    bool flashUpdate = false;
    unsigned int x;
    tnc_t *tp = &tnc[0];

#ifdef TNCEMUDEBUG
    printf("PC=%x cycles=%.0f\n",state.pc,total);
    cycles = Z80Emulate(&state, 1);
#endif
    cycles = Z80Emulate(&state, CYCLES_PER_PASS);
    total += cycles;
    timer_int += cycles;
    sio_int += cycles;

#if CODEX_RX_DIAGNOSTICS
    // Added by Codex
    if ((unsigned int)cycles > codex_diag.emulate_max_cycles)
      codex_diag.emulate_max_cycles = (unsigned int)cycles;
    if (cycles > (CYCLES_PER_PASS * 2))
      codex_diag.emulate_long_passes++;
#endif


  /* Every other run do a timer interrupt, we come into the emulator
  roughly ever 10ms not super acurate but it */
  if( timer_int >= 150000)
  {
    timer_int = 0;
    cycles = 0;
#if CODEX_RX_DIAGNOSTICS
    // Added by Codex
    unsigned int codex_wait_loops = 0;
#endif

    while(!state.iff1)
    {
      cycles += Z80Emulate(&state, CYCLES_PER_INT );
#if CODEX_RX_DIAGNOSTICS
      // Added by Codex
      codex_wait_loops++;
#endif
      watchdog_update();

      // Added by Codex
      if ((++codex_wait_loops & 0x03) == 0) {
        receive();
      }      
    }
    cycles += Z80Interrupt (&state, 0x10 );
    total += cycles;
    timer_int += cycles;
    sio_int += cycles;
#if CODEX_RX_DIAGNOSTICS
    // Added by Codex
    if (codex_wait_loops > 50) codex_diag.emulate_long_passes++;
    if (codex_wait_loops > codex_diag.emulate_max_cycles)
      codex_diag.emulate_max_cycles = codex_wait_loops;
#endif
  }

  /* Every second update our clock from pico rtc */
  if (tnc_time() - parm_check_time >= TIME_1SECOND)
  {
    parm_check_time = tnc_time();
    datetime_t current_time;
    rtc_get_datetime(&current_time);
   // printf("Pllq=%d,%d\n",tp->pll_quality,tp->flag_count);
    // Hear check if our Clock's year matches PICO RTC and if not
    // Update RTC from tncemu's clock.
    x= current_time.year;
    x = x - ((x / 100) * 100);
    if( Ram[clock_address+5] != tobcd(x) )
    {
      // printf("RTC Before %d:%d:%d:%d:%d\n",current_time.year,current_time.month,current_time.day,
      //   current_time.hour,current_time.min);
      current_time.sec = 0;
      current_time.min = frombcd(Ram[clock_address+1]);
      current_time.hour = frombcd(Ram[clock_address+2]);
      current_time.day = frombcd(Ram[clock_address+3]);
      current_time.month = frombcd(Ram[clock_address+4]);
      current_time.year = frombcd(Ram[clock_address+5]) + 2000;
      rtc_set_datetime(&current_time);
      // printf("RTC Update %d:%d:%d:%d:%d\n",current_time.year,current_time.month,current_time.day,
      //   current_time.hour,current_time.min);
    }
    else
    {
      /* here update tnc time with our pico rtc time */
      x= current_time.sec;
      Ram[clock_address] = tobcd(x);
      x= current_time.min;
      Ram[clock_address+1] = tobcd(x);
      x= current_time.hour;
      Ram[clock_address+2] = tobcd(x);
      x= current_time.day;
      Ram[clock_address+3] = tobcd(x);
      x= current_time.month;
      Ram[clock_address+4] = tobcd(x);
      // Don't need to update years as they match
      // x= current_time.year;
      // x= x - ((x / 100) * 100);
      // Ram[clock_address+5] = tobcd(x);
    }

    /* Check if any new bbs msgs have arrived and if so save ram to disk */
    if( PrevbbsMsgNo != GetNextBbsMsgNo())
    {
      /* Check if next bbs msg no is > than previous otherwise no new msg */
      if(GetNextBbsMsgNo() > PrevbbsMsgNo)
      {
        newMsg = true;
      }
      flashUpdate = true;
      PrevbbsMsgNo = GetNextBbsMsgNo();
    }

    /* compare saved kiss parms to ram parms and if KISS_TXDELAY changed update flash 
        after updating ax25_parms to match emulator */
    for(x=0; x< NUMKISSPARMS-1; x++)
    {
      if(tp->ax25_parms[x] != Ram[ax25_parm_location[x]] )
      {
        tp->ax25_parms[x] = Ram[ax25_parm_location[x]];
        if(x == KISS_TXDELAY) flashUpdate = true; /* only update if this parm is changed */
      }
    }

    if(flashUpdate)
    {
      flashUpdate = false;

      /* Disable watchdog during flash writes */
      watchdog_disable();

      consoleOutputStr("TNCEMU:Saving Ram Data to Flash ");
      int slot = flash_write(Ram, sizeof(Ram));
      if (slot >= 0) 
      {
          snprintf(msgbuf,sizeof(msgbuf),"slot %d\n", slot);
      } else 
      {
          snprintf(msgbuf,sizeof(msgbuf),".\nWrite failed!\n");
      }
      consoleOutputStr(msgbuf);

      // set watchdog, timeout 1000 ms
      watchdog_enable(1000, true);
    }

    /* Here check SIO dtr values and set gpio's for leds according to status */
    gpio_put(tp->conled_pin, !(siob.registers[5] & 0x80));

    if((sioa.registers[5] & 0x80) == 0x80 && newMsg == true ) 
    {
      gpio_put(tp->staled_pin, newMsgFlashState);
      newMsgFlashState = !newMsgFlashState;
    }
    else
    {
      gpio_put(tp->staled_pin, !(sioa.registers[5] & 0x80));
    }
  }

  if(sio_int > 55000 )
  {
    flop = flop ^0x01;
    sio_int = 0;
    if(flop)
    {

      if(RxCharIn_Idx || ax25rdy)
      {
        if(RxCharIn_Idx)
        {
          cycles = 0;
#if CODEX_RX_DIAGNOSTICS
        // Added by Codex
        unsigned int codex_wait_loops = 0;
#endif
          while(!state.iff1)
          {
            cycles += Z80Emulate(&state, CYCLES_PER_INT );
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          codex_wait_loops++;
#endif
            watchdog_update();
            // Added by Codex
            if ((++codex_wait_loops & 0x03) == 0) {
              receive();
            }            
          }
          cycles += Z80Interrupt (&state, siob.registers[2] | 0x0c);   // ax25 char read int
          total += cycles;
          timer_int += cycles;
          sio_int += cycles;
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          if (codex_wait_loops > 50) codex_diag.emulate_long_passes++;
          if (codex_wait_loops > codex_diag.emulate_max_cycles)
            codex_diag.emulate_max_cycles = codex_wait_loops;
#endif
        }

        if(ax25rdy)
        {
          cycles = 0;
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          unsigned int codex_wait_loops = 0;
#endif
          while(!state.iff1)
          {
            cycles += Z80Emulate(&state, CYCLES_PER_INT );
#if CODEX_RX_DIAGNOSTICS
            // Added by Codex
            codex_wait_loops++;
#endif
            watchdog_update();
            // Added by Codex
            if ((++codex_wait_loops & 0x03) == 0) {
              receive();
            }            
          }
          cycles += Z80Interrupt (&state, siob.registers[2] | 0x0e); // eof int
          total += cycles;
          timer_int += cycles;
          sio_int += cycles;
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          if (codex_wait_loops > 50) codex_diag.emulate_long_passes++;
          if (codex_wait_loops > codex_diag.emulate_max_cycles)
            codex_diag.emulate_max_cycles = codex_wait_loops;
#endif
        }
      }
      else
      {
        if(txundr_count)
        {
          if(--txundr_count == 0)
          {
            feedflag = 1; /* txunderrun we can send packet!*/
            codex_diag.tx_feedflag_sets++;
            if(Ax25_Out_Cnt)
            {
              codex_diag.tx_sendpacket_calls++;
              if(!send_packet(&tnc[0], Ax25_Out, Ax25_Out_Cnt)) codex_diag.tx_sendpacket_fails++;
              else Ax25_Out_Cnt = 0;
            }
          }
        }

        if(feedflag || abortflag )
        {
          codex_diag.tx_extstat_interrupts++;
          cycles = 0;
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          unsigned int codex_wait_loops = 0;
#endif
          while(!state.iff1)
          {
            cycles += Z80Emulate(&state, CYCLES_PER_INT );
#if CODEX_RX_DIAGNOSTICS
            // Added by Codex
            codex_wait_loops++;
#endif
            watchdog_update();
            // Added by Codex
            if ((++codex_wait_loops & 0x03) == 0) {
              receive();
            }
          }
          cycles += Z80Interrupt (&state, siob.registers[2] | 0x0a); // ext stat int
          total += cycles;
          timer_int += cycles;
          sio_int += cycles;
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          if (codex_wait_loops > 50) codex_diag.emulate_long_passes++;
          if (codex_wait_loops > codex_diag.emulate_max_cycles)
            codex_diag.emulate_max_cycles = codex_wait_loops;
#endif
        }
        else
        {
          if(siob.registers[1] & 2)
          {
            cycles = 0;
#if CODEX_RX_DIAGNOSTICS
            // Added by Codex
            unsigned int codex_wait_loops = 0;
#endif
            while(!state.iff1)
            {
              cycles += Z80Emulate(&state, CYCLES_PER_INT );
#if CODEX_RX_DIAGNOSTICS
              // Added by Codex
              codex_wait_loops++;
#endif
              watchdog_update();
              // Added by Codex
              if ((++codex_wait_loops & 0x03) == 0) {
                receive();
              }              
            }
            cycles += Z80Interrupt (&state, siob.registers[2] );
            total += cycles;
            timer_int += cycles;
            sio_int += cycles;
#if CODEX_RX_DIAGNOSTICS
            // Added by Codex
            if (codex_wait_loops > 50) codex_diag.emulate_long_passes++;
            if (codex_wait_loops > codex_diag.emulate_max_cycles)
              codex_diag.emulate_max_cycles = codex_wait_loops;
#endif
          }
        }
      }
    }
    else /* flip */
    {
      if( consolePeek() )
      {
// This breaks inital autobaud!   if(state.iff1 && (siob.registers[1] & 0x18) )
//      {
        cycles = 0;
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          unsigned int codex_wait_loops = 0;
#endif
        while(!state.iff1)
        {
          cycles += Z80Emulate(&state, CYCLES_PER_INT );
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          codex_wait_loops++;
#endif
          watchdog_update();
          // Added by Codex
          if ((++codex_wait_loops & 0x03) == 0) {
            receive();
          }          
        }
        cycles += Z80Interrupt (&state, siob.registers[2] | 4);
        total += cycles;
        timer_int += cycles;
        sio_int += cycles;
#if CODEX_RX_DIAGNOSTICS
        // Added by Codex
        if (codex_wait_loops > 50) codex_diag.emulate_long_passes++;
        if (codex_wait_loops > codex_diag.emulate_max_cycles)
          codex_diag.emulate_max_cycles = codex_wait_loops;
#endif
//      }
        tp->active_timeout = DEFAULT_ACTIVITY_COUNT;
      } 
      else 
      {
#if CODEX_RX_DIAGNOSTICS
    // Added by Codex
        unsigned int codex_wait_loops = 0;
#endif
        cycles = 0;
        while(!state.iff1)
        {
          cycles += Z80Emulate(&state, CYCLES_PER_INT );
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          codex_wait_loops++;
#endif
          watchdog_update();
          // Added by Codex
          if ((++codex_wait_loops & 0x03) == 0) {
            receive();
          }
        }
        cycles += Z80Interrupt (&state, siob.registers[2] | 8 );
        total += cycles;
        timer_int += cycles;
        sio_int += cycles;
#if CODEX_RX_DIAGNOSTICS
    // Added by Codex
        if (codex_wait_loops > 50) codex_diag.emulate_long_passes++;
        if (codex_wait_loops > codex_diag.emulate_max_cycles)
          codex_diag.emulate_max_cycles = codex_wait_loops;
#endif
      }
    }

    // if(ax25_InQ_HasData() && !RxCharIn_Idx && !ax25rdy && !txundr_count  && !Ax25_In_Dly ) /* do we have a socket */
    // if(ax25_InQ_HasData() && !RxCharIn_Idx && !ax25rdy && !txundr_count && !(sioa.registers[5] & 2) && !Ax25_In_Dly )
    if(ax25_InQ_HasData() && !RxCharIn_Idx && !ax25rdy && !txundr_count  && !Ax25_In_Dly && !feedflag && !abortflag ) /* do we have a socket */
    {
#if CODEX_RX_DIAGNOSTICS
      // Added by Codex
      unsigned int queued = (Ax25_In_Head + AX25_IN_MAXSIZE - Ax25_In_Tail) % AX25_IN_MAXSIZE;
      codex_diag.rx_queue_started++;
      if (queued > codex_diag.rx_queue_max_depth) codex_diag.rx_queue_max_depth = queued;
      if (codex_diag.rx_queue_started > codex_diag.rx_queue_completed + 1) codex_diag.rx_queue_restarts++;
#endif
      RxCharIn_Idx = 1; /* Let everyone know */
      Ax25_In_Dly = 75; /* this is an arbitrary delay amount so emulator can process rx packets */

      /* Before removing any incoming ax25 packets send to any kiss ports */
      // incoming KISS frame to serial
      if(tty[0].kiss_mode) kiss_output(&tty[0],&tnc[0]);
      if(tty[1].kiss_mode) kiss_output(&tty[1],&tnc[0]);
    }

    if(Ax25_In_Dly && !RxCharIn_Idx && !txundr_count) Ax25_In_Dly--;
  }

  if(oldptt != (sioa.registers[5] & 2))
  {
    oldptt = sioa.registers[5] & 2;
#ifdef TNCEMUDEBUG
    printf("ptt=%x\n",oldptt);
#endif
    if(oldptt == 2)
    {
      codex_diag.tx_ptt_asserts++;
      txundr_count=10;
      Ax25_Out_Cnt=0; // Matches tcp version
    }
  }

  /* Here check status of tnc buffers and if work to do set activity */
  if(RxCharIn_Idx > 0 || Ax25_Out_Cnt > 0 )
  {
    //printf("%d-%d\n",RxCharIn_Idx,Ax25_Out_Cnt);
    tp->active_timeout = DEFAULT_ACTIVITY_COUNT;
  }

  /* If activity timer set decrement until 0 */
  if(tp->active_timeout > 0)
  {
    tp->active_timeout--;
#ifdef TNC_EMULATING_LED_PIN
    gpio_put(TNC_EMULATING_LED_PIN,1);
#endif
  }

#ifdef TNCEMUDEBUG
  if (state.status & FLAG_STOP_EMULATION) 
  {
    printf("\n%.0f cycle(s) emulated.\n" 
    "For a Z80 running at %.2fMHz, "
    "that would be %d second(s) or %.2f hour(s).\n",
    total,
    Z80_CPU_SPEED / 1000000.0,
    (int) (total / Z80_CPU_SPEED),
    total / ((double) 3600 * Z80_CPU_SPEED));
  }
#endif

#ifdef TNC_EMULATING_LED_PIN
    if(tp->active_timeout == 0) gpio_put(TNC_EMULATING_LED_PIN,0);
#endif
}

/*************************************************************************
  Here we emulate as best we can the I/O of the Toshiba TMPZ84C015 CPU
  Integrated CPU/IO I/C. The following is the portmap of the i/o

  Internal Halt Mode Setting Registers
  This controls how the ic internal osc operates during certain cpu 
  halt modes. Also watchdog is in hee. Probably can ignore since we are emulating.

  Halt Mode Settings Register 0xF0 
  Code writes an 0x7B Putting All devices in Powered up Run Mode.

  Halt Mode Control Register 0XF1
  Code writes an 0xB1 to disable watchdog timer since wdog enable bit in 0xF0 is off

  Interrupt Priority Register 0xF4 Bits 1,2,0 as follows
  HIGH  TO  LOWEST PRIORITY
  CTC - SIO - PIO 0 0 0 
  SIO - CTC - PIO 0 0 1 (THIS IS THE ONE THE CODE SELECTS)
  CTC - PIO - SIO 0 1 0
  PIO - SIO - CTC 0 1 1
  PIO - CTC - SIO 1 0 0
  SIO - PIO - CTC 1 0 1
  
  CTC Timer I/O Map
  0x10 = Chan 0
  0x11 = Chan 1
  0x12 = Chan 2
  0x13 = Chan 3

  SIO Serial Device I/O Map 
  0x18 = Chan A Data
  0x19 = Chan A Command
  0x1A = Chan B Data
  0x1B = Chan B Command

  PIO I/O Map
  0x1C = Port A Data
  0x1D = Port A Command
  0x1E = Port B Data
  0x1F = Port B Command

*/

int IO_in (int port)
{
  int x=0;
  port &= 255;

  switch (port)
  {
    case 0x10: // CTC Chan0
      break;

    case 0x11: // CTC Chan1
      break;

    case 0x12: // CTC Chan2
      break;

    case 0x13: // CTC Chan3
      break;

    case 0x18: // SIOA Data
      x=0xff;
      if(RxCharIn_Idx) 
      {
        x = Ax25_In_Q[Ax25_In_Tail].data[RxCharIn_Idx-1];
//printf("%x\n",x);
        RxCharIn_Idx++;
        if(--Ax25_In_Q[Ax25_In_Tail].count == 0) 
        {
          RxCharIn_Idx = 0;
#if CODEX_RX_DIAGNOSTICS
          // Added by Codex
          codex_diag.rx_queue_completed++;
#endif
          ax25_InQ_Remove();
          ax25rdy=1;
        }
      }
      break;

    case 0x19: // SIOA Cmd
      x = SIO_Cmd_Read( &sioa );
      break;

    case 0x1A: // SIOB Data
      x = consoleInput();

//#ifdef TNCEMUDEBUG
#ifdef CODEX_RX_DIAGNOSTICS
      if(x == '&')
      {
        // Added by Codex
        printf("\nCodex RX diag: in=%u drop=%u start=%u done=%u restart=%u qmax=%u pktmax=%u adcovr=%u txovr=%u\n",
          codex_diag.rx_queue_inserted,
          codex_diag.rx_queue_dropped_full,
          codex_diag.rx_queue_started,
          codex_diag.rx_queue_completed,
          codex_diag.rx_queue_restarts,
          codex_diag.rx_queue_max_depth,
          codex_diag.rx_packet_max_len,
          codex_diag.adc_dma_overruns,
          codex_diag.tx_out_overflows);
        printf(" rxbacklog=%u drainloops=%u drainmax=%u elong=%u emax=%u\n",
          codex_diag.receive_backlog_max,
          codex_diag.receive_drain_loops,
          codex_diag.receive_drain_max,
          codex_diag.emulate_long_passes,
          codex_diag.emulate_max_cycles); // Added by Codex

        printf("\n Diagnostic Info:\n");
        printf("txunder=%d, feedflag=%d, Ax25OutCount=%d\n",txundr_count,feedflag,Ax25_Out_Cnt);
        printf("abotr=%d, busy=%d, sendState=%d, sendQfree=%d \n",abortflag,tnc[0].busy,tnc[0].send_state,send_queue_free(&tnc[0]));
        printf("SIO REG 0 = %x ",sioa.registers[0]);
        printf("SIO REG 1 = %x ",sioa.registers[1]);
        printf("SIO REG 2 = %x\n",sioa.registers[2]);
        printf("SIO REG 3 = %x ",sioa.registers[3]);
        printf("SIO REG 4 = %x ",sioa.registers[4]);
        printf("SIO REG 5 = %x\n",sioa.registers[5]);
        printf("SIO REG 6 = %x ",sioa.registers[6]);
        printf("SIO REG 7 = %x\n",sioa.registers[7]); // Added by Codex
        printf("SIO state=%d cmd_ptr=%d\n",sioa.state, sioa.cmd_ptr); // Added by Codex
        printf("RxCharIn_Idx=%u ax25rdy=%d Ax25_In_Dly=%u\n", RxCharIn_Idx, ax25rdy, Ax25_In_Dly); // Added by Codex
        printf("Ax25 In Head=%u Tail=%u HasData=%d\n", Ax25_In_Head, Ax25_In_Tail, ax25_InQ_HasData()); // Added by Codex
        printf("Current In Count=%u\n", Ax25_In_Q[Ax25_In_Tail].count); // Added by Codex

        printf("tx_bytes=%u ptt_asserts=%u feed_sets=%u extstat_ints=%u\n",
          codex_diag.tx_bytes_from_emu,
          codex_diag.tx_ptt_asserts,
          codex_diag.tx_feedflag_sets,
          codex_diag.tx_extstat_interrupts); // Added by Codex

        printf("sendpkt_calls=%u sendpkt_fails=%u last_sioa_cmd=%02x\n",
          codex_diag.tx_sendpacket_calls,
          codex_diag.tx_sendpacket_fails,
          codex_diag.last_sioa_cmd); // Added by Codex

        printf("oldptt=%d send_busy=%d send_state=%d send_q_free=%d dac_q_level=%d\n",
          oldptt,
          tnc[0].busy,
          tnc[0].send_state,
          send_queue_free(&tnc[0]),
          queue_get_level(&tnc[0].dac_queue)); // Added by Codex                
      }
#endif
      break;

    case 0x1B: // SIOB Cmd
      x = SIO_Cmd_Read( &siob );
      break;

    case 0x1C: // PIOA Data
      break;

    case 0x1D: // PIOA Cmd
      break;

    case 0x1E: // PIOB Data
      break;

    case 0x1F: // PIOB Cmd
      break;

    default:  // All else do nothing
      break;

  }

//    printf("IO In from port %x = %x:%x\n",port,x,state.pc);
  return (x);
}

void IO_out (int port, int x)
{
  port &= 255;

 // printf("IO out %x to port %x\n",x,port);

  switch (port)
  {
    case 0x10: // CTC Chan0
      break;

    case 0x11: // CTC Chan1
      break;

    /* This CTC Channel is used to set Baud rates for serial port */
    case 0x12: // CTC Chan2
      if(tnc[0].ctc2_control & 0x04) /* If time constant load bit is set */
      {
        tnc[0].ctc2_control &= 0xfb; /* reset load bit */
        tnc[0].ctc2_tc = x; /* set time constant */

        unsigned int ctc_div = 16;
        if( tnc[0].ctc2_control & 0x20 ) ctc_div = 256;

        /* Compute Baud Rate */
        unsigned int baud = 2457600 / ctc_div;
        baud = baud / tnc[0].ctc2_tc;
        baud = baud / 16;
        tnc[0].conSpeed = baud;
        Con_Serial_ParmChange();
      }
      else tnc[0].ctc2_control = x; /* set ctc2_control */

      break;

    case 0x13: // CTC Chan3
      break;

    case 0x18: // SIOA Data
      codex_diag.tx_bytes_from_emu ++;
      if(Ax25_Out_Cnt < BUFLEN) // Check for possible overflow
      {
        Ax25_Out[Ax25_Out_Cnt++] = x;
      }
#if CODEX_RX_DIAGNOSTICS
      else
      {
        // Added by Codex
        codex_diag.tx_out_overflows++;
      }
#endif      
      txundr_count=10; /* reset tx underrun */
      break;

    case 0x19: // SIOA Cmd
      codex_diag.last_sioa_cmd = x;
      SIO_Cmd_Write( &sioa, x);
      break;

    case 0x1A: // SIOB Data
      consoleOutput(x);
      tnc[0].active_timeout = DEFAULT_ACTIVITY_COUNT;
      break;

    case 0x1B: // SIOB Cmd
      SIO_Cmd_Write( &siob, x);
      break;

    case 0x1C: // PIOA Data
      break;

    case 0x1D: // PIOA Cmd
      break;

    case 0x1E: // PIOB Data
      break;

    case 0x1F: // PIOB Cmd
      break;

    default:  // All else do nothing
    break;

  }
}

/* SIO functions are here to handle emulation of SIO Channels */

/* Reset SIO registers and cmd ptr */
void SIO_Reset( IC_SIO *sio )
{
  // Added by Codex
  // Fully clear the emulated SIO state so channel resets behave like a fresh device.
  sio->state = 0; // Set state for cmd reg
  sio->cmd_ptr = 0; // Set cmd ptr to reg 0

  for (int i = 0; i < 8; i++) // Added by Codex
  {
    sio->registers[i] = 0; // Added by Codex
  }

  // Channel A owns the AX.25 sideband flags in this emulator, so clear them too.
  if (sio == &sioa)
  {
    ax25rdy = 0;
    feedflag = 0;
    abortflag = 0;
    txundr_count = 0;
    Ax25_In_Dly = 0;
    RxCharIn_Idx = 0;
  }
}

// void SIO_Reset( IC_SIO *sio )
// {
//   sio->state = 0; // Set state for cmd reg
//   sio->cmd_ptr = 0; // Set cmd ptr to reg 0
//   sio->registers[0] = 0; 
// }

/* Handle Writes to SIO Command Port */
void SIO_Cmd_Write( IC_SIO *sio, unsigned char x)
{
  unsigned char wr0_cmd = x & 0x38;
  if(sio->state) /* write to actual reg */
  {
//    if(abortflag && sio->cmd_ptr == 5 && !(x & 2)) 
//      printf("PC=%x\n",state.pc);
    sio->registers[sio->cmd_ptr] = x; 
    sio->cmd_ptr = 0; /* after a write it sets back to 0 */
    sio->state = 0; /* next state is command */
  }
  else /* set write register */
  {
    if(wr0_cmd != 0) /* A command for sio ? */
    {
      switch (wr0_cmd) // Process special cmd
      {
        case 0x08:    // Abort SDLC Seq 
          abortflag = 1;
          break;

        case 0x10:
          // external status interrupt reset
          break;

        case 0x18:
          SIO_Reset(sio); /* sio logic reset */
          return;

        case 0x20:
          // enable interrupt on next receive char
          break;

        case 0x28: // Reset Interrupt Pending
          feedflag = 0;
          break;

        case 0x30:
          // CLear Latched error bits
          break;

        case 0x38:
          // return from interrupt
          break;
      }
    }
    else // Cmd 0 = set reg address 
    {
      sio->cmd_ptr = x & 0x07; /* lsb 3 bits select reg for next write/read */
      sio->state = 1; /* flip state */
    }
    // If bits 6  & 7 are set its a reset of EOM Status 
    if((x & 0xc0) == 0xc0) feedflag = 0;
  }
}

/* Handle Reads from SIO Command Port */
int SIO_Cmd_Read( IC_SIO *sio )
{

int val = 0;

  switch ( sio->cmd_ptr )
  {
    case 0: /* Reg Indicates the rx/tx buffer state & pins state */
            /* MSB -> BRK/ABORT, UNDRRUN, CTS, SYNC/HUNT, DCD, TBUF_EMPTY
               INT_PENDING, RX_CHAR_RDY <-LSB */
      if(sio == &siob )
      {
        val = 0x2c; /* set CTS, DCD, TBUF_EMPTY always */
        if( consolePeek() ) val |=1; /* if keys in buffer set flag we have rx chars */  
      }
      else /* handle sioa */
      {
        val = val | 4; /* TBUF_EMPTY */
        if( sio->registers[5] & 2 ) val |= 0x20; /* cts is wired to rts so it follows it */
        if(abortflag) 
        {
          val |= 0x10; /* set Sync/Hunt */
          val &= 0xFB; /* Clear TFBUF_EMPTY emulating crc going out in uart */
          if(!(val & 0x20)) abortflag=0;
        }
        if( RxCharIn_Idx ) val |= 0x09; /* DCD & RX_CHAR_RDY */
        if(feedflag) val |= 0x40; 
      }
      break;
     
    case 1: /* Reg Indicates error status and end of frame code */
            /* MSB -> EOF_FRAM, FRAME_ERR, RX_OVRRUN, PARITY_ERROR
               NONE, FRACTION, NONE, TX_EMPTY or always 1 in SYNC MODE */
      if(sio == &siob )
      {
        val = 0x01	; /* TX Empty always! */
      }
      else /* chan a */
      {
        val= 0x01;
        if(ax25rdy)
        {
          val |= 0x86; /* set eof detected and correct fraction bits! */
          ax25rdy = 0;
          /* If input queue has more data retrigger to process next packet */
          /* scratch that, this is done abocve after a delay period now.*/
          //if(Ax25_In_HasData()) RxCharIn_Idx=1;
        }
      }

      break;

    case 2: /* Returns int vector but only for port b but we do both */
      if(sio == &siob )
        {
          val = sio->registers[2];
        } else val=0; /* no int vec on chan a! */

      break;

    default:
      break;

  }

  sio->state = 0;
  sio->cmd_ptr = 0;
//  sio->state = sio->state ^ 0x01; /* flip state */

  return val;
}

/* Memory Access Functions go here */

unsigned int Memory_Read_Byte(unsigned int address)
{
  if(address > 0x7FFF) return Ram[address & 0x7fff];
  else return Rom[address];
}

unsigned int Memory_Read_Word(unsigned int address)
{
  if(address > 0x7FFF) 
    return Ram[address & 0x7fff] | ( Ram[ (address+1) & 0x7fff ] << 8 );
  else
    return Rom[address] | ( Rom[ address+1 ] << 8 );
}

void Memory_Write_Byte(unsigned int address, unsigned int data)
{
  if(address > 0x7FFF) Ram[address & 0x7fff] = data & 0xff;
}

void Memory_Write_Word(unsigned int address, unsigned int data)
{
  if(address > 0x7FFF) 
  {
    Ram[address & 0x7fff] = data & 0xff; 
    if( ((address+1) & 0xffff) > 0x7fff)                                	
    Ram[(address + 1) & 0x7fff] = data >> 8; 
  }
}

// Convert int to bcd 
char tobcd(unsigned int val)
{
    if (val < 0 || val > 99) {
        return 0; // out of range
    }
    return (char)(((val / 10) << 4) | (val % 10));
}

// Convert from BCD
char frombcd(unsigned int bcd)
{
  return (char)(((bcd >> 4) & 0x0F) * 10 + (bcd & 0x0F));
}

void RewriteBbsMsg(int addr, char *txt )
{
  int x;

  for(x=0; x<22; x++)
  {
    Rom[addr+x] = 0x20; // space char
  }

  Rom[addr+12] = 0; // Plant terminator

  for(x=0; x<12; x++)
  {
    if(*txt == 0) break;
    Rom[addr+x] = *txt++;
  }

  if(*txt == 0) return;

  for(x=13; x<22; x++)
  {
    if(*txt == 0) break;
    Rom[addr+x] = *txt++;
  }
}

unsigned int GetNextBbsMsgNo(void)
{
  int msg = 0;
  msg = Ram[bbsmsg_address] + Ram[bbsmsg_address+1] * 256;
  return msg;
}

bool consolePeek(void)
{
  bool retval = false;

  // If tnc is not ready to accept more characters return false 
  if((siob.registers[5] & 0x02) == 0) return false;

  if(tty[0].con_mode)
  {
    if( tty_peek(&tty[0])) retval = true;
  }

  if(tty[1].con_mode)
  {
    if( tty_peek(&tty[1])) retval = true;
  }

  return retval;
}

int consoleInput(void)
{
  int retval = 0xff;
  if(tty[0].con_mode)
  {
    if( !tty_getch(&tty[0], &retval) )
    {
      retval = 0xff;
      if (tty[1].con_mode)
      {
        if( !tty_getch(&tty[1], &retval) )
        {
          retval = 0xff;
        }
      }
    }
  }
  else if (tty[1].con_mode)
  {
      if( !tty_getch(&tty[1], &retval) )
      {
        retval= 0xff;
      }
  }

  /* some key translations are they needed? */
  if(retval == 0x0a) retval=0x0d;
  if(retval == 0x7f) retval=0x08;
  /* if a return char from console clear new bbs msgs */
  if(retval == 0x0d) newMsg = false;

  return retval;
}

void consoleOutput(uint8_t c)
{
  if (tty[0].con_mode) tty_write_char(&tty[0], c);
  if (tty[1].con_mode) tty_write_char(&tty[1], c);
}

void consoleOutputStr(uint8_t const *str)
{
  if (tty[0].con_mode) tty_write_str(&tty[0], str );
  if (tty[1].con_mode) tty_write_str(&tty[1], str );
}

void Con_Serial_ParmChange(void)
{
  uint baud_rate;
  uint data_bits;
  uint stop_bits;
  uart_parity_t parity;

  /* If tty1 the uart port is not in console mode do nothing !*/
  if(!tty[1].con_mode) return;

  baud_rate = tnc[0].conSpeed;

  /* translate lower bauds to high ones that tnc normally did not support */
  if( baud_rate == 75 ) baud_rate = 115200;
  if( baud_rate == 120 ) baud_rate = 38400; /* 110 baud divisors calcs to 120! */

  /* determine data_bits */
  switch( siob.registers[3] >> 6)
  {
    case 0:
      data_bits = 5;
      break;

    case 1:
      data_bits = 7;
      break;

    case 2:
      data_bits = 6;
      break;

    case 3:
      data_bits = 8;
      break;

    default:
      printf("TNCEMU:Con_Serial_ParmChange invalid data_bits!\n");
      data_bits = 8;
      break;
  }

  /* determine stop_bits */
  switch( (siob.registers[4] >> 2) & 0x03 )
  {
    case 0:
      printf("TNCEMU:Con_Serial_ParmChange invalid stop_bits!\n");
      stop_bits = 1;
      break;

    case 1:
      stop_bits = 1;
      break;

    case 2:
      printf("TNCEMU:Con_Serial_ParmChange invalid stop_bits!\n");
      stop_bits = 1;
      break;

    case 3:
      stop_bits = 2;
      break;

    default:
      stop_bits = 2;
      break;
  }

  printf("\nTTL Serial= %d, %d, %d, ", baud_rate,data_bits,stop_bits);

  /* Determine Parity */
  if( siob.registers[4] & 0x01 )
  {
    if( siob.registers[4] & 0x02 )
    {
      parity = UART_PARITY_EVEN;
      printf("Even\n");
    }
    else
    {
      parity = UART_PARITY_ODD;
      printf("Odd\n");
    }
  }
  else 
  {
    parity = UART_PARITY_NONE;
    printf("None\n");
  }
  uint result_baud = uart_init(uart0, baud_rate);
  uart_set_format(uart0, data_bits, stop_bits, parity);
}