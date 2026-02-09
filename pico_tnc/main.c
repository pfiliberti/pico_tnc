/*
Copyright (c) 2021, JN1DFF
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
#include <math.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/structs/uart.h"
#include "pico/util/queue.h"
#include "hardware/watchdog.h"

#include "tnc.h"
#include "receive.h"
#include "send.h"
#include "ax25.h"

//#include "usb_input.h"
#include "usb_output.h"
#include "serial.h"
#include "tty.h"
#include "kiss.h"

#define TIME_10MS (10 * 1000)    // 10 ms = 10 * 1000 us

// greeting message
static const uint8_t greeting[] =
    "\r\nPico TNCEMU Emulated Z80 TNC Ver 0.72\r\n";

// Watchdog Timer Reset Message
static const uint8_t wdtfailmsg[] =
    "Watch Dog Timer Failure\n";

uint8_t Dip_Option; /* Holds dip switch option setting */

int Read_Dip_Switch()
{
    int retval = 0;
    if (!gpio_get(DIP_SWITCH_0)) retval += 1;
    if (!gpio_get(DIP_SWITCH_1)) retval += 2;
    if (!gpio_get(DIP_SWITCH_2)) retval += 4;
    return retval;
}


int main()
{
    uint32_t dip_check_time = tnc_time();

    stdio_init_all();

    #ifdef BUSY_PIN
    gpio_init(BUSY_PIN);
    gpio_set_dir(BUSY_PIN, true); // output
#endif

#define SMPS_PIN 23
#if 1
    gpio_init(SMPS_PIN);
    gpio_set_dir(SMPS_PIN, true); // output
    gpio_put(SMPS_PIN, 0);
#endif

    // Initialize the dip switch i/o pins
    gpio_init(DIP_SWITCH_0);
    // Set the pin as input
    gpio_set_dir(DIP_SWITCH_0, GPIO_IN);
    // Enable the internal pull-up resistor
    gpio_pull_up(DIP_SWITCH_0);

    gpio_init(DIP_SWITCH_1);
    // Set the pin as input
    gpio_set_dir(DIP_SWITCH_1, GPIO_IN);
    // Enable the internal pull-up resistor
    gpio_pull_up(DIP_SWITCH_1);

    gpio_init(DIP_SWITCH_2);
    // Set the pin as input
    gpio_set_dir(DIP_SWITCH_2, GPIO_IN);
    // Enable the internal pull-up resistor
    gpio_pull_up(DIP_SWITCH_2);

    /* Set dip option val */
    Dip_Option = Read_Dip_Switch();

    /* check psave input and if set go into program download mode */
    if( Dip_Option == BOOTLOAD) {
        reset_usb_boot(0, 0); // Enter USB bootloader mode
    }

    // create usb output queue
    usb_output_init();

    tty[0].kiss_mode = false; // default kiss off
    tty[1].kiss_mode = false;
    tty[0].con_mode = false; // default console mode off
    tty[1].con_mode = false;

    /* Configure ports based on Dip Option */
    switch (Dip_Option)
    {
        case CON_CON:
            tty[0].con_mode = true;
            tty[1].con_mode = true;
            break;

        case KIS_KIS:
            tty[0].kiss_mode = true;
            tty[1].kiss_mode = true;
            break;

        case KIS_CON:
            tty[0].kiss_mode = true;
            tty[1].con_mode = true;
            break;

        case CON_KIS:
            tty[0].con_mode = true;
            tty[1].kiss_mode = true;
            break;

        case CON_MSG:
            break;

        case MSG_CON:
            break;

        case SPARE:
            break;
    }

    /* If either port is on console wait for connect 10 seconds */
    if(tty[0].con_mode || tty[1].con_mode)
    {
        int usbWaitcnt = 500;  // Wait 5 seconds for USB CDC serial is connected
        while (!stdio_usb_connected()) {
            sleep_ms(10);
            if(--usbWaitcnt == 0)
                break;
        }
    }

    if (watchdog_caused_reboot()) consoleOutputStr(wdtfailmsg);

    // initialize tnc
    tnc_init();
    send_init();
    receive_init();
    serial_init();
    tty_init();     // should call after tnc_init()
    //bell202_init();

    // Write greeting to any consoles
    consoleOutputStr(greeting);

    //uint32_t ts = time_us_32();

    // set watchdog, timeout 1000 ms
    watchdog_enable(1000, true);

    // main loop
    while (1) {

        // update watchdog timer
        watchdog_update();

#if 0 /* now done in receive.c receive funtion*/
        // advance tnc time
        if (time_us_32() - ts >= TIME_10MS) {
            ++tnc_time;
            ts += TIME_10MS;
        }
#endif

        // Emulate the virtual TNC z80 code
        tnc_emulate();

        /* check if any ports are kiss and have input */
        // incoming KISS frame from serial
        int ch;
        if(tty[0].kiss_mode)
        {
            if( tty_getch(&tty[0], &ch) ) kiss_input(&tty[0], ch);
        }

        if(tty[1].kiss_mode)
        {
            if( tty_getch(&tty[1], &ch) ) kiss_input(&tty[1], ch);
        }

        if (tnc_time() - dip_check_time >= TIME_1SECOND) {
            dip_check_time = tnc_time();
            if(Read_Dip_Switch() != Dip_Option) /* dip switch change? */
            {
                while(1) /* loop forever so watchdog resets */
                {
                }
            }
        }

        // receive packet
        receive();
        // send packet
        send();
        // process uart I/O
        serial_input();
        serial_output();

        // calibrate off
//        calibrate();

#ifdef BUSY_PIN
//        gpio_put(BUSY_PIN, 0);
#endif

    // if not busy wait small time for next interrupt
    if(tnc[0].active_timeout == 0)
    {
        __wfi();
    }

#ifdef BUSY_PIN
//        gpio_put(BUSY_PIN, 1);
#endif

    }

    return 0;
}
