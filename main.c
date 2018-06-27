/*
 * main.c
 *
 *  Created on: 05.06.2018
 *      Author: Sebastian
 */


/*
 * This file is part of the MSP430 USCI I2C slave example.
 *
 * Copyright (C) 2012 Stefan Wendler <sw@kaltpost.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This firmware acts as an I2C slave on address 0x90(W)/0x91(R).
 * It receives a command / parameter pair from the I2C master and
 * sends out a response on a masters read request.
 *
 * The firmware knows two commands:
 *
 * CMD_SETLED: sets the build in LED (P1.0) of the launchpad depending
 * on the given parameter to HIGH (0x01) or LOW (0x00).
 *
 * CMD_GETBTN: copies the state of the build in push-button (P1.3) to
 * the response buffer and transmits it to the master on the next read
 * request.
 *
 * As a I2C master, the bus pirate could be used. To set the BP into
 * I2C mode use:
 *
 *    m4 3
 *
 * Bus-Piret I2C commands to
 *
 * - set LED HIGH (P1.0)
 *    [0x90 0x00 0x01
 *
 * - set LED LOW  (P1.0)
 *    [0x90 0x00 0x00
 *
 * - get BUTTON state (P1.3)
 *    [0x90 0x01 [0x91 r
 *
 * For a quick start to setup the I2C slave, the TI USCI I2C slave
 * code from slaa383 was used and slightly modified to work on
 * the MSP430G2553 and the MSP-GCC. For detail on leagal issues
 * regarding the TI_USCI_I2C_slave code see "ti-usci-i2c-slave-legal.txt".
 *
 * NOTE: 100k extrnal pull-ups are needed on SDA/SDC.
 */

#include <msp430.h>
#include <stdint.h>
//#include <legacymsp430.h>

#include "TI_USCI_I2C_slave.h"

/* callback for start condition */
void start_cb();

/* callback for bytes received */
void receive_cb(unsigned char receive);

/* callback to transmit bytes */
void transmit_cb(unsigned char volatile *receive);

/* Commands */
#define CMD_SETLED  0x00
#define CMD_GETBTN  0x01
#define CMD_UNKNOWN 0xFF
#define CMD_READ    0x02

/* Parameters */
#define PAR_LEDOFF  0x00
#define PAR_LEDON   0x01
#define PAR_UNKNOWN 0xFF

/* Responses */
#define RES_ERROR   0xFF

/* last command */
unsigned char cmd = CMD_UNKNOWN;

/* last parameter */
unsigned char par = PAR_UNKNOWN;

/* response to send out on read req. */
unsigned char res = RES_ERROR;

int8_t functionData[4];
int8_t * f_ptr;

typedef struct s_encoderData{
    uint8_t debouncing[2];
    uint8_t state;
} RotaryEncoderData;


void process_cmd(unsigned char cmd, unsigned char par)
{
    res = RES_ERROR;

    switch(cmd) {
    case CMD_SETLED:
        if(par == PAR_LEDON) {
            P1OUT |= BIT0;
        } else if(par == PAR_LEDOFF) {
            P1OUT &= ~BIT0;
        }
        break;
    case CMD_GETBTN:
        if((P1IN & BIT3)) {
            res = 0x01;
        } else {
            res = 0x00;
        }
        break;
    case CMD_READ:


    break;
    }
}

void start_cb()
{
    cmd = CMD_UNKNOWN;
    par = PAR_UNKNOWN;
    f_ptr = functionData;
}

void receive_cb(unsigned char receive)
{
    if(cmd == CMD_UNKNOWN) {

        cmd = receive;

        if(cmd == CMD_GETBTN) {
            process_cmd(cmd, PAR_UNKNOWN);
        }
    } else {
        par = receive;
        process_cmd(cmd, par);
    }
}

void transmit_cb(unsigned char volatile *byte)
{
    *byte = *f_ptr++;
    if(f_ptr - functionData > 4){
        functionData[0] = 0;
    }
}


/* returns change in encoder state (-1,0,1) */
int8_t read_encoder(uint8_t * oldstate, uint8_t newstate)
{
  static int8_t enc_states[] = {0,-1,1,0, 1,0,0,-1, 0,0,0,0, 0,0,0,0};
  //static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  //static uint8_t old_AB = 0;
  *oldstate <<= 2;                   //remember previous state
  *oldstate |= ( newstate & 0x03 );  //add current state
  return ( enc_states[( *oldstate & 0x0f )]);
}

// Timer A0 interrupt service routine
//#pragma INTERRUPT (Timer_A)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    static RotaryEncoderData oldstate;
    oldstate.debouncing[0] = (oldstate.debouncing[0] << 1) + ((P2IN & 0x01) ? 1 : 0);
    oldstate.debouncing[1] = (oldstate.debouncing[1] << 1) + ((P2IN & 0x02) ? 1 : 0);

    if(    (oldstate.debouncing[0] == 0 || oldstate.debouncing[0] == 0xFF)
        && (oldstate.debouncing[1] == 0 || oldstate.debouncing[1] == 0xFF))
    {
        //stable state
        uint8_t stablestate = (oldstate.debouncing[0] == 0xff ? 1 : 0) | (oldstate.debouncing[1] == 0xFF ? 2 : 0);
        int8_t inc = read_encoder(&oldstate.state, stablestate);
        functionData[0] += inc;
        if(functionData[0] >= 15){
            functionData[0] = 15;
        }
        if(functionData[0] < -15){
            functionData[0] = -15;
        }
        if(inc > 0){
            P1OUT ^= BIT0;
        }
    }
    TACCTL0 &= ~CCIFG;
}

int main(void)
{
    functionData[0]=3;
    functionData[1]=4;
    functionData[2]=5;
    functionData[3]=6;

    WDTCTL = WDTPW + WDTHOLD;                      // Stop WDT
    P1DIR |= BIT0;                                 // Set P1.0 to output direction
    P1DIR &= ~BIT3;                                // Set P1.3 to input  direction
    P1OUT &= ~BIT0;

    P2DIR = 0x00; //Input
    P2REN = 0xFF; //Resistor Enable
    P2OUT = 0xFF; //Set Pull-UP
    //P2IES = 0x01; //High to Low
    //P2IE =  0x01; //enable Interrupt

    TI_USCI_I2C_slaveinit(start_cb, transmit_cb, receive_cb, 0x48);
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL  = CALDCO_16MHZ;

    TACTL|=TASSEL_2+ID_3;   /* SMCLK, 8x divider */
    TACCR0=200;            /* will be 5 ms */
    TACCTL0|=CCIE;          /* enable TimerA interrupt */
    TACTL|=MC_1;            /* start timer in up-mode */

    __bis_SR_register(GIE);

    while(1)
    {
        __asm__("nop");
        _BIS_SR(LPM0_bits + GIE); // Enter LPM3 w/interrupt

    }
    return 0;
}
