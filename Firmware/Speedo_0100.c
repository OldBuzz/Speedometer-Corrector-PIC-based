/*-----------------------------------------------------------------------------

 Speedo_0100.c

 Speedometer Corrector
 Originally for Jeep Wrangler TJ (1997-2006)

 MCU: PIC 18F1320
 Using this particular PIC for its two 16-bit timers, and the ability to clock
 those timers at an appropriate rate without using a crystal.

 Input pin: RB0/INT0 (pin 8) (has a Schmitt Trigger input for INT0)
 Output pin: RB1 (pin 9)
 Switch pins: RA0-RA4 and RB5-RB7 (pins 1, 2, 6, 7, 3, 11, 12, 13)
 Inc/Dec pin: RB2 (pin 17)

 Version:
  01.00  2025/07/30  First "real" version.

 Copyright 2025 Dale A. Keller (OldBuzzard)

 This program is free software: you can redistribute it and/or modify it under
 the terms of the GNU General Public License as published by the Free Software
 Foundation, either version 3 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along with
 this program. If not, see <https://www.gnu.org/licenses/>. 

-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 #pragma config statements should precede project file includes.
 Use project enums instead of #define for ON and OFF.
-----------------------------------------------------------------------------*/

// CONFIG1H
#pragma config OSC = INTIO2     // Oscillator Selection bits (Internal RC oscillator, port function on RA6 and port function on RA7)
#pragma config FSCM = OFF       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bit (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 1024     // Watchdog Timer Postscale Select bits (1:1024)

// CONFIG3H
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RA5 input pin enabled, MCLR disabled)

// CONFIG4L
#pragma config STVR = ON        // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Low-Voltage ICSP Enable bit (Low-Voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (00200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (00200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (00200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

/*-----------------------------------------------------------------------------
 Includes and Defines
-----------------------------------------------------------------------------*/
#include <xc.h>                 // Pulls in the device header for your PIC.
#include <stdbool.h>            // true & false defined here

#define _XTAL_FREQ 2000000      // 2 MHz

/*-----------------------------------------------------------------------------
 Global variables
-----------------------------------------------------------------------------*/
volatile unsigned int   inTicks;        // 16-bit
volatile unsigned int   tmrTicks;       // 16-bit
volatile unsigned int   calFactor;      // 16-bit
volatile _Bool          inInt;          // flag showing input int occurred
volatile _Bool          T1overflow;     // T1 overflow semaphore
volatile _Bool          T1overflow2;    // T1 second overflow semaphore

/*-----------------------------------------------------------------------------
 Initialization
-----------------------------------------------------------------------------*/
void init(void) {
    unsigned int iAmount = 0;   // amount to increase or decrease

    // set up internal oscillator prescaler
    OSCCON = 0b01011110;    // IDLEN=0, IRCF=2MHz, OSTS=1, IOFS=1, SCS=10
    
    // disable analog input (disabled at reset, but ports are analog - dumb)
    ADCON0bits.ADON = 0;    // disable A/D convertor
    ADCON1 = 0b01111111;    // set RA0, RA1, RA2, RA3 and RB0, RB1, RB4 to
                            //   digital, not analog

    // RA0-RA7 input (no pull-ups available)
    TRISA = 0b11111111;

    // RB0 input, RB1 output, RB2-7 input (with weak pull-ups)
    TRISB = 0b11111101;     // RB1 output, all others input
    RBPU = 0;               // RBPU (INTCON2 bit 7) enable all B pull-ups

    // set up Timer1
    T1CON = 0b10110000;     // 16-bit R/W, 1:8 prescale, Fosc/4, disabled
    PIE1bits.TMR1IE = 1;    // enable Timer1 interrupt (overflow)
    PIR1bits.TMR1IF = 0;    // clear Timer1 interrupt flag
    T1overflow = true;      // T1 overflow semaphore
    T1overflow2 = true;     // T1 second overflow semaphore

    // set up Timer3
    T3CON = 0b10110000;         // 16-bit R/W, 1:8 prescale, Fosc/4, disabled
    PIE2bits.TMR3IE = 1;        // enable Timer3 interrupt
    PIR2bits.TMR3IF = 0;        // clear Timer3 interrupt flag

    // set up INT0 on RB0
    INTCONbits.INT0IE = 1;      // enable INT0 on RB0 (always high priority)
    INTCON2bits.INTEDG0 = 0;    // set INT0 to falling edge
    INTCONbits.INT0IF = 0;      // clear INT0 flag

    // set up priority interrupts
    RCONbits.IPEN = 1;          // enable priority interrupts
                                // INT0 interrupts are always high priority
    IPR1bits.TMR1IP = 0;        // set Timer1 interrupt to low priority
    IPR2bits.TMR3IP = 0;        // set Timer3 interrupt to low priority

    // read switches (5 switches on port A and 3 switches on port B)
    //   this 8-bit number represents 2 * percentage (0-99%) so we can get
    //   1/2 percent increments
    iAmount = (PORTA & 0b00011111) | (PORTB & 0b11100000);
    if(iAmount > 199u) iAmount = 199u;  // set a limit (99.5%)

    // read Inc/Dec jumper, set calibration factor
    if(PORTBbits.RB2) {             // if RB2 is high, increase
        calFactor = 200u - iAmount; // smaller number = higher frequency
    } else {                        // else decrease
        calFactor = 200u + iAmount; // larger number = lower frequency
    }

    // clear "input int occurred" flag
    inInt = false;

    // init tmrTicks for Timer 3 (full 16-bit count, ~1 rollover per second,
    //   ~2 seconds per full cycle, ~1/8 MPH)
    tmrTicks = 0;

    WDTCONbits.SWDTEN = 1;          // enable watchdog timer

    WRITETIMER1(0);                 // clear timer 1
    T1CONbits.TMR1ON = 1;           // start Timer1

    WRITETIMER3(0);                 // clear timer 3
    T3CONbits.TMR3ON = 1;           // start Timer3

    INTCONbits.GIEH = 1;            // enable high global interrupts
    INTCONbits.GIEL = 1;            // enable low global interrupts
}

/*-----------------------------------------------------------------------------
 Interrupt (HIGH priority)

 Interrupt from RB0/INT0 falling edge (in pin 8 from speed sensor, full wave).
-----------------------------------------------------------------------------*/
void __interrupt(high_priority) highISR(void) {
    // INT0 occurs on the falling edge of the speed squarewave input
    if (INTCONbits.INT0IF) {        // if interrupt flag is set:
        if (!T1overflow2) {         // if T1 has not overflowed,
            inTicks = READTIMER1(); // get Timer 1 count (full input cycle)
            inInt = true;           // set flag that input int occurred
            T1overflow = false;     // clear semaphore
        }
        WRITETIMER1(0);             // clear Timer 1
        T1overflow2 = false;        // clear second semaphore
        INTCONbits.INT0IF = 0;      // clear RB0/INT0 interrupt flag
    }
}

/*-----------------------------------------------------------------------------
 Interrupt (LOW priority)

 Interrupt from Timer1 overflow (vehicle very slow or stopped).
 Interrupt from Timer3 overflow (an output half wave has completed).
-----------------------------------------------------------------------------*/
void __interrupt(low_priority) lowISR(void) {
    // T1 overflows if vehicle is extremely slow or stopped
    if (PIR1bits.TMR1IF) {          // if T1 overflow interrupt flag is set:
        T1overflow = true;          // set semaphore
        T1overflow2 = true;         // set second semaphore
        PIR1bits.TMR1IF = 0;        // clear Timer 1 overflow interrupt flag
    }
    // T3 overflows when an output half cycle has completed
    if (PIR2bits.TMR3IF) {          // if T3 overflow interrupt flag is set:
        if (!T1overflow)            // if T1 has not overflowed,
            LATBbits.LATB1 = ~LATBbits.LATB1; // toggle output pin (RB1 pin 9)
        WRITETIMER3(tmrTicks);      // write value to Timer 3
        PIR2bits.TMR3IF = 0;        // clear Timer 3 overflow interrupt flag
    }
}

/*-----------------------------------------------------------------------------
 Main
-----------------------------------------------------------------------------*/
void main(void) {
    unsigned int i, outTicks;

    init(); // do initialization
    
    while(1) {
        if (inInt) {
            // calculate half-cycle time, and scale it
            INTCONbits.GIEH = 0;        // disable high global interrupts
            i = inTicks;                // quick copy
            INTCONbits.GIEH = 1;        // enable high global interrupts
            outTicks = (unsigned int)(((unsigned long)(i >> 1u) *
                       (unsigned long)calFactor) / 200u);

            // calculate next half-cycle timer value
            i = 65535u - outTicks;
            INTCONbits.GIEL = 0;        // disable low global interrupts
            tmrTicks = i;               // quick copy
            INTCONbits.GIEL = 1;        // enable low global interrupts

            // clear "input int occurred" flag
            inInt = false;
        }

        CLRWDT();                       // kick the dog
    }
    
    return; // never happens
}
