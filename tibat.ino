/*
 * t402code.c
 *
 * Created: 01/10/2019 15:22:39
 * Author : Wilenzo
 */ 

#define F_CPU 10000000UL // 20 MHz clock speed / 2 prescaler

#include <avr/io.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

//Pins
#define UAMP  1
#define BUTN  2
#define AOUT  3
#define NMIC  6
#define PMIC  7

//Port
#define FALLING       0x03
#define INPUT_DISABLE 0x04
#define PULLUPEN      0x08

//Analog Comparator
#define HYS_00        0x01 
#define HYS_10        0x03
#define HYS_25        0x05
#define HYS_50        0x07
#define AC_OFF        0x00
#define NEG_EDGE      0x20

//Global variables
volatile uint8_t cntAC = 0;  //AC call counter, for frequency division
volatile uint8_t toAC  = 1;  //time-out flag for AC, silences output when active
volatile uint8_t fDiv  = 16; //Division factor for frequency of microphone 32kHz->2kHz after reset, must never be lower than 2!
volatile uint8_t Zzz   = 0;  //Sleep trigger

//Every low to high transition a counter is increased, except when a previous time-out is detected, then only the time-out is cleared.
ISR(AC0_AC_vect){
    AC0_STATUS |= 0x01;
    if (!(toAC)) ++cntAC; else toAC = 0; 
    TCB0_CNT = 0;
}

//Timeout! Signal used to disable sound generation is setting cntAC to max.
ISR(TCB0_INT_vect){
    TCB0_INTFLAGS |= 0x01;
    toAC = 1;
    cntAC = 0xFF;
}

//Wake-up from deep sleep or button pressed. Turn on RTC if it was turned off, else change division factor of frequency
ISR(PORTA_PORT_vect){
    if (RTC_CTRLA & 0x01){
        // Already running: Change division factor, selectable are: 2, 4, 8, 16, 32, 64 and 128
        while(RTC_STATUS & 0x02);
        RTC_CNT = 0;
        fDiv <<= 1; 
        if (fDiv == 0) fDiv = 2; 
        if (fDiv > 4) TCB0_CCMP = 1000; else TCB0_CCMP = 2000; //Select 10kHz or 5kHz lower detection limit (for divisions 2 and 4).
    } else {
        // Wake from sleep
        sleep_disable();
        Zzz = 0;
        while(RTC_STATUS & 0x01);
        RTC_CTRLA |= 0x01;
    }

    //Wait a while, then wait for release, wait more: poor man's debounce function
    _delay_loop_2(0);
    _delay_loop_2(0);    
    _delay_loop_2(0);    
    while(!(PORTA_IN & 1<<BUTN));
    _delay_loop_2(0);
        
    //Turn on microphone amplifier, clear interrupt flags
    PORTA_OUTSET = (1<<UAMP);
    PORTA_INTFLAGS = 0xFF;
    RTC_INTFLAGS |= 0x02;
}

//Auto power off after 5 minutes of button inactivity
ISR(RTC_CNT_vect){
    RTC_INTFLAGS |= 0x02;
    Zzz = 1;
}

//Init I/O, timers, interrupts etc...
void setup(void) {
    	//Set clock to 10MHz, should work from 2.7V and up.
      _PROTECTED_WRITE(CLKCTRL_MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);
      _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, CLKCTRL_PEN_bm | CLKCTRL_PDIV_2X_gc);

      //CPU: Round robin interrupt handling
      CPUINT_CTRLA = CPUINT_LVL0RR_bm;

      //Port configuration
      PORTA_DIR = (1<<AOUT)|(1<<UAMP);
      PORTA_OUT = 0;
      PORTA_PIN1CTRL = INPUT_DISABLE;
      PORTA_PIN2CTRL = PULLUPEN | FALLING;
      PORTA_PIN3CTRL = INPUT_DISABLE;
      PORTA_PIN6CTRL = INPUT_DISABLE;
      PORTA_PIN7CTRL = INPUT_DISABLE;
      
      //Analog comparator
      AC0_CTRLA    = HYS_00 | NEG_EDGE; //0mV hysteresis, negative edge interrupt, on.
      AC0_MUXCTRLA = 0;                 //Normal pos/neg IO pins are used
      AC0_INTCTRL  = 0x01;              //Negative edge interrupt on

      //Timer B (time-out check)
      TCB0_CCMP    = 1000; // 10000Hz interrupts, for ignoring lower frequencies, higher number => lower frequency
      TCB0_CTRLA   = 0x01; // Counter on, peripheral speed (=> 10MHz)
      TCB0_INTCTRL = 0x01; // Interrupt on

      //RTC, used for auto power off 
      while(RTC_STATUS & 0x08);
      RTC_CMP      = 300;  // About 5 minutes of listening time before the unit is turned off 
      while(RTC_STATUS & 0x02);
      RTC_CNT      = 0;
      RTC_INTCTRL  = 0x02; // Compare interrupt
      RTC_CLKSEL   = 0x01; // Set clock rate to 1024Hz
      while(RTC_STATUS & 0x01);
      RTC_CTRLA    = 0x51; // Turn RTC on, with 1024 prescaling factor.

      //Sleep to power down mode (deep)
      SLPCTRL_CTRLA = 0x04;

      //Enable interrupts
      sei();
}

void loop(void)
{
    //Check sleepiness
    if (Zzz) {
        //beep
        PORTA_DIRSET = (1<<AOUT);
        for (uint16_t a=50; a<255; ++a){
          _delay_loop_2(a<<5);
          PORTA_OUTTGL = (1<<AOUT);
        }

        //Turn off all outputs, RTC too and reset RTC
        PORTA_OUT = 0;
        while(RTC_STATUS & 0x01);
        RTC_CTRLA &= 0xFE; 
        while(RTC_STATUS & 0x02);
        RTC_CNT   = 0;

        //Sleep (and unsleep after wake to be sure)
        sleep_enable();
        sleep_cpu();
        sleep_disable();
    }

    //Auto turn off underflow protection
    if(RTC_CNT > 300) RTC_CNT = 300;

    //Check if output pin should be toggled or be turned off. This produces "sound".
    if (fDiv < 2) fDiv = 2;
    if (cntAC >= (fDiv>>1)){
        if (cntAC == 0xFF){
            //Off
            PORTA_DIRCLR = (1<<AOUT);
            PORTA_OUTCLR = (1<<AOUT);
        } else {
            //Toggle
            cntAC -= (fDiv>>1);
            PORTA_DIRSET = (1<<AOUT);
            PORTA_OUTTGL = (1<<AOUT);
        }
    }
}

