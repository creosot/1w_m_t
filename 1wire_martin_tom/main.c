/*
 * 1wire_martin_tom.c
 *
/* 
   DS18x20 Demo-Program
   
   V 0.9.2, 2/2011
   
   by Martin Thomas <eversmith@heizung-thomas.de>
   http://www.siwawi.arubi.uni-kl.de/avr-projects
    
   features:
   - DS18X20 and 1-Wire code is based on an example from Peter 
     Dannegger
   - uses Peter Fleury's uart-library which is very portable 
   - additional functions not found in the  uart-lib available
     in uart.h/.c
   - CRC-check based on code from Colin O'Flynn
   - accesses multiple sensors on multiple 1-Wire busses
   - example how to address every sensor in the bus by ROM-code
   - independant of system-timers (more portable) but some
     (very short) delays used
   - avr-libc's stdint.h in use 
   - no central include-file, parts of the code can be used as
     "library" easily
   - verbose output example
   - one-wire-bus can be changed at runtime if OW_ONE_BUS
     is not defined in onewire.h. There are still minor timing 
     issues when using the dynamic bus-mode
   - example on read/write of DS18x20 internal EEPROM
*/


/* This example has been tested with ATmega324P at 3.6864MHz and 16Mhz */
//#define F_CPU 16000000UL

#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 10606UL
#error "please update to avrlibc 1.6.6 or newer, not tested with older versions"
#endif


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>

#include "uart.h"
#include "uart_addon.h"
#include "onewire.h"
#include "ds18x20.h"

#define BAUD 19200
// 2400 for 1MHz and 2MHz internal RC
// #define BAUD 2400

#define MAXSENSORS 5

#define NEWLINESTR "\r\n"


int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
    }
}

