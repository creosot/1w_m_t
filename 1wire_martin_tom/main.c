// /*
//  * 1wire_martin_tom.c
//  *
// /* 
//    DS18x20 Demo-Program
//    
//    V 0.9.2, 2/2011
//    
//    by Martin Thomas <eversmith@heizung-thomas.de>
//    http://www.siwawi.arubi.uni-kl.de/avr-projects
//     
//    features:
//    - DS18X20 and 1-Wire code is based on an example from Peter 
//      Dannegger
//    - uses Peter Fleury's uart-library which is very portable 
//    - additional functions not found in the  uart-lib available
//      in uart.h/.c
//    - CRC-check based on code from Colin O'Flynn
//    - accesses multiple sensors on multiple 1-Wire busses
//    - example how to address every sensor in the bus by ROM-code
//    - independant of system-timers (more portable) but some
//      (very short) delays used
//    - avr-libc's stdint.h in use 
//    - no central include-file, parts of the code can be used as
//      "library" easily
//    - verbose output example
//    - one-wire-bus can be changed at runtime if OW_ONE_BUS
//      is not defined in onewire.h. There are still minor timing 
//      issues when using the dynamic bus-mode
//    - example on read/write of DS18x20 internal EEPROM
// */


/* This example has been tested with ATmega324P at 3.6864MHz and 16Mhz */
//#define F_CPU 16000000UL
#define CTC_MATCH_OVERFLOW ((F_CPU / 1000) / 8)
#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 10606UL
#error "please update to avrlibc 1.6.6 or newer, not tested with older versions"
#endif

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
/*#include <avr/eeprom.h>*/
#include <avr/wdt.h>
#include <util/delay.h>
/*#include <string.h>*/
/*#include <stdint.h>*/
#include <util/atomic.h>
// #include "uart.h"
// #include "uart_addon.h"
#include "onewire.h"
#include "ds18x20.h"

#define BAUD 9600

#define MAXSENSORS 1
#define NEWLINESTR "\r\n"

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
#define MAXNUMBER 17
uint8_t number[MAXNUMBER] = {		//portb
	0b11010111,	//0
	0b00010001,	//1
	0b01100111,	//2
	0b01110101,	//3
	0b10110001,	//4
	0b11110100,	//5
	0b11110110,	//6
	0b01010001,	//7
	0b11110111,	//8
	0b11110101,	//9
	0b00000000,	//10 clear 0 in high digit
	0b10000000,	//11 - defis heat	
	0b00100000,	//12 - minus
	0b01000000,	//13 - defis up
	0b00000100,	//14 - defis down
	0b11100110,	//15 - E
	0b00100010 	//16 - r
};

/* return values */
// #define DS18X20_OK                0x00
// #define DS18X20_ERROR             0x01
// #define DS18X20_START_FAIL        0x02
// #define DS18X20_ERROR_CRC         0x03
#define TESTTIME 1000
#define NO_ERROR 0
#define ERROR_NO_SENSOR_START 1
#define ERROR_NO_SENSOR_AFTER_START 2
#define ERROR_TEMP_OVERHEAT 3
#define ERROR_WDT 4
#define CLEAR 10
#define DEFIS_HEAT 11
#define MINUS 12
#define DEFIS_UP 13
#define DEFIS_DOWN 14
#define E 15
#define r 16
#define HEAT_ON 1
#define HEAT_OFF 0
#define MODE_ERROR 0
#define MODE_NORMAL 1
#define MODE_HIGH 2
#define MODE_LOW 3
#define BUTTON_DOWN 0
#define BUTTON_UP 1
#define BUTTON1 0
#define BUTTON2 1
#define ALARM_HEAT 300
#define RELE_ON PORTD |= (1 << PORTD5)
#define RELE_OFF PORTD &= ~(1 << PORTD5)
uint8_t digit[4] = {0b11111011, 0b11111101, 0b11111110, 0b11110111};	//1,2,3,4
static uint8_t temperature[4] = {0,0,0,0};
static volatile uint8_t counterDigit = 0;
static int16_t decicelsius = 230;
static int16_t limitHigh = 260;
static int16_t limitLow = 250;
static volatile uint8_t mode = 0;
static volatile uint8_t heatStatus = 0;
uint8_t buttonPrefState[3] = {(1<<PORTC0), (1<<PORTC1), (1<<PORTC2)}; //PC0,PC1,PC2
uint8_t buttonMask[3] = {(1<<PORTC0), (1<<PORTC1), (1<<PORTC2)}; //PC0,PC1,PC2
uint8_t buttonDebounceState[3] = {0,0,0};
uint32_t lastDebounceTime[3] = {0,0,0};
uint8_t buttonStateON[3] = {0,0,0};
uint8_t buttonStateOFF[3] = {0,0,0};
static volatile unsigned long timer1_millis = 0;
static long milliseconds_since = 0;
static volatile uint8_t err = NO_ERROR;
static volatile uint8_t temp = 0;


// #if DS18X20_EEPROMSUPPORT
// static void th_tl_dump(uint8_t *sp);
// static void eeprom_test(void);
// #endif /* DS18X20_EEPROMSUPPORT */

static uint8_t search_sensors(void);
/*static void uart_put_temp(int16_t decicelsius);*/
unsigned long millis ();
void initDisplay();
void initTimer1();
void initTimer2();
void initButton();
void initRele();
void initGenerator();
void initWDT();
void disableWDT();
void setNumber(uint8_t num, uint8_t dig);
void convertTempToDigit(uint16_t t, uint8_t mode);
void getButtonState(uint8_t but);
void handleError(uint8_t e);

ISR (TIMER1_COMPA_vect){
	timer1_millis++;
}

ISR(TIMER2_COMP_vect){
	counterDigit &= 0b00000011;
	temp += 1;
	if (!counterDigit){
		if (heatStatus == HEAT_ON && mode == MODE_NORMAL){
			if (temp & 0b10000000) temperature[0] = DEFIS_HEAT;
			else temperature[0] = CLEAR;
		}
	}
	setNumber(temperature[counterDigit], counterDigit);
	counterDigit += 1;
}

void readTempForOnlyDS18b20(){
	uint8_t i, err1, err2;
	for (i = 10; i; i--){
		wdt_reset();
		err1 = DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL); //start measure
		_delay_ms( DS18B20_TCONV_12BIT ); //delay for measure
		err2 = DS18X20_read_decicelsius_single( gSensorIDs[0][0], &decicelsius );
		if (err1 == DS18X20_OK && err2 == DS18X20_OK){
			return;
		}
		_delay_ms(200);
	}
	err = ERROR_NO_SENSOR_AFTER_START;
}


int main(void)
{
	uint8_t nSensors;
			
/*	uart_init((UART_BAUD_SELECT((BAUD),F_CPU)));*/
		
	#ifndef OW_ONE_BUS
		ow_set_bus(&PIND,&PORTD,&DDRD,PORTD7);
	#endif
		
 	sei();
		
	nSensors = search_sensors();
	if (nSensors == 0){
		err = ERROR_NO_SENSOR_START;
	}
// 	uart_put_int( (int)nSensors );
// 	uart_puts_P( " DS18X20 Sensor(s) available:" NEWLINESTR );
// 		
// 	#if DS18X20_VERBOSE
// 		for (i = 0; i < nSensors; i++ ) {
// 			uart_puts_P("# in Bus :");
// 			uart_put_int( (int)i + 1);
// 			uart_puts_P(" : ");
// 			DS18X20_show_id_uart( &gSensorIDs[i][0], OW_ROMCODE_SIZE );
// 			uart_puts_P( NEWLINESTR );
// 		}
// 	#endif
// 		
// 	for ( i = 0; i < nSensors; i++ ) {
// 		uart_puts_P( "Sensor# " );
// 		uart_put_int( (int)i+1 );
// 		uart_puts_P( " is a " );
// 		if ( gSensorIDs[i][0] == DS18S20_FAMILY_CODE ) {
// 			uart_puts_P( "DS18S20/DS1820" );
// 		} else if ( gSensorIDs[i][0] == DS1822_FAMILY_CODE ) {
// 			uart_puts_P( "DS1822" );
// 		}
// 		else {
// 			uart_puts_P( "DS18B20" );
// 		}
// 		uart_puts_P( " which is " );
// 		if ( DS18X20_get_power_status( &gSensorIDs[i][0] ) == DS18X20_POWER_PARASITE ) {
// 			uart_puts_P( "parasite" );
// 			} else {
// 			uart_puts_P( "externally" );
// 		}
// 		uart_puts_P( " powered" NEWLINESTR );
// 	}	
		
// 	#if DS18X20_EEPROMSUPPORT
// 		if ( nSensors > 0 ) {
// 			eeprom_test();
// 		}
// 	#endif	
				
	initDisplay();
	initTimer2();
	initTimer1();
	initButton();
	initRele();
	mode = MODE_NORMAL;
	_delay_ms(1000);
	
	if (MCUCSR & (1 << WDRF)){
		MCUCSR &= ~(1 << WDRF);
		disableWDT();
		err = ERROR_WDT;
	}
	else{
		initWDT();
	}
/*	_delay_ms(1650);*/
		
    while (1) {		
		wdt_reset();	
 		handleError(err);	
		getButtonState(BUTTON1);
 		if (buttonStateON[BUTTON1]){
 			buttonStateON[BUTTON1] = 0;
 			mode += 1;
 			if (mode == 4) mode = MODE_NORMAL;
		}
 		switch (mode){
 		case MODE_NORMAL:
			if (millis() - milliseconds_since > 999){
				milliseconds_since = millis();
				readTempForOnlyDS18b20();
 				convertTempToDigit(decicelsius, mode);
 				wdt_reset();
 			}
			if (decicelsius > ALARM_HEAT){
				err = ERROR_TEMP_OVERHEAT;
			}
			if (decicelsius < limitLow){ //heat on
				heatStatus = HEAT_ON;
				RELE_ON;
			}
			if (decicelsius > limitHigh){ //cool on
				heatStatus = HEAT_OFF;
				RELE_OFF;
			}
			if (decicelsius > limitLow && decicelsius < limitHigh){ //cool on
				
			}
 			break;		
 		case MODE_HIGH:
 			convertTempToDigit(limitHigh, mode);
			RELE_OFF;
 			wdt_reset();
 			break;		
 		case MODE_LOW:
 			convertTempToDigit(limitLow, mode);
			RELE_OFF;
 			wdt_reset();
 			break;	
 		default:
 			break;
 		}
    }
}


void handleError(uint8_t err){
	while(err){
		RELE_OFF;
		disableWDT();
		mode = MODE_ERROR;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			temperature[0] = E;
			temperature[1] = r;
			temperature[2] = r;
			temperature[3] = err;
		}
		_delay_ms(2000);
	}
}

// void demo_sensor(){
// 	uint8_t i;
// 	uint8_t nSensors = search_sensors();
// 	if ( nSensors == 1 ) {
// 		uart_puts_P( NEWLINESTR "There is only one sensor "
// 		"-> Demo of \"DS18X20_read_decicelsius_single\":" NEWLINESTR );
// 		i = gSensorIDs[0][0]; // family-code for conversion-routine
// 		/*DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );*/
// 		DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL ); //start measure
// 		_delay_ms( DS18B20_TCONV_12BIT ); //delay for measure
// 		DS18X20_read_decicelsius_single( i, &decicelsius );
// 		uart_puts_P( NEWLINESTR );
// 		uart_put_temp( decicelsius );
// 		uart_puts_P( NEWLINESTR );
// 	}
// }

static uint8_t search_sensors(void){
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
/*	uart_puts_P( NEWLINESTR "Scanning Bus for DS18X20" NEWLINESTR );*/
	
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
/*			uart_puts_P( "No Sensor found" NEWLINESTR );*/
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
/*			uart_puts_P( "Bus Error" NEWLINESTR );*/
			break;
		}
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
		gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}	
	return nSensors;
}

// static void uart_put_temp(int16_t decicelsius)
// {
// 	char s[10];
// 
// 	uart_put_int( decicelsius );
// 	uart_puts_P(" deci°C, ");
// 	DS18X20_format_from_decicelsius( decicelsius, s, 10 );
// 	uart_puts( s );
// 	uart_puts_P(" °C");
// }
// 
// static void th_tl_dump(uint8_t *sp)
// {
// 	DS18X20_read_scratchpad( &gSensorIDs[0][0], sp, DS18X20_SP_SIZE );
// 	uart_puts_P( "TH/TL in scratchpad of sensor 1 now : " );
// 	uart_put_int( sp[DS18X20_TH_REG] );
// 	uart_puts_P( " / " );
// 	uart_put_int( sp[DS18X20_TL_REG] );
// 	uart_puts_P( NEWLINESTR );
// }
// 
// static void eeprom_test(void)
// {
// 	uint8_t sp[DS18X20_SP_SIZE], th, tl;
// 	
// 	uart_puts_P( NEWLINESTR "DS18x20 EEPROM support test for first sensor" NEWLINESTR );
// 	// DS18X20_eeprom_to_scratchpad(&gSensorIDs[0][0]); // already done at power-on
// 	th_tl_dump( sp );
// 	th = sp[DS18X20_TH_REG];
// 	tl = sp[DS18X20_TL_REG];
// 	tl++;
// 	th++;
// 	DS18X20_write_scratchpad( &gSensorIDs[0][0], th, tl, DS18B20_12_BIT );
// 	uart_puts_P( "TH+1 and TL+1 written to scratchpad" NEWLINESTR );
// 	th_tl_dump( sp );
// 	DS18X20_scratchpad_to_eeprom( DS18X20_POWER_PARASITE, &gSensorIDs[0][0] );
// 	uart_puts_P( "scratchpad copied to DS18x20 EEPROM" NEWLINESTR );
// 	DS18X20_write_scratchpad( &gSensorIDs[0][0], 0, 0, DS18B20_12_BIT );
// 	uart_puts_P( "TH and TL in scratchpad set to 0" NEWLINESTR );
// 	th_tl_dump( sp );
// 	DS18X20_eeprom_to_scratchpad(&gSensorIDs[0][0]);
// 	uart_puts_P( "DS18x20 EEPROM copied back to scratchpad" NEWLINESTR );
// 	DS18X20_read_scratchpad( &gSensorIDs[0][0], sp, DS18X20_SP_SIZE );
// 	if ( ( th == sp[DS18X20_TH_REG] ) && ( tl == sp[DS18X20_TL_REG] ) ) {
// 		uart_puts_P( "TH and TL verified" NEWLINESTR );
// 		} else {
// 		uart_puts_P( "verify failed" NEWLINESTR );
// 	}
// 	th_tl_dump( sp );
// }

unsigned long millis (){
	unsigned long millis_return;
	// Ensure this cannot be disrupted
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		millis_return = timer1_millis;
	}	
	return millis_return;
}

void initWDT(){
	WDTCR = (1 << WDCE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0); //reset 2s;
	WDTCR = (0 << WDCE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0); //reset 2s;
	wdt_reset();
}

void disableWDT(){
	wdt_reset();
	WDTCR |= (1 << WDCE) | (1 << WDE);
	WDTCR = 0x00;
}

void initRele(){
	RELE_OFF;
	DDRD |= 0b00100000; //rele-PD5; 
}

void initButton(){
	DDRC &= ~((1 << PORTC2) | (1 << PORTC1) | (1 << PORTC0)); //INPUT PC0-BUT+; PC1-BUT-; PC2-BUT_M	
	PORTC |= (1 << PORTC2) | (1 << PORTC1) | (1 << PORTC0); //pull up
}

void initGenerator(){
	DDRC |= (1 << PORTC4); //output
}

void initDisplay(){		//									   					     _A_
	DDRB |= 0b11111111; //PB0-B; PB1-E; PB2-D; PB3-DP; PB4-C; PB5-G; PB6-A; PB7-F  F|_G_|B
	DDRD |= 0b00001111; //dig1-PD2; dig2-PD1; dig3-PD0; dig4-PD4				   E|_D_|C													  
}

void initTimer1(){
	TCNT1 = 0;
	OCR1AH = (uint8_t)(CTC_MATCH_OVERFLOW >> 8);
	OCR1AL = (uint8_t)CTC_MATCH_OVERFLOW;
	TCCR1A = 0;
	TCCR1A |= (0 << WGM11) | (0 << WGM10); //mode CTC
	TCCR1B |= (0 << WGM13) | (1 << WGM12) |(0 << CS12) | (1 << CS11) | (0 << CS10); //mode CTC, clk/8
	TIMSK |= (1 << OCIE1A); //Output Compare A Match Interrupt Enable
}

void initTimer2(){
	TCNT2 = 0;
	OCR2 = 125; //4ms
	TCCR2 = 0;
	TCCR2 |= (1 << WGM21) | (0 << WGM20); //mode CTC
	TCCR2 |= (1 << CS22) | (1 << CS21) | (0 << CS20); //mode CTC, clk/256
	TIMSK |= (1 << OCIE2); //Output Compare A Match Interrupt Enable
}

void getButtonState(uint8_t but){
	uint8_t state = PINC & buttonMask[but];
	if (state ^ buttonDebounceState[but]){
		buttonDebounceState[but] = state;
		lastDebounceTime[but] = millis();
	}
	if ((millis() - lastDebounceTime[but] > 39)){
		if (state ^ buttonPrefState[but]){
			buttonPrefState[but] = state;
			buttonStateON[but] = (~state) & buttonMask[but];
			buttonStateOFF[but] = (~state) ^ buttonMask[but];
		}
	}
}

void setNumber(uint8_t num, uint8_t dig){
	if (num > MAXNUMBER-1 || dig>3) return;
	PORTD |= 0b00001111;
	PORTB = number[num];
	if (dig == 2) PORTB |= 0b00001000;
	PORTD &= digit[dig];
}

void convertTempToDigit(uint16_t t, uint8_t mode){
	uint16_t temp, d1, d2, d3, d4;
	temp = t;
	d4 = temp/1000;
	temp = temp - d4*1000;
	if (t & 0x8000) d4 = 11;
	d3 = temp/100;
	temp = temp - d3*100;
	d2 = temp/10;
	d1 = temp - d2*10;
	switch (mode){
		case MODE_NORMAL:
			if (d3 == 0 && d4 == 0) d3 = CLEAR;
			if (d4 == 0) d4 = CLEAR;
			break;
		case MODE_HIGH:
			d4 = DEFIS_UP;
			break;
		case MODE_LOW:
			d4 = DEFIS_DOWN;
			break;
		default:
			break;
	}
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		temperature[0] = (uint8_t)d4;
		temperature[1] = (uint8_t)d3;
		temperature[2] = (uint8_t)d2;
		temperature[3] = (uint8_t)d1;
	}
}