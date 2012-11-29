#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>


/*
	ATMEGA8 fuses should be set HIGH=0xD9 LOW=0xE4
	(Internal RC Osc 8Mhz, Start-up time 6CK+64mS)

	PB0 - LED ANODES 9
	PB1 - LED ANODES 10
	PB2 - WIGGLE FOR ADC6
	PB3 - MUX CONTROL A
	PB4 - MUX CONTROL B
	PB5 - MUX CONTROL C, BUTTON
	PB6 - RESCUE XTAL
	PB7 - WIGGLE FOR ADC5

	PC0 - RED LED GROUP 1 CATHODES
	PC1 - GREEN LED GROUP 1 CATHODES
	PC2 - RED LED GROUP 2 CATHODES
	PC3 - GREEN LED GROUP 2 CATHODES
	PC4 - WIGGLE FOR ADC7
	PC5 - ANALOGUE INPUT FROM MUX 1 HANDLING A0-5 & D0-1
	PC6 - ANALOGUE INPUT FROM MUX 2 HANDLING D2-D9
	PC7 - ANALOGUE INPUT FROM MUX 3 HANDLING D10-D13

	PD0 - LED ANODES 1
	PD1 - LED ANODES 2
	PD2 - LED ANODES 3
	PD3 - LED ANODES 4
	PD4 - LED ANODES 5
	PD5 - LED ANODES 6
	PD6 - LED ANODES 7
	PD7 - LED ANODES 8



	Short presses - switch between
		Mode A0	Standard, No wiggler
		Mode A1	Standard, Wiggler

		Mode A2	Alternate, No wiggler
		Mode A3	Alternate, Wiggler


	Medium press - enter Voltage Measurement mode
		Voltage is shown on Hi D0-D7
		Pin to measured is show with Lo light
		Switch between pins with short presses
		Exit to normal mode with long press

	Long press - enter Setup mode
		Setup data to be sent from Arduino sketch

	Extra-long press - enter Attract mode 


*/

//
// Some bit-handling macros
//
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT)) 
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT)) 
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT)) 
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT)) 
#define WRITEBIT(RADDRESS,RBIT,WADDRESS,WBIT) (CHECKBIT(RADDRESS,RBIT) ? SETBIT(WADDRESS,WBIT) : CLEARBIT(WADDRESS,WBIT))

//
// Bit masks for the LEDs in g_greenbits and g_redbits
//
#define NOLED	0
#define LED_D0	0b00000000000000000000000000000001L
#define LED_D1	0b00000000000000000000000000000010L
#define LED_D2	0b00000000000000000000000000000100L
#define LED_D3	0b00000000000000000000000000001000L
#define LED_D4	0b00000000000000000000000000010000L
#define LED_D5	0b00000000000000000000000000100000L
#define LED_D6	0b00000000000000000000000001000000L
#define LED_D7	0b00000000000000000000000010000000L
#define LED_D8	0b00000000000000000000000100000000L
#define LED_D9	0b00000000000000000000001000000000L
#define LED_D10	0b00000000000000000000010000000000L
#define LED_D11	0b00000000000000000000100000000000L
#define LED_D12	0b00000000000000000001000000000000L
#define LED_D13	0b00000000000000000010000000000000L
#define LED_A0	0b00000000000000000100000000000000L
#define LED_A1	0b00000000000000001000000000000000L
#define LED_A2	0b00000000000000010000000000000000L
#define LED_A3	0b00000000000000100000000000000000L
#define LED_A4	0b00000000000001000000000000000000L
#define LED_A5	0b00000000000010000000000000000000L


//
// EEPROM variables
//
uint8_t EEMEM eeDummy;			// I don't trust the first byte of the eeprom
uint8_t EEMEM eeLastMode=0xFF;		// The last mode used
uint8_t EEMEM eeLoLevel1=0xFF; 
uint8_t EEMEM eeHiLevel1=0xFF; 
uint8_t EEMEM eeLoLevel2=0xFF; 
uint8_t EEMEM eeHiLevel2=0xFF; 
uint8_t EEMEM eeLastAdcPin=0x00;	
uint8_t EEMEM eeStartAttract=0x00;
//
// Global variables
//
volatile uint32_t g_redbits;	// Bit 0-19 is the state of the red LEDs
volatile uint32_t g_greenbits;	// Bit 0-19 is the state of the green LEDs
volatile uint16_t g_time;		// Incremented @ 500Hz by the T0 interrupt handler
uint8_t	g_wiggler;				// True if the wiggler is activated
uint8_t g_lolevel;				// Max value to indicate low level on a pin
uint8_t g_hilevel;				// Min value to indicate high level on a pin

#define PROPERLOW		15		// Fixed low max level for setup-communications
#define PROPERHIGH		240		// Fixed high min level for setup-communications


#define WIGGLER_LOW 	0		// Pull the ADC lines to ground via 1M resistors
#define WIGGLER_HIGH 	1		// Pull the ADC lines to VCC via 1M resistors
#define WIGGLER_HiZ		2		// Wiggler deactivated, ADC lines float

#define MUXA	3				// PB3 Selects input at the muxes
#define MUXB	4				// PB4 Selects input at the muxes
#define MUXC	5				// PB5 Selects input at the muxes

#define MUX1ADC	5				// MUX 1 is connected to ADC5
#define MUX2ADC	6				// MUX 2 is connected to ADC6
#define MUX3ADC	7				// MUX 3 is connected to ADC7

#define BUTTON 	5 				// The CONFIG button is at PB5 (sharing with MUXC)


#define TIMESHORT    500		//
#define TIMEMEDIUM  1500		//
#define TIMELONG	3000		//


#define VERSION_MAJOR	0
#define VERSION_MINOR	3


//
// Timer0 Overflow Interrupt
//
// Refreshes the LEDs from the g_greenbit and g_redbits global vatiables
// Also increments g_time
// Is called approximately every 2mS (about 500Hz)
//
ISR(TIMER0_OVF_vect) {
	static uint8_t cathode=0;
	uint8_t a1,a2;

	// Turn off all cathode drivers
	PORTC&=(~0X0F);						

	// Calculate new bitmaps for the group
	if (cathode==0) {
		a1=g_greenbits & 0xff;				// Green D0-D7
		a2=(g_greenbits>>8)&0x03;			// Green D8-D9
	} else if (cathode==1) {
		a1=g_redbits & 0xff;				// Red D0-D7
		a2=(g_redbits>>8)&0x03;				// Red D8-D9
	} else if (cathode==2) {
		a1= ((g_greenbits>>10)&0x0f) | 		// Green D10-D13 
			(((g_greenbits>>19)&1)<<4) |	// Green A5
			(((g_greenbits>>18)&1)<<5) |	// Green A4
			(((g_greenbits>>17)&1)<<6) |	// Green A3
			(((g_greenbits>>16)&1)<<7);		// Green A2
		a2=	(((g_greenbits>>15)&1)<<0) | 	// Green A1
			(((g_greenbits>>14)&1)<<1); 	// Green A0
	} else {
		a1= ((g_redbits>>10)&0x0f) | 		// Red D10-D13 
			(((g_redbits>>19)&1)<<4) |		// Red A5
			(((g_redbits>>18)&1)<<5) |		// Red A4
			(((g_redbits>>17)&1)<<6) |		// Red A3
			(((g_redbits>>16)&1)<<7);		// Red A2
		a2=	(((g_redbits>>15)&1)<<0) | 		// Red A1
			(((g_redbits>>14)&1)<<1); 		// Red A0
	} 

	// Output the calculated bitmaps to the leds
	PORTD=a1;							
	PORTB&=(~0x03);
	PORTB|=a2;

	// Turn on the new  cathode driver
	SETBIT(PORTC,cathode);				

	// Point to next cathode group
	cathode=(cathode+1)&3;

	// Increment time counter
	g_time++;
}



void Delay100mS(uint8_t cnt) {
	int16_t i;

	i=cnt*10;
	while (i-->0) _delay_ms(10);
}




//
// Flash all leds briefly the specified number of times
//
void Flash(uint8_t cnt) {
	uint8_t i;

	for (i=0; i<cnt; i++) {
		g_greenbits=0xFFFFF;
		g_redbits=0xFFFFF;
		Delay100mS(1);
		g_greenbits=0;
		g_redbits=0;
		Delay100mS(5);
	}
}




//
// Read an 8-bit value from the specified ADC channel
// 10000 of SelectMux() and AdcRead() takes ~ 2500 mS
//
uint8_t AdcRead(uint8_t channel) {
	uint8_t v;

   	ADMUX = _BV(ADLAR) | channel;		// Channel selection, use AREF, Left adjust 
	ADCSRA |= _BV(ADSC);              	// Start conversion
	while(!bit_is_set(ADCSRA,ADIF));  	// Loop until conversion is complete
	ADCSRA |= _BV(ADIF);              	// Clear ADIF by writing a 1 (this sets the value to 0)
	v=ADCH;
	return v;
}
 


//
// Set bits for selection of input port of the 4051 analogue multiplexers
//
void SelectMux(uint8_t no) {
	CLEARBIT(PORTB, MUXA);
	CLEARBIT(PORTB, MUXB);
	CLEARBIT(PORTB, MUXC);
	if (no&1) SETBIT(PORTB, MUXA);
	if (no&2) SETBIT(PORTB, MUXB);
	if (no&4) SETBIT(PORTB, MUXC);
}


//
// Get the state of the button.
// Button is shared with Mux Selection line C (PB5) 
// pulling the pin to ground via a resistor.
//
uint8_t ReadButton(void) {
	uint8_t v;

	CLEARBIT(DDRB, BUTTON);  	// Input mode
	SETBIT(PORTB, BUTTON); 		// Enable internal pullup
	v=CHECKBIT(PINB, BUTTON);	// Read button state
	CLEARBIT(PORTB, BUTTON); 	// Disable pullup
	SETBIT(DDRB, BUTTON);  		// Restore to output mode again
	return !v;
}



//
// Set the wiggler to specified state
// Wigglers are pullup- or pulldown -resistors connected to the
// ADC inputs. This allows the firmware to determine if the ADC pin is 
// floating or not.
//
void WiggleState(uint8_t state) {

	if (state==WIGGLER_LOW) { 
		SETBIT(DDRB, 2);	// Set wiggler pins as output
		SETBIT(DDRB, 7);
		SETBIT(DDRC, 4);
		CLEARBIT(PORTB,2);	// Set wigglers low
		CLEARBIT(PORTB,7);
		CLEARBIT(PORTC,4);

	} else if (state==WIGGLER_HIGH) {
		SETBIT(DDRB, 2);	// Set wiggler pins as output
		SETBIT(DDRB, 7);
		SETBIT(DDRC, 4);
		SETBIT(PORTB,2);	// Set wigglers high
		SETBIT(PORTB,7);
		SETBIT(PORTC,4);

	} else { // DISABLED
		CLEARBIT(DDRB, 2);	// Set wiggler pins as input
		CLEARBIT(DDRB, 7);
		CLEARBIT(DDRC, 4);
		CLEARBIT(PORTB, 2);	// Disable internal pullups
		CLEARBIT(PORTB, 7);
		CLEARBIT(PORTB, 4);
	}
}





uint8_t Read1PinSpecial(uint8_t mux, uint8_t adc) {
	uint8_t v;

	SelectMux(mux); 
	v=AdcRead(adc);	
	return v;
}




//
// Test one pin to for proper high- or low- values as according to the
// levels in g_hilevel and g_lolevel. (These values are setup by the user
// and stored in the EEPROM).
//
// If wiggler mode is activated verify that the external voltage can override
// a 1M resistor. If it can't then the pin if floating.
//
// Note that the A/B/C selection lines of the analogue multiplexers must have been
// set with SelectMux() before this function is called.
//
// Returns 0 for low, 1 for high, 2 for floating 
//
void Test1Pin(uint8_t adc, uint32_t *lobits, uint32_t *hibits, uint32_t pin) {
	uint8_t v1,v2;
	
	if (pin!=NOLED) {
		v1=AdcRead(adc);
		if (!g_wiggler) {
			if (v1>g_hilevel) *hibits|=pin;
			if (v1<g_lolevel) *lobits|=pin;
			return;
		}

		if (v1>g_hilevel) WiggleState(WIGGLER_LOW);
		if (v1<g_lolevel) WiggleState(WIGGLER_HIGH);
		v2=AdcRead(adc);
		if ((v1>g_hilevel) && (v2>g_hilevel)) *hibits|=pin;
		if ((v1<g_lolevel) && (v2<g_lolevel)) *lobits|=pin;
		WiggleState(WIGGLER_HiZ);
	}
}




void Test3Pins(uint8_t mux, uint32_t *lobits, uint32_t *hibits, uint32_t pin1, uint32_t pin2, uint32_t pin3) {
	SelectMux(mux); 
	Test1Pin(MUX1ADC, lobits, hibits, pin1);
	Test1Pin(MUX2ADC, lobits, hibits, pin2);
	Test1Pin(MUX3ADC, lobits, hibits, pin3);
}











uint8_t ReadPin(uint8_t pin, uint8_t rawmode) {
	uint8_t v;
	switch (pin) {
		case  0:v=Read1PinSpecial(0, MUX1ADC); break; 	// D0
		case  1:v=Read1PinSpecial(3, MUX1ADC); break; 	// D1
		case  2:v=Read1PinSpecial(3, MUX2ADC); break; 	// D2
		case  3:v=Read1PinSpecial(0, MUX2ADC); break; 	// D3
		case  4:v=Read1PinSpecial(1, MUX2ADC); break; 	// D4
		case  5:v=Read1PinSpecial(2, MUX2ADC); break; 	// D5
		case  6:v=Read1PinSpecial(5, MUX2ADC); break; 	// D6
		case  7:v=Read1PinSpecial(7, MUX2ADC); break; 	// D7
		case  8:v=Read1PinSpecial(6, MUX2ADC); break; 	// D8
		case  9:v=Read1PinSpecial(4, MUX2ADC); break; 	// D9
		case 10:v=Read1PinSpecial(3, MUX3ADC); break; 	// D10
		case 11:v=Read1PinSpecial(0, MUX3ADC); break; 	// D11
		case 12:v=Read1PinSpecial(1, MUX3ADC); break; 	// D12
		case 13:v=Read1PinSpecial(2, MUX3ADC); break; 	// D13
		case 14:v=Read1PinSpecial(2, MUX1ADC); break; 	// A0
		case 15:v=Read1PinSpecial(1, MUX1ADC); break; 	// A1
		case 16:v=Read1PinSpecial(4, MUX1ADC); break; 	// A2
		case 17:v=Read1PinSpecial(6, MUX1ADC); break; 	// A3
		case 18:v=Read1PinSpecial(7, MUX1ADC); break; 	// A4
		case 19:v=Read1PinSpecial(5, MUX1ADC); break;	// A5
		default: v=0;
	}

	if (rawmode==0) {
		if (v<PROPERLOW) {				// Clearly LOW
			v=0;		
		} else if (v>PROPERHIGH) {		// Clearly HIGH
			v=1;
		} else v=2;	
	}

	return v;
}




void SetupMode(void) {
	uint32_t hibits,lobits;
	uint8_t v;
	uint8_t b;
	uint8_t clkstate;
	uint8_t trigSeq[]={'V','i','s','u','l','i','z','e','r',0xF0, 0XF1, 0};
	uint8_t trigPos;
	uint8_t buf[16];

	lobits=0;
	hibits=0;
	clkstate=0;
	trigPos=0;
	b=0;
	while (1) {

		hibits=0;
		lobits=0;

		Test3Pins(0, &lobits, &hibits, NOLED, NOLED, LED_D11);
		Test3Pins(1, &lobits, &hibits, NOLED, NOLED, LED_D12);
		Test3Pins(2, &lobits, &hibits, NOLED, NOLED, LED_D13);
		Test3Pins(3, &lobits, &hibits, NOLED, LED_D2, LED_D10);
		Test3Pins(4, &lobits, &hibits, NOLED, LED_D9, NOLED);
		Test3Pins(5, &lobits, &hibits, NOLED, LED_D6, NOLED); 
		Test3Pins(6, &lobits, &hibits, NOLED, LED_D8, NOLED);
		Test3Pins(7, &lobits, &hibits, NOLED, LED_D7, NOLED);

		switch ((g_time/32)%12) {
			case  0: lobits|=LED_A0; hibits&=(~LED_A0); break;
			case  1: lobits|=LED_A1; lobits&=(~LED_A0); break;
			case  2: lobits|=LED_A2; lobits&=(~LED_A1); break;
			case  3: lobits|=LED_A3; lobits&=(~LED_A2); break;
			case  4: lobits|=LED_A4; lobits&=(~LED_A3); break;
			case  5: lobits|=LED_A5; lobits&=(~LED_A4); break;
			case  6: hibits|=LED_A5; lobits&=(~LED_A5); break;
			case  7: hibits|=LED_A4; hibits&=(~LED_A5); break;
			case  8: hibits|=LED_A3; hibits&=(~LED_A4); break;
			case  9: hibits|=LED_A2; hibits&=(~LED_A3); break;
			case 10: hibits|=LED_A1; hibits&=(~LED_A2); break;
			case 11: hibits|=LED_A0; hibits&=(~LED_A1); break;
		} 

		v=ReadPin(2,0);
		// If CLK is low set state to wait for CLK to be high
		if (v==0) {			
			clkstate=1;
		}
		// If CLK is high now see if this was the transition LOW->HIGH
		if (v==1) {
			if (clkstate==1) {
				// It was the transition. Set state to ignore until LOW again
				clkstate=2;
				// Read 8 bits from pins and assemble into a byte
				b=0;
				if (ReadPin(13,0)==1) b|=0x80;
				if (ReadPin(12,0)==1) b|=0x40;
				if (ReadPin(11,0)==1) b|=0x20;
				if (ReadPin(10,0)==1) b|=0x10;
				if (ReadPin( 9,0)==1) b|=0x08;
				if (ReadPin( 8,0)==1) b|=0x04;
				if (ReadPin( 7,0)==1) b|=0x02;
				if (ReadPin( 6,0)==1) b|=0x01;

				// Compare sequential bytes for the trigger sequence.
				// Values 0xF0..0xFF is for collecting data into buffer
				// Value of 0x00 denotes end of string
				if ((b==trigSeq[trigPos]) || (trigSeq[trigPos]>=0xF0)) {
					if (trigSeq[trigPos]>=0xF0) buf[trigSeq[trigPos]-0xF0]=b; 
					trigPos++;
					// At end of string execute command
					if (trigSeq[trigPos]==0) {
						if (buf[0]==1) { // Exit setup mode
							return;
						}
						if (buf[0]==2) { // Attract mode set
							eeprom_update_byte(&eeStartAttract,buf[1]);
							Delay100mS(5);
							Flash(1);
							Delay100mS(5);
						}
						if (buf[0]==3) { // Standard Low Level
							eeprom_update_byte(&eeLoLevel1,buf[1]);
							Delay100mS(5);
							Flash(1);
							Delay100mS(5);
						}
						if (buf[0]==4) { // Standard High Level
							eeprom_update_byte(&eeHiLevel1,buf[1]);
							Delay100mS(5);
							Flash(1);
							Delay100mS(5);
						}
						if (buf[0]==5) { // Alternate Low Level
							eeprom_update_byte(&eeLoLevel2,buf[1]);
							Delay100mS(5);
							Flash(1);
							Delay100mS(5);
						}
						if (buf[0]==6) { // Alternate High Level
							eeprom_update_byte(&eeHiLevel2,buf[1]);
							Delay100mS(5);
							Flash(1);
							Delay100mS(5);
						}
						trigPos=0;
					}
				} else {
					trigPos=0;
				}


			}
		}

		g_greenbits=lobits;
		g_redbits=hibits;
	}
}




void AdcMode(void) {
	uint32_t hibits,lobits;
	uint8_t currentPin;
	uint8_t v;


	currentPin=eeprom_read_byte(&eeLastAdcPin);
	if (currentPin>19) currentPin=0;
	v=0;
	while (1) {
		// Short press advances to next pin
		if (ReadButton()) {
			g_time=0;
			g_greenbits=0;
			g_redbits=0;
			_delay_ms(20);
			while (ReadButton()) {
				if (g_time==TIMESHORT) Flash(1);
			}
			_delay_ms(20);
			// Medium press return to normal live-mode
			if (g_time>500) return;
			currentPin++;
			if (currentPin>19) currentPin=0;
			eeprom_update_byte(&eeLastAdcPin,currentPin);
		}

		v=ReadPin(currentPin,1);

		hibits=v; // Light up red D0-D7 as the voltage
		lobits=1L<<currentPin; // Show what pin is active
		g_greenbits=lobits;
		g_redbits=hibits;
	}
}





void AttractMode(void) {
	uint8_t c;
	uint8_t i;
	uint8_t bias;
	int8_t biasDir;

	// Show current FW version
	for (i=0; i<100; i++) { 
		g_redbits=1L<<VERSION_MAJOR;
		g_greenbits=1L<<VERSION_MINOR;
		Delay100mS(1);
		g_greenbits=0;
		g_redbits=0;
		Delay100mS(1);
	}

	biasDir=1;
	bias=50;
	g_time=0;
	while (1) {
		// Return to main mode if the button is pressed
		if (ReadButton()) {
			_delay_ms(20);
			while (ReadButton()) _delay_ms(20);
			_delay_ms(20);
			return;
		}

		// Light a random number of green and red LEDs
		c=1+(random()%10);
		for (i=0; i<c; i++) {
			// Bias the LEDs towards either green or red
			if ((random()%100)>bias) g_greenbits|=1L<<(random()%20);
			if ((random()%100)<bias) g_redbits|=1L<<(random()%20);
		}

		// Let the LEDs be lit for a random time
		c=10+(random()%100);
		for (i=0; i<c; i++) _delay_ms(1);

		// Turn them off for a random, but shorter time
		g_greenbits=0;
		g_redbits=0;
		c=1+(random()%200);

		for (i=0; i<c; i++) _delay_ms(2);
		// Slowly shift the bias between read and green
		if (g_time>200) {
			bias+=biasDir;
			if (bias==1) biasDir=-biasDir;
			if (bias==99) biasDir=-biasDir;
		}
	}

}






void ReadEeGlobals(uint8_t mode) {
	if ((mode==0) || (mode==1)) {
		g_lolevel=eeprom_read_byte(&eeLoLevel1);
		g_hilevel=eeprom_read_byte(&eeHiLevel1);
	}
	if ((mode==2) || (mode==3)) {
		g_lolevel=eeprom_read_byte(&eeLoLevel2);
		g_hilevel=eeprom_read_byte(&eeHiLevel2);
	}
	if ((mode==0) || (mode==2)) g_wiggler=0;
	if ((mode==1) || (mode==3)) g_wiggler=1;
}



int main(void) {
	uint32_t lobits;
	uint32_t hibits;
	uint8_t	readMode;

	readMode=eeprom_read_byte(&eeLastMode);
	if (readMode>4) {
		readMode=0;
		g_lolevel=15;
		g_hilevel=240;
		eeprom_update_byte(&eeLastMode,readMode);
		eeprom_update_byte(&eeLoLevel1, g_lolevel);
		eeprom_update_byte(&eeHiLevel1, g_hilevel);
		eeprom_update_byte(&eeLoLevel2, g_lolevel);
		eeprom_update_byte(&eeHiLevel2, g_hilevel);
		eeprom_update_byte(&eeLastAdcPin,readMode);
	}
	ReadEeGlobals(readMode);


	// Initialize port directions
	DDRB=0b00111011; 	// Wigglers and XTAL are inputs
	DDRC=0b00001111; 	// Analogue input and wigglers are inputs
	DDRD=0b11111111; 	// All leds are output

	WiggleState(0);		// Wigglers disabled

	// Start tTimer0 interrupt @ ~500Hz
    TIMSK|=(1<<TOIE0);  			// Enable timer0 overflow interrupt
    TCCR0|=(1<<CS01) | (1<<CS00);   // Prescale clock/64
    sei();							// Enable global interrupts

	// Initialize ADC 
	ADCSRA = _BV(ADEN) | 				// Enable ADC
			 _BV(ADPS2) | 				// Prescaler /128 
			 _BV(ADPS1) | 
			 _BV(ADPS0); 				


	if (eeprom_read_byte(&eeStartAttract)==1) AttractMode();

	for (;;) {

		if (ReadButton()) {
			g_time=0;
			g_greenbits=0;
			g_redbits=0;
			_delay_ms(20);
			while (ReadButton()) {
				if (g_time==TIMESHORT) Flash(1);
				if (g_time==TIMEMEDIUM) Flash(2);
				if (g_time==TIMELONG) Flash(3);
			}
			_delay_ms(20);

			// Short press activates next stored settings
			if (g_time<TIMESHORT) {
				readMode++;
				if (readMode>3) readMode=0;
				g_redbits=NOLED; 
				switch (readMode) {
					case 0: 
						g_greenbits=LED_A0; 
						ReadEeGlobals(readMode);
						break;
					case 1: 
						g_greenbits=LED_A1;  
						ReadEeGlobals(readMode);
						break;
					case 2: 
						g_greenbits=LED_A2; 
						ReadEeGlobals(readMode);
						break;
					case 3: 
						g_greenbits=LED_A3;  
						ReadEeGlobals(readMode);
						break;
				}

				Delay100mS(3);
			// Medium press activates ADC mode
			} else if((g_time>=TIMESHORT) && (g_time<TIMEMEDIUM)) {
				AdcMode();
			// Long press activates SETUP mode
			} else if ((g_time>=TIMEMEDIUM) && (g_time<TIMELONG)) {
				SetupMode();
			// Very long press activates AttractMode
			} else if (g_time>=TIMELONG) {
				AttractMode();
			}
		}

		// Display current state of all pins
		hibits=0;
		lobits=0;
		Test3Pins(0, &lobits, &hibits, LED_D0, LED_D3, LED_D11);
		Test3Pins(1, &lobits, &hibits, LED_A1, LED_D4, LED_D12);
		Test3Pins(2, &lobits, &hibits, LED_A0, LED_D5, LED_D13);
		Test3Pins(3, &lobits, &hibits, LED_D1, LED_D2, LED_D10);
		Test3Pins(4, &lobits, &hibits, LED_A2, LED_D9, NOLED);
		Test3Pins(5, &lobits, &hibits, LED_A5, LED_D6, NOLED); 
		Test3Pins(6, &lobits, &hibits, LED_A3, LED_D8, NOLED);
		Test3Pins(7, &lobits, &hibits, LED_A4, LED_D7, NOLED);
		g_greenbits=lobits;
		g_redbits=hibits;
	}
}

