/*
Chinese Scale/Caliper readout for Arduino/Microchip Studio
Optimized for Arduino Nano (ATMega328P) platform
2021 - Marian Neubert
*/

// Clock 16MHz
#define F_CPU 16000000UL

// Pins
#define CLK_PIN PORTD2
#define DATA_PIN PORTD4
#define ZERO_PIN PORTD5
#define MODE_PIN PORTD6

// Function Prototypes
void fncSetup (void);
long fncConvertScale(volatile unsigned char *, unsigned char);
void fncSerialHandle();
void fncFastMode();
void fncZeroScale();

// Includes
#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "inc/uart.h"

// Variables
// global flags/vars: bit counter, buffer full, buffer runtime window, read in progress, timer flag
volatile unsigned char ucBitCount1=0, ucConvFlag1=0, ucBufWndw1=0, ucBufRun1=0, ucTimerFlag1=0;

// global buffer for scales
// meant for only one bit but set to whole byte for better performance
volatile unsigned char ucRawValue1[24];

// scale value as union (long and byte)
typedef union {
	long lVal;
	unsigned char chrs[4];
} ufScaleValue1_t;
ufScaleValue1_t  ufScaleValue1;

// Serial - set baud rate
//const unsigned int ulBaudRate=9600;  // (0.2% error)
const unsigned int ulBaudRate=38400; // (0.2% error)
//const unsigned long ulBaudRate=76800; // (0.2% error)



// Main Function, never ends
int main(void)
{

	// Initialize hardware
	fncSetup();

	// Main loop
	while(1)
	{

		if(ucConvFlag1) // data conversion could be done
		{
			long lTempScaleValue = fncConvertScale(ucRawValue1, DATA_PIN); // convert data to mm, store temporarily
			ufScaleValue1.lVal = lTempScaleValue; // copy 4 bytes of data to union
			ucConvFlag1=0; // reset conversion flag, start new caliper reading
		}
		
		if(ucTimerFlag1) // cyclic timer has triggered
		{
			uart_putc(0xAA); // Signal Start by sending b10101010 (170 Decimal)
			// serially send out long value from union, 4 bytes one by one, LSB first
			for (unsigned char ucTemp = 0; ucTemp < 4; ucTemp++) uart_putc(ufScaleValue1.chrs[ucTemp]);
			ucTimerFlag1=0; // reset timer flag
		}
		
		fncSerialHandle(); // Handle serial incoming data
		
		wdt_reset(); // reset watchdog
	}

	return 0; // never reached
}



//
// #### Functions ####
//

// handle incoming serial data
void fncSerialHandle()
{
	if(uart_available() > 0) // Only if there is a buffered byte available
	{
		unsigned int ciRXByte = uart_getc(); // read received byte out of buffer
		switch (ciRXByte)
		{
			case 'z': // if "z" is entered, zero scale
			fncZeroScale();
			break;
			case 'm': // if "m" is entered, switch mode (normal/fast)
			fncFastMode();
			break;
			default: // default: do nothing
			break;
		}
	}
}


// Initialize Hardware
void fncSetup (void)
{
	// Configure Interrupt-Pin and Scale Data Input
	DDRD &= ~((1<<CLK_PIN)|(1<<DATA_PIN)); // Input
	PORTD &= ~((1<<CLK_PIN)|(1<<DATA_PIN)); // Tristate (Hi-Z)

	// Configure Zero and Reset pins
	DDRD |= ((1<<ZERO_PIN)|(1<<MODE_PIN)); // Output
	PORTD |= ((1<<ZERO_PIN)|(1<<MODE_PIN)); // Set Output Level to High

	// Configure external interrupt INT0
	MCUCR |= (1<<ISC01); // falling edge external interrupt 0
	EIMSK |= (1<<INT0); // External Interrupt Request 0 Enable
	EICRA |= (1<<ISC01); //Enable external interrupts for INT0

	// Timer 1 (cyclic actions)
	// Waveform Generation Mode 4 (Clear Timer on Compare Match / CTC)
	TCCR1B |= (1<<WGM12); // (TOP on OCRA, Update of OCRx immediately, TOV Flag Set on MAX)
	TCCR1B |= ((1<<CS12)|(1<<CS10)); // clkI/O / 1024 (from prescaler) = 15,625kHz at 16Mhz
	OCR1A = 781; // CTC on ~50ms @ 15,625kHz
	TIMSK1 |= (1<<OCIE1A); // Timer/Counter1, Output Compare A Match Interrupt Enable

	// Timer2 (sync on start of scale's packet transmission)
	// Waveform Generation Mode 1 (Clear Timer on Compare Match / CTC)
	TCCR2A |= (1<<WGM21); // (TOP on OCR1A, Update of OCR1x immediately, TOV1 Flag Set on MAX)
	TCCR2B |= (1<<CS22); // clkT2S / 64 (from prescaler) = 250kHz at 16Mhz
	OCR2A = 30; // CTC on ~120us @ 250kHz
	TIMSK2 |= (1<<OCIE2A); // Timer/Counter2 Output Compare Match A Interrupt Enable

	// Initialize serial port
	uart_init( UART_BAUD_SELECT(ulBaudRate,F_CPU) );
	
	// Enable watchdog timer, 500ms
	wdt_enable(WDTO_500MS);

	// Globally enable interrupt handling
	sei();
}

// convert collected bits in array ucRawValue to millimeters as long with 100's mm
long fncConvertScale(volatile unsigned char *ucRawValue, unsigned char ucPin)
{
	unsigned char ucTemp=0;
	long lScaleValue=0;
	
	// Iterate though 24bit of ucRawValue, use pin value (ucPin) to select bit out of the byte
	// pick only the interesting bit of the data-PIN (left-/right-shift by pin value)
	// Shift left for later correct complements conversion
	for(ucTemp=0; ucTemp<24; ucTemp++) lScaleValue |= ((int32_t)((((ucRawValue[ucTemp])&(1<<ucPin))>>ucPin))<<(ucTemp+8));

	// see https://www.shumatech.com/support/chinese_scales.htm for protocol description - excerpt:
	// The Chinese scales do not handshake the data output.
	// The scale data is sent continuously with a fixed, ~300ms period.
	// The serial data stream is 48 bits long and is clocked by the scale at a nominal frequency of 90 kHz.
	// The 48-bit serial stream contains two 24-bit words that are the absolute and relative positions of
	// the scale in binary format, not BCD format like the Digimatic protocol.  Each 24-bit word is sent least
	// significant bit (LSB) first, which is opposite from most serial protocols that send the most
	// significant bit (MSB) first.  The units of each word are in 20,480 positions per inch or 0x5000 in
	// hexadecimal.  The first word is the absolute position of the scale with an arbitrary origin that
	// remains fixed until the scale loses power.  The second word is the relative position of the scale
	// with an origin that is reset with every press of the zero button on the scale.
	// The positions are signed values so negative values are expressed in two's complement notation.

	lScaleValue = (lScaleValue>>8); // Shift 8 bits right for complements conversion
	lScaleValue = ((lScaleValue * 1270) / 10240); // convert from inch to mm * 100

	return lScaleValue;
}

// Switch between fast and slow mode
void fncFastMode()
{
	PORTD &= ~(1<<MODE_PIN); // set DATA pin to high level
	_delay_ms(120);
	PORTD |= (1<<MODE_PIN); // set DATA pin to normal level
	_delay_ms(120);
	PORTD &= ~(1<<ZERO_PIN); // set CLK pin to high level
	_delay_ms(60);
	PORTD |= (1<<ZERO_PIN); // set CLK pin to normal level
	wdt_reset(); // reset watchdog
}

// Set scale to zero
void fncZeroScale()
{
	PORTD &= ~(1<<ZERO_PIN); // set CLK pin to high level
	_delay_ms(300);
	PORTD |= (1<<ZERO_PIN); // set CLK pin to normal level
	wdt_reset(); // reset watchdog
}



//
// #### Interrupts ####
//

// External Interrupt 0
ISR(INT0_vect)
{
	if(ucConvFlag1==0) // Proceed only if there is no active conversion running
	{
		if(ucBitCount1==0) 	ucBufRun1=1; // start reading data on clock edge

		ucBitCount1++;
		
		if(ucBitCount1==48)	// stop reading data on clock edge
		{
			ucBufRun1=0; // indicate stop of readout
			ucBufWndw1=0; // reset window
		}
		
		if(ucBitCount1==49) // finish reading data on clock edge
		{
			ucBitCount1=0; // final clock edge, reset bit counter
			ucConvFlag1=1; // indicate, that data could be converted
		}
		
		// insert whole port value into array (no triple sampling due to strict timing issues!)
		if(ucBitCount1>24) ucRawValue1[(ucBitCount1-25)] = ~PIND;
	}
}

// Timer/Counter2 Output Compare Match A Interrupt
// Sync on start bit by setting ucBitCount1 to 0 if time is exceeded while waiting for 48bit of data
ISR(TIMER2_COMPA_vect)
{
	if(ucBufRun1) //check only when running
	{
		if((++ucBufWndw1)==10)
		{
			 //reset when reading not finished after ~1300us
			ucBitCount1=0;
			ucBufWndw1=0;
		}
	}

}

// Timer/Counter0 Overflow Interrupt
// trigger cyclic jobs
ISR(TIMER1_COMPA_vect)
{
	ucTimerFlag1=1; // indicate triggered timer
}
