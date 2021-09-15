/*
 * Main includes
 */

#define F_CPU 16000000UL

// Pins
#define CLK_PIN PORTD2
#define DATA_PIN PORTD4
#define ZERO_PIN PORTD5
#define MODE_PIN PORTD6
#define DGB_PIN PORTD7


//Prototypes
void fInitHw (void);
signed long int fConvertScale(volatile unsigned char *, unsigned char);
