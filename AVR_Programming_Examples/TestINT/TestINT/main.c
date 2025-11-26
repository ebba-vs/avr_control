/*
 * TestINT.c
 *
 * EIEN65 Applied Mechatronics 2023
 *  Author: FranM
 */ 

// Includes and definitions
#define F_CPU 1000000  //Needed for the "delay" function
#include <avr/io.h>
#include <avr/interrupt.h> //We need these definitions to be able to use interrupts
#include <util/delay.h>

// Function declaration
int init_LEDs(void);
int init_INTs(void);
int set_LED(int position, int value);



int main(void)
{
    //Local variables declaration
	char ChannelA, ChannelB;
	
	init_LEDs();
				
	/* Uncomment these two lines to use interrupts instead of polling...
	init_INTs();
	sei();    // set global interrupt flag -> enable ALL interrupts (but those that are masked won't do anything)
	*/
	
	while(1)
    {
       // Polling... - Comment out these four lines when running with interrupts.
	   ChannelA = PINC & (1<<PINC4);	// Bits 4 and 5 are the ones with the sensor information
	   ChannelB = PINC & (1<<PINC5);	
	   set_LED(2,ChannelA);
	   set_LED(3,ChannelB);	 
	 
	   
	   // Flash an LED while waiting...
	   set_LED(1,1);
	   _delay_ms(1000);
	   set_LED(1,0);
	   _delay_ms(1000);
	 
	     
	}
}

ISR(PCINT1_vect)
{
	//This is the exact same code as used when polling!
	
	char ChannelAint, ChannelBint;
	ChannelAint = PINC & (1<<PINC4);	// Bits 4 and 5 are the ones with the sensor information
	ChannelBint = PINC & (1<<PINC5);	
	set_LED(2,ChannelAint);
	set_LED(3,ChannelBint);
}

int init_LEDs(void)
{
	DDRC |= 0x0F;	    // Corresponding pins set as outputs
	PORTC &= ~(0x0F);	// Initially all LED pins set to 0
	
	return 1;
}

int init_INTs(void)
{
	DDRC &= ~(0x30);	// PINC4 - PINC5 set as inputs
	PCMSK1 |= (1<<PCINT12)|(1<<PCINT13);   // PCINT12 and PCINT13  enabled
	PCICR |= (1<<PCIE1);	// The PC interrupt group 1 (PCINT8 -> PCINT15) enabled
	return 1;
}

int set_LED(int position, int value)
{
	switch(position)
	{
		case 1:
		if (value == 0)
		{
			PORTC &= ~(1 << PC1);
		}
		else
		{
			PORTC |= (1 << PC1);
		}
		break;
		case 2:
		if (value == 0)
		{
			PORTC &= ~(1 << PC2);
		}
		else
		{
			PORTC |= (1 << PC2);
		}
		break;
		case 3:
		if (value == 0)
		{
			PORTC &= ~(1 << PC3);
		}
		else
		{
			PORTC |= (1 << PC3);
		}
		break;
		
	}
	return 1;
}

