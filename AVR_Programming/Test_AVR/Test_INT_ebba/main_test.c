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

    DDRD &= ~((1<<DDD2) | (1<<DDD3));   // PD2, PD3 = inputs
    // Optional: enable internal pull-ups if your encoder needs it:
    // PORTD |= (1<<PD2) | (1<<PD3);

    // Keep interrupts disabled for now (pure polling test)
    init_INTs();
    sei();
				
	/* Uncomment these two lines to use interrupts instead of polling...
	init_INTs();
	sei();    // set global interrupt flag -> enable ALL interrupts (but those that are masked won't do anything)
	*/
	
	while(1)
    {
       // Polling... - Comment out these four lines when running with interrupts.
	   //ChannelA = PIND & (1<<PIND2);	// Bits 2 and 3 are the ones with the sensor information
	   //ChannelB = PIND & (1<<PIND3);	
	   //set_LED(2,ChannelA);
	   //set_LED(3,ChannelB);	 
	 
	   
	   // Flash an LED while waiting...
	   set_LED(1,1);
	   //_delay_ms(1000);

	     
	}
}

ISR(PCINT2_vect)
{
	//This is the exact same code as used when polling!
	 set_LED(1,0);
	 
	 if((PIND & (1<<PIND2))) {
		 set_LED(2,1);
	 } else {
		 set_LED(2,0);
	 }
	if((PIND & (1<<PIND3))) {
		set_LED(3,1);
		} else {
		set_LED(3,0);
		}
}

int init_LEDs(void)
{
	DDRB |= (1 << PB7);
	DDRC |= (1 << PC4);	// Corresponding pins set as outputs 0b 0000 1111
	DDRD |= (1 << PD7);
	PORTB &= ~(1 << PB7);
	PORTC &= ~(1 << PC4);	// Initially all LED pins set to 0
	PORTD &= ~(1 << PD7);

	return 1;
}

int init_INTs(void)
{
    // PD2 and PD3 as inputs
    DDRD &= ~((1<<DDD2) | (1<<DDD3));    

    // Optional: internal pull-ups if the encoder has open switches
    // PORTD |= (1<<PD2) | (1<<PD3);

    // Enable pin change interrupts on PD2 (PCINT18) and PD3 (PCINT19)
    PCMSK2 |= (1<<PCINT18) | (1<<PCINT19);

    // Enable pin change interrupt group 2 (PCINT[23:16] = Port D)
    PCICR  |= (1<<PCIE2);

    return 1;
}

int set_LED(int position, int value)
{
		switch(position)
	{
		case 1:
		if (value)
		{
			PORTB &= ~(1 << PB7);
		}
		else
		{
			PORTB |= (1 << PB7);
		}
		break;
		case 2:
		if (value)
		{
			PORTC &= ~(1 << PC4);
		}
		else
		{
			PORTC |= (1 << PC4);
		}
		break;
		case 3:
		if (value)
		{
			PORTD &= ~(1 << PD7);
		}
		else
		{
			PORTD |= (1 << PD7);
		}
		break;
		
	}
	return 1;
}

