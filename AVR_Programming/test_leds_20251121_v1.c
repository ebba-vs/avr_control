/*
 * TestLEDS.c
 *
 * Created: 16/10/2019 20:59:27
 *  Author: FranM
 */ 

// Includes and definitions
#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>

// Functions declaration
int init_LEDs(void);
int	set_LED(int position, int value);


int main(void)
{
	init_LEDs();
	while(1)
	{
		set_LED(1,1);
		_delay_ms(200);
		set_LED(2,1);
		_delay_ms(200);
		set_LED(3,1);
		_delay_ms(200);
		
		set_LED(1,0);
		_delay_ms(200);
		set_LED(2,0);
		_delay_ms(200);
		set_LED(3,0);
		_delay_ms(200);
	}
	return 1;
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