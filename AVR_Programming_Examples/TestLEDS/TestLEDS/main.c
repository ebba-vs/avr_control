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
	DDRC |= 0x0F;	// Corresponding pins set as outputs 0b 0000 1111 
	PORTC &= ~(0x0F);	// Initially all LED pins set to 0
	return 1;
}

int set_LED(int position, int value)
{
	switch(position)
	{
		case 1:
		if (value == 0)
		{
			PORTC &= ~(1 << 1);
		}
		else
		{
			PORTC |= (1 << 1);
		}
		break;
		case 2:
		if (value == 0)
		{
			PORTC &= ~(1 << 2);
		}
		else
		{
			PORTC |= (1 << 2);
		}
		break;
		case 3:
		if (value == 0)
		{
			PORTC &= ~(1 << 3);
		}
		else
		{
			PORTC |= (1 << 3);
		}
		break;
		
	}
	return 1;
}