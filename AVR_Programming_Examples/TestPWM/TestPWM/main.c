/*
 * TestPWM.c
 *
 * EIEN65 Applied Mechatronics 2023
 *  Author: FranM
 */ 

#define F_CPU 1000000  //Required only for the "delay" function
#include <avr/io.h>
#include <util/delay.h>

int initPWM(void);
int updatePWM(int);

int main(void)
{
	
	int duty = 10;
	initPWM();
	updatePWM(duty);
	while(1)
    {  //Change the duty cycle of the PWM signals every 250 ms
        _delay_ms(250);
		duty+=10;
		if (duty>255)
		{
			duty = 10;
		}
		updatePWM(duty); 
    }
}

int initPWM(void)
{
	DDRD |= ((1<<DDD5)|(1<<DDD6));	//Set PIND5 and PIND6 as outputs
	TCCR0A |= (1<<COM0A1)|(1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1<<WGM00); //0b10110011 - COM0A non-inverting, COM0B inverting, Mode Fast-PWM
	TCCR0B |= (1<<CS00); //0b00000001 - Clock select: internal clock no prescaler
	return 1;
}

int updatePWM(int value)
{
	OCR0A = value;
	OCR0B = value;
	//OCR0B = 127;  //If we uncomment this line, the output COM0B is fixed at about 50% duty-cycle
	return value;
}

