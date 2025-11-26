/*
 * HelloWorld.c
 *
 * Created: 2025-10-13 10:47:34
 * Author : frani
 */ 

#include <avr/io.h>


int main(void)
{
    char temp;
	/* Configure input/output pins */
	DDRC = DDRC | (1 << PC3);  //PC3 as output
	DDRC = DDRC & ~(1 << PC4); //PC4 as input (not needed!)
	
    while (1) 
    {
		temp = PINC & (1<<PINC4);  // Read bit 4 in Port C
		temp = temp >> PINC4; // temp is 1 or 0 after this
		if (temp==1)
			PORTC = PORTC | (1<<PC3); // turn on LED
		else
			PORTC = PORTC & ~(1<<PC3); // turn off LED
    }
}

