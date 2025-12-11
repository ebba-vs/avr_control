/*Compile the code, gcc -o myfile.exe serialexample.c serialport.c */
/*Execute your file!   ./myfile.exe */

#include "serialport.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

void loopBackTest(void);
void setRPM(void);
void setPWM(void);
void getRPM(void);

/*Declaration of variables*/
int sp,sl;
char cin[9];
char cout[9];
uint8_t function;
uint8_t functionin;

int main(void)
{
	/*Initialise serial port */
	sp = serial_init("/dev/ttyS0",0);
	if(sp == 0)
	{
		printf("Error! Serial port could not be opened.\n");
	}
	else
	{
		printf("Serial port open with identifier %d \n",sp);
	}

	while(1){
		printf("-----------------------------------\n");
		printf("Enter your function: \n 0-Loop back test \n 1-Set RPM \n 2-Set PWM \n 3-Get RPM \n");
		printf("-----------------------------------\n");
		//Wait until input, then continue
		//scanf("%hhu", &function);

		while (1) {
            if (scanf("%hhu", &function) == 1 && (function == 0 || function == 1 || function==2 || function==3)) {
                break;  // Valid input, exit loop
            } else {
                printf("Invalid input. Please enter 0, 1, 2 or 3.\n");
                while (getchar() != '\n'); // Clear the input buffer
            }
        }

		//printf("You chose %hhu \n", function);
		write(sp,&function,1);
		sleep(1);
		read(sp,&functionin,1);
		/*Check that the input string is equal to the output one! */
		printf("Output: %hhu, Input: %hhu \n", function, functionin);

		switch(function){
			case 0:
				loopBackTest();
			break;
			case 1:
				setRPM();
			break;
			case 2:
				setPWM();
			break;
			case 3:
				getRPM();
			break;
		}
		
	}
 return 1;
}

void setRPM(void){
	uint8_t rpmout, rpmin;
	printf("Set your RPM: \n");
	scanf("%hhu", &rpmout);
	write(sp,&rpmout,1);
	sleep(1);
	read(sp,&rpmin,1);
	/*Check that the input string is equal to the output one! */
	printf("Output: %hhu, Input: %hhu \n", rpmout, rpmin);
	
	
	/*Close the serial port */	
	//serial_cleanup(sp);
}

void setPWM(void){
	uint8_t rpmout, rpmin;
	printf("Set your PWM: \n");
	scanf("%hhu", &rpmout);
	write(sp,&rpmout,1);
	sleep(1);
	read(sp,&rpmin,1);
	/*Check that the input string is equal to the output one! */
	printf("Output: %hhu, Input: %hhu \n", rpmout, rpmin);
	
	
	/*Close the serial port */	
	//serial_cleanup(sp);
}

void getRPM(void){ 
	uint8_t getrpm, getu;
	sleep(1);
	read(sp,&getrpm,1);
	sleep(1);
	read(sp,&getu,1);
	
	printf("Current RPM: %hhu \nFine adjustment: %hhu \n", getrpm, getu);
	
	
	/*Close the serial port */	
	//serial_cleanup(sp);
}

void loopBackTest(void){
	/*Initialise both strings*/
	/*The max size of the strings is 8 characters (by declaration)*/
	/*Remember to leave space for the termination character!*/
	
	/*The output string is initialised through the keyboard...*/
	printf("Please enter your desired string:");
	scanf("%s", &cout);
	//OBS! scanf will stop reading if there are any space characters. In that case, try using the function fgets instead!

	/*The input string is initialised to xxxxxxxx*/
	strncpy(cin,"xxxxxxxx",9);
	
	/*Verify that the strings were successfully initialised... */	
	printf("Output: %s, Input: %s \n", cout, cin);

	/*Send the string out */
	write(sp,cout,9);
	/*NOTE: The string itself may be smaller than 9 bytes... 
	but we are still sending 9 bits. 
	When communicating with the AVR we must be more careful! 
	We may want to use fixed string length all the time,
	or have a for loop and send one character at a time*/
	
	
	/*Wait a little bit... (or until a signal comes!)*/
	sleep(1);
	
	/*Read the incoming string */
	read(sp,cin,9);
	
	/*Check that the input string is equal to the output one! */
	printf("Output: %s, Input: %s \n", cout, cin);
	
	/*Close the serial port */	
	//serial_cleanup(sp);
}
