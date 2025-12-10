/*
 * ChipEncoder.c
 *
 * Created: 2024-11-08 17:33:24
 * Author : tmk24dd
 */




#define DEBOUNCE_DELAY 1000
#define F_CPU 1000000
#define BAUD 2400
#define MYUBRR ((F_CPU/16/BAUD)-1)
#define BUFFERSIZE 8
#define FXP 8 
#define KP 400
#define KI 800
#define DT 0.05 //20Hz
#define DT_FXP 13 //12.8

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 

int init_LEDs(void);
int initPWM(void);
int init_INTs(void);
void init_COM(unsigned int ubrr);
void init_Timer(void);
void init_POT(void);

int USART_Transmit(unsigned char);
unsigned char USART_Receive(void);
int updatePWM(int);
void panel(uint8_t command);
void updatePI(void);

uint8_t duty=255;
uint8_t flag=0;

uint8_t kolla=0;

uint8_t rpmIn=0;
uint8_t rpmOut=0;
int8_t rpmPot=0;
uint32_t currRPM = 0;
uint32_t currRPM_FXP = 0;

int32_t e = 0;
int32_t error_FXP;
int32_t integral = 0;
int32_t u = 0;
int32_t P_term;
int32_t I_term;

uint32_t nbrOfMeasurements=0;
uint32_t lastTime = 0;    
uint32_t currentTime = 0;
uint32_t timeDiff = 0;
uint32_t sumTimediff;
uint32_t buffer[BUFFERSIZE] = {0};
uint32_t bufferPos = 0;

int main(void){
	init_LEDs();
	initPWM();
	init_INTs();
	init_POT();
	init_COM(MYUBRR);
	init_Timer();
	sei(); //Enable global interrupts (SREG |= (1<<I-bit))
	while(1){
		updatePWM(duty);
		if(flag){
			updatePI();
			flag=0;
		}

		// Check if data has been received
		if (UCSR0A & (1 << RXC0)){
			unsigned char command = USART_Receive();
			USART_Transmit(command);
			panel(command);
		}
	}
	return 1;
}

void panel(uint8_t command){

	switch(command){
		case 0:
			PORTC |= (1 << PC0);
			PORTC |= (1 << PC2);
			_delay_ms(1000);
			PORTC &= ~(1 << PC0);
			PORTC &= ~(1 << PC2);
		break;

		case 1:
			rpmIn = USART_Receive();
			USART_Transmit(rpmIn);
		break;

		case 2:
		duty = USART_Receive();
		USART_Transmit(duty);         
		break;	
		
		case 3:
		USART_Transmit(currRPM);
		USART_Transmit(kolla);
		break;
	}
}

int USART_Transmit(unsigned char data){ //ISR USART_RX_vect??
	// Wait for the transmit buffer to be empty
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data; //Transmit data 
	return 1;	
}
unsigned char USART_Receive(void){
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}
int updatePWM(int value){
	//OCR0A = value; //Motor pwm
	//rpm=1; //Set for test with fixed PWM
	
	OCR0B = value; //Control led
	return value;
}
void updatePI(void){
	ADCSRA |= (1<<ADSC); //Conversion analog to digital 10bit
	while(ADCSRA & (1<<ADSC));
	int16_t check = ADC;
	rpmPot = (int8_t)((check-512)/48); //Take value from register 0-1023
	
	
	duty = 255-(check>>2);
	kolla = duty;
	
	//kolla = 100+rpmPot;
	int8_t rrppmm = ((int8_t) (rpmIn+rpmPot));
	
	if(rrppmm<=0){
		
		rpmOut=0;
		}else{
			rpmOut = (uint8_t) rrppmm;
			//rpmOut = rpmIn+rpmPot;
	}
	
	
	
	if(rpmIn==0){
		OCR0A = 255; //stop motor
		for(int i=0;i<BUFFERSIZE;i++){
			buffer[i]=0;
		}
		nbrOfMeasurements=0;
	} else{
		
	
			
	sumTimediff = 0;
		
	for(int i=0;i<BUFFERSIZE;i++){
		sumTimediff += buffer[i];
	}
	//sumTimediff +=4; //Add 1/2
	
	uint32_t konst = 19531;
	konst = (konst<<FXP);
	
	uint32_t nbrOfMeasurementsFXP = (uint32_t)(nbrOfMeasurements<<FXP);
	
	uint64_t tt = (uint64_t) (nbrOfMeasurementsFXP); //FXP
	tt += (1<<(FXP-1)); //Add 1/2
	
	uint64_t nn = (uint64_t) (sumTimediff<<FXP);
	
	currRPM_FXP = (uint64_t) ((tt<<FXP)/nn);
	currRPM_FXP = (uint64_t) ((currRPM_FXP*konst)>>FXP);
	
	//kolla = (uint8_t) currRPM_FXP;
	
	//uint32_t tt = ((60*F_CPU*nbrOfMeasurements)<<FXP);
	//uint32_t nn = ((sumTimediff*48*64)<<FXP);
	//currRPM_FXP = ((tt<<FXP)/nn);
	//currRPM_FXP = (currRPM_FXP<<FXP);
	
	currRPM = ((60*F_CPU*nbrOfMeasurements)/(sumTimediff*48*64));
		
	e = (rpmOut - currRPM);
	error_FXP = (e<<FXP);
		
	integral += ((error_FXP*DT_FXP)>>FXP); //X*Y=Z
		
	if(integral>18000){
		integral=18000;
	} else if(integral<-15000){
			integral = -15000;
	}
	
	//if(rpmOut<25){
		//P_term = ((400*error_FXP)>>FXP);
		//I_term = ((800*integral)>>FXP);
	//} else{
		//P_term = ((KP*error_FXP)>>FXP);
		//I_term = ((KI*integral)>>FXP);
	//}
	P_term = ((KP*error_FXP)>>FXP);
	I_term = ((KI*integral)>>FXP);
	
	u= P_term + I_term; //FXP
	u = (u>>FXP);
		
	if(u<0){
		integral -= ((u)<<FXP);
		u=0;
	} else if(u>255){
		integral += ((255-u)<<FXP);
		u=255;
	}

	OCR0A = 255-u;
	}
}

//Encoder interrupts
ISR(PCINT1_vect){
//24*2*2=96 interrupts per rotation, if only A -> 48 interrupts
	
	currentTime = TCNT1; //Current time from Timer1, 1 = 64us
	
	if(nbrOfMeasurements<BUFFERSIZE){ //Let buffer fill up
		nbrOfMeasurements++; //1-8
	}

	if(currentTime>=lastTime){
		timeDiff = currentTime - lastTime;
	} else{
		timeDiff = (currentTime+65536) - lastTime; //Overflow
	}
	
	if(timeDiff>110){
		PORTC &= ~(1 << PC2);
		buffer[bufferPos] = timeDiff;
		if(bufferPos<BUFFERSIZE-1){ //Next pos or loop around
			bufferPos++;
		} else{
			bufferPos = 0;
		}
	}
	
	lastTime = currentTime;
}

ISR(TIMER2_COMPA_vect){//PI controller at 20Hz
	flag=1;
}
void init_Timer(void){

	//F_TIMER (interrupt) = F_CPU/(prescaler*(TOP+1))

	// Set Timer1, 16 bit (65536)
	TCCR1A = 0;  // Normal operation
	TCCR1B = (1 << CS11)|(1 << CS10);  //Prescaler = 64, counts every 64th tick, 64/F_CPU=64us per tick
	//|(1 << CS10) if 64
	
	//Set Timer2, 8 bit (255), to interrupt at 20.03 Hz
	TCCR2A = 0; //Normal operation (clean start)
	TCCR2B = (1 << CS22) | (1 << CS21)| (1 << WGM22); //Prescaler = 256 and enable CTC mode (Clear Timer on Compare Match)
	OCR2A = 194; //TOP value for Output Compare interrupt and compare match
	TIMSK2 = (1<<OCIE2A); //Enable Output Compare match A interrupts
}
int init_LEDs(void){
	DDRC |= ((1<<PC2) | (1<<PC0));	// PC2 & PC0 to output 
	//PORTC &= ~((1<<PC2) | (1<<PC0));	// Initially all LED pins set to 0
	return 1;
}
int init_INTs(void){
	DDRC &= ~((1<<PC3) | (1<<PC1));    // Set PC3 & PC1 as inputs
	PCICR |= (1<<PCIE1);    // Enable PC interrupt group 1 (PCINT8 -> PCINT15)
	PCMSK1 |= ((1<<PCINT11)| (1<<PCINT9)); // Enable interrupts on PC3 (PCINT11) and PC1 (PCINT9)
	PORTC |= ((1<<PC3) | (1<<PC1)); // Enable pull-up resistors on PC1 and PC3 (optional but typical for interrupt handling)
	return 1;
}
int initPWM(void){ //Phase correct??
	//DDRD |= ((1<<DDD5)|(1<<DDD6));	//Set PIND5 and PIND6 as outputs
	//TCCR0A |= (1<<COM0A1)|(1<<COM0B1)|(1<<COM0B0)|(1<<WGM01)|(1<<WGM00); //0b10110011 - COM0A non-inverting, COM0B inverting, Mode Fast-PWM
	//TCCR0B |= (1<<CS01); //0b00000001 - Clock select: internal clock (1<<CS00) => no pre-scaler, (1<<CS01)=>8 prescaler
	DDRD |= ((1<<DDD5)|(1<<DDD6));	//Set PIND5 and PIND6 as outputs
	TCCR0A |= (1<<COM0A1)|(1<<COM0B1)|(1<<WGM00); //Phase Correct
	TCCR0B |= (1<<CS00); //No prescaler
	return 1;
}
void init_COM(unsigned int ubrr){
	//cli(); //Disable global interrupt flag before initialization of USART 
	
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00); //UCSZ01
		
	//sei(); //Enable
}
void init_POT(void){
	ADCSRA |= (1<<ADEN); //ADC enable
	ADMUX |= (1<<REFS0); //Set ref to AVCC
	ADMUX |= ((1<<MUX2)|(1<<MUX0)); //0101 for MUX3-0 for ADC5 as negative input for ANALOG COMPARATOR
}