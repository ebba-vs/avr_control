/*
 * measure_speed.c
 *
 * Created: 2025-12-09 12:37:28
 * Author : tmk25evs
 */ 

#define F_CPU 1000000UL         // or 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>             // dtostrf
#include <stdint.h>             // INT16_MAX
#include <stdio.h>
#include <stdbool.h>

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char c);
unsigned char USART_Receive(void);
int init_LEDs(void);
int init_INTs(void);
int init_PWM(void);
void timer1_init(void);
int set_LED(int position, int value);

void uart_send_string(const char *s);
int16_t get_rpm_q8_7(void);
void uart_print_rpm(int16_t rpm_q8_7);

#define EDGES_PER_REV      96UL
#define TIMER1_PRESCALER   8UL
#define n 7                 // Q8.7 -> 7 fractional bits
#define Q_SCALE            (1 << n)
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD - 1

volatile uint16_t dt_ticks = 0;              // ticks between encoder edges
volatile int16_t motorspeed_q = 0;           // RPM in Q8.7

volatile uint8_t received_data = 0;
volatile bool new_cmd = false;
// 60 * F_CPU * 2^7 / (prescaler * edges)
static const uint32_t K_RPM_Q =
(uint32_t)((60ULL * F_CPU * (uint32_t)Q_SCALE) /
(TIMER1_PRESCALER * EDGES_PER_REV));


int main(void)
{
	int sp;
	int8_t serialin, serialout;
	int16_t motorspeed;

    USART_Init(MYUBRR);
    init_INTs();
    init_PWM();
    timer1_init();
    init_LEDs();
    sei();

    while (1) {
        if (new_cmd) {
            uint8_t cmd = received_data;  // copy to local
            new_cmd = false;

            // echo back what we got
            USART_Transmit(cmd);

            motorspeed = get_rpm_q8_7();
            uart_print_rpm(motorspeed);
        }
    }
}
void USART_Init(unsigned int ubrr)
{
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;    

	DDRD |= (1 << PD1);                // TXD as output
    
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);	// Enable transmitter
    UCSR0C = (0 << USBS0) | (3 << UCSZ00); // 8N1
}

void USART_Transmit(unsigned char c)
{
	while (!(UCSR0A & (1 << UDRE0)));  // Wait until buffer empty
	set_LED(1, 1);
	__delay__ms(1000);
	set_LED(1,0);
	UDR0 = c;
}
unsigned char USART_Receive(void){
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void uart_send_string(const char *s)
{
	while (*s) USART_Transmit(*s++);
}


void uart_print_rpm(int16_t rpm_q8_7)
{
	int rpm = rpm_q8_7 >> n;  // Q8.7 -> float

	char str[20];
	sprintf(str, "%d", rpm);
	uart_send_string(str);
	uart_send_string("\r\n");
}

void timer1_init(void)
{
	TCCR1A = 0;                  // normal mode
	TCCR1B = (1 << CS11);        // prescaler = 8
	TCNT1  = 0;
}
int init_LEDs(void) {
	DDRB |= (1 << PB7);
	DDRC |= (1 << PC4);
	DDRD |= (1 << PD7);
	PORTB &= ~(1 << PB7);
	PORTC &= ~(1 << PC4);
	PORTD &= ~(1 << PD7);

	return 1;
}
int set_LED(int position, int value) {
	switch (position) {
		case 1:
		if (value) {
			PORTB &= ~(1 << PB7);
			} else {
			PORTB |= (1 << PB7);
		}
		break;
		case 2:
		if (value) {
			PORTC &= ~(1 << PC4);
			} else {
			PORTC |= (1 << PC4);
		}
		break;
		case 3:
		if (value) {
			PORTD &= ~(1 << PD7);
			} else {
			PORTD |= (1 << PD7);
		}
		break;
	}
	return 1;
}

int init_INTs(void)
{
	DDRD &= ~((1 << DDD2) | (1 << DDD3));	// PD2 and PD3 as inputs
	PCICR  |= (1 << PCIE2); 	// enable PCINT group 2
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); // enable pin change on PD2 (PCINT18) and PD3 (PCINT19)
	PORTD |= (1 << PD2) | (1 << PD3); 	// internal pull-ups
	return 1;
}
int init_PWM(void) {
	// PD6 (OC0A) as output
	DDRD |= (1 << DDD6);

	TCCR0A = 0;
	TCCR0B = 0;

	TCCR0A |= (1 << WGM00);

	// Non-inverting PWM on OC0A: COM0A1:0 = 0b10
	TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0);

	// Prescaler: clk/8  (CS01 = 1)
	// F_PWM = F_CPU / (2 * N * 256) = 1MHz / (2*8*256) ≈ 244 Hz
	TCCR0B |= (1 << CS00); // no prescaling
	
	OCR0A = 20;
	set_LED(3, 1);
	return 1;
}

int16_t get_rpm_q8_7(void)
{
	uint16_t ticks;

	// safely copy dt_ticks (16-bit volatile)
	cli();
	ticks = dt_ticks;
	sei();

	if (ticks == 0) {
		return 0;
	}

	// rpm_q = K_RPM_Q / ticks  (integer division)
	uint32_t temp = K_RPM_Q / (uint32_t) ticks;

	// saturate to int16_t range
	if (temp > INT16_MAX) {
		temp = INT16_MAX;
	}

	return (int16_t)temp;  // Q8.7 rpm
}

ISR(PCINT2_vect)
{
	dt_ticks = TCNT1;   // ticks since last edge
	TCNT1   = 0;        // restart timer for next period
}

ISR(USART_RX_vect){
    received_data = UDR0;  // grab byte
    new_cmd = true;        // signal main
}




