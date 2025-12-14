/*
 * measure_speed.c
 *
 * Created: 2025-12-09 12:37:28
 * Author : tmk25evs
 */

#define F_CPU 1000000 // UL // or 8000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h> // INT16_MAX
#include <stdint.h> // INT16_MAX
#include <stdio.h>
#include <stdlib.h> // dtostrf
#include <stdlib.h> // dtostrf
#include <util/delay.h>

void init_LEDs(void);
void init_INTs(void);
void init_PWM(void);
int set_LED(int position, int value);
void set_PWM(int direction);
void USART_Transmit(unsigned char c);
void USART_Init(unsigned int ubrr);
unsigned char get_rpm_q8_7(uint16_t dt_tick);

#define EDGES_PER_REV 96UL
#define TIMER1_PRESCALER 8UL
#define n 7 // Q8.7 -> 7 fractional bits
#define Q_SCALE (1 << n)
#define BAUD 2400
#define MYUBRR ((F_CPU / 16 / BAUD) - 1)
#define MAX_TICK_COUNT 15625 // equal to 5 rpm (min)
#define MIN_TICK_COUNT 651	 // equal to 120 rpm (max)
#define MAX_RPM_Q 15744		 // 123*2^7, max_rpm in fixed point
#define N_SAMPLES 16
#define U_MIN 0
#define U_MAX 255
#define Kp 0.8
#define Ki 0.02

uint8_t duty = 255;
uint8_t flag = 0;

volatile uint8_t pi_tick = 0;
volatile uint8_t rpm_ref = 60; // setpoint
volatile uint8_t rpm_meas = 0; // from your measurement
volatile uint8_t pwm_cmd = 0;

int16_t Kp_q = (int16_t)(0.8 * Q_SCALE);  // example
int16_t Ki_q = (int16_t)(0.02 * Q_SCALE); // Ki*Ts example (already includes Ts!)

int32_t I_q = 0; // integrator in Q8.8 (PWM units)

volatile uint16_t dt_ticks = 0;		// ticks between encoder edges
volatile uint16_t motorspeed_q = 0; // RPM in Q8.7

volatile uint16_t delta_ticks[N_SAMPLES];
uint8_t len = N_SAMPLES;
volatile int i = 0;
volatile uint8_t n_ticks = 0;
volatile uint32_t sum = 0;
// volatile uint16_t dt_tick = 0; // ticks since last edge

volatile uint8_t received_data = 0;
volatile uint8_t new_cmd = 0;
volatile uint8_t motorspeed = 0;
volatile uint8_t pi_enabled = 0;

volatile uint8_t rpm_meas = 0;

volatile uint16_t last_tick = 0;

// 60 * F_CPU * 2^7 / (prescaler * edges)
static const uint32_t K_rpm = (uint32_t)((60ULL * F_CPU * (uint32_t)Q_SCALE) /
										 (TIMER1_PRESCALER * EDGES_PER_REV)); // 78125 for ps=8, F_CPU=1Mhz

void USART_Init(unsigned int ubrr)
{
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;

	DDRD |= (1 << PD1); // TXD as output
	// DDRD &= ~(1 << PD0);

	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable transmitter
	UCSR0C = (0 << USBS0) | (3 << UCSZ00);				  // 8N1
}

void USART_Transmit(unsigned char c)
{
	while (!(UCSR0A & (1 << UDRE0)))
	{

	} // Wait until buffer empty
	UDR0 = c;
}

void timer1_init()
{
	TCCR1A = 0; // normal mode
	TCCR1B = 0;
	TCCR1B = (1 << CS11); // prescaler = 8
	TCNT1 = 0;
}
void timer2_init()
{
	// Set Timer2, 8 bit (255), to interrupt at 20Hz
	TCCR2A = 0;										   // Normal operation (clean start)
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << WGM22); // Prescaler = 256 and enable CTC mode (Clear Timer on Compare Match)
	OCR2A = 194;									   // TOP value for Output Compare interrupt and compare match
	TIMSK2 = (1 << OCIE2A);							   // Enable Output Compare match A interrupts
}
void init_LEDs()
{

	DDRB |= (1 << PB7);
	DDRC |= (1 << PC4);
	DDRD |= (1 << PD7);
	PORTB &= ~(1 << PB7);
	PORTC &= ~(1 << PC4);
	PORTD &= ~(1 << PD7);
}
int set_LED(int position, int value)
{
	switch (position)
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

void init_INTs()
{
	DDRD &= ~((1 << DDD2) | (1 << DDD3)); // PD2 and PD3 as inputs
	PCICR |= (1 << PCIE2);				  // enable PCINT group 2
	PCMSK2 |=
		(1 << PCINT18) |
		(1 << PCINT19);				  // enable pin change on PD2 (PCINT18) and PD3 (PCINT19)
	PORTD |= (1 << PD2) | (1 << PD3); // internal pull-ups
}
void init_PWM()
{
	// PD6 (OC0A) as output
	DDRD |= (1 << DDD6);

	TCCR0A = (1 << WGM00) | (1 << COM0A1);
	TCCR0B = (1 << CS01); // prescaler /8 (adjust if needed)

	OCR0A = 0;
}

uint8_t calc_rpm() {
	uint32_t sum = 0;
	for (uint8_t k = 0; k < N_SAMPLES; k++) {
		sum += delta_ticks[k];
	}
	uint32_t avg = (sum + (len / 2)) / len; // integer rounded mean
	uint16_t dt_tick = (uint16_t)avg;
	if (dt_tick != 0) {
		uint32_t q = (K_rpm + (dt_tick >> 1)) / dt_tick; // Q?.n
		if (q > MAX_RPM_Q) {
			q = MAX_RPM_Q;
		}
		rpm_meas = (uint8_t)(q >> n); // right shift 7 -> real number
	}
	return rpm_meas;
}
	void set_PWM(int direction)
	{
		if (direction == 1)
		{
			if (OCR0A <= 255 - 5)
			{ // avoid overflow
				set_LED(2, 0);
				OCR0A += 5;
			}
		}
		else if (direction == -1)
		{
			set_LED(2, 1);
			if (OCR0A >= 5)
			{ // avoid underflow
				OCR0A -= 5;
			}
		}
	}
	void updatePI()
	{
		int16_t e = (int16_t)rpm_ref - (int16_t)rpm_meas; // signed
		int16_t Kp_q = (int16_t)(Kp << n);
		int16_t Ki_q = (int16_t)(Ki << n);
		// P term: Q9.7 = (Q9.7 * Q0.0)
		int32_t P_q = (int32_t)Kp_q * (int32_t)e; // Q9.7

		// I term update: Q9.7 += (Q9.7 * Q0.0)
		I_q += (int32_t)Ki_q * (int32_t)e; // Q9.7

		// Clamp integrator to output range
		int32_t I_min_q = ((int32_t)U_MIN) << n; // 0 << 7
		int32_t I_max_q = ((int32_t)U_MAX) << n; // 255 << 7
		if (I_q > I_max_q)
		{
			I_q = I_max_q;
		}
		if (I_q < I_min_q)
		{
			I_q = I_min_q;
		}
		// Sum and saturate output
		int32_t u_q = P_q + I_q; // Q9.7

		// Convert to integer PWM
		int32_t u = u_q >> n; // back to Q0.0

		if (u > U_MAX)
		{
			u = U_MAX;
		}
		if (u < U_MIN)
		{
			u = U_MIN;
		}
		pwm_cmd = (uint8_t)u;
		OCR0A = U_MAX - pwm_cmd; // or your PWM register
	}

	int main(void)
	{
		// int sp;
		// int8_t serialin, serialout;

		init_LEDs();
		init_PWM();
		USART_Init(MYUBRR);
		init_INTs();
		timer1_init();
		timer2_init()
			sei();

		while (1)
		{
			if (new_cmd)
			{
				uint8_t cmd = received_data; // copy to local
				new_cmd = 0;

				rpm_ref = cmd;
				if (rpm_ref > 120)
				{
					rpm_ref = 120;
				}
				I_q = 0; // reset integrator
				if (cmd == 0)
				{
					USART_Transmit(rpm_meas);
				}
				if (pi_tick)
				{
					pi_tick = 0;
					updatePI(); // uses rpm_ref and rpm_meas
				}
			}
		}
	}
	ISR(TIMER2_COMPA_vect) // fires every Ts
	{
		pi_tick = 1;
	}
	ISR(PCINT2_vect)
	{
		uint16_t delta = TCNT1;
		// Unsigned subtraction handles overflow automatically

		if (delta > MAX_TICK_COUNT)
		{
			delta = MAX_TICK_COUNT;
		}
		if (delta < MIN_TICK_COUNT)
		{
			delta = MIN_TICK_COUNT;
		}
		if (i >= 16)
		{
			i = 0;
		}
		delta_ticks[i] = delta;
		i++;

		TCNT1 = 0;
	}

	ISR(USART_RX_vect)
	{
		received_data = UDR0;
		rpm_ref = received_data;
		new_cmd = 1;
	}
