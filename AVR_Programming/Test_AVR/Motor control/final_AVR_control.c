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
#define n 8 // Q8.7 -> 7 fractional bits
#define Q_SCALE (1 << n)
#define BAUD 2400
#define MYUBRR ((F_CPU / 16 / BAUD) - 1)
#define MAX_TICK_COUNT 15625 // equal to 5 rpm (min)
#define MIN_TICK_COUNT 651	 // equal to 120 rpm (max)
#define MAX_RPM_Q 15744		 // 123*2^7, max_rpm in fixed point
#define N_SAMPLES 16
#define U_MIN 0
#define U_MAX 255
#define n_ctr 8
#define Q_ctr_SCALE (1 << n_ctr)
#define Kp 8
#define Ki 02

uint8_t duty = 255;
uint8_t flag = 0;

volatile uint8_t pi_tick = 0;
volatile uint8_t pi_enabled = 0;
//volatile uint8_t rpm_meas = 0; // from your measurement
volatile uint8_t pwm_cmd = 0;
volatile uint8_t rpm_ref = 0; // copy to local
volatile uint8_t rpm_acc = 0;
volatile uint8_t cmd = 0;

int16_t Kp_q = (int16_t)(0.8 * Q_ctr_SCALE);	  // Fixed-point Proportional gain Q7.8
int16_t Ki_q = (int16_t)(0.02 * Q_ctr_SCALE);	  // Fixed-point Integral gain Q7.8
int16_t Ts_q = (int16_t)(0.065536 * Q_ctr_SCALE); // Sample Time Ts = 0.065536, in Q7.8 format

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

uint8_t print = 0;

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
	// Set Timer2, 8 bit (255), to interrupt at 15-50Hz
	TCCR2A = 0;							// Normal operation (clean start), overruns when it passes its maximum 8-bit value -> restarts from bottom
	TCCR2B = (1 << CS22) | (1 << CS21); // prescaler = 256 --> f_ovf = 15,3Hz. precaler_128 wold give fovf = 30,52Hz
	TIMSK2 = (1 << TOIE2);				//: Timer/Counter2 overflow interrupt enable
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

// Implement an interrup timer that fires every interval

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

	TCCR0A = (1 << WGM00) | (1 << COM0A1) | (1 << COM0A0);
	TCCR0B = (1 << CS01); // prescaler /8 (adjust if needed)

	OCR0A = 0;
}

// Initialize Analog-to-Digital Converter (ADC) for PC5
int init_ADC(void)
{
    DDRC &= ~(1 << DDC0); // enable PC5 as a input

    ADMUX = (1 << REFS0) | 0; // sets VRef to VCC and enables ADC on PC5

    ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC (ADEN) and sets prescaler to 8 -> 125 kHz, safe range 50k - 200k

    // Perform a dummy read to stabilize the ADC
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));

    return 1;
}

// Read the ADC value from a specified channel (0-5, PC0 is channel 0)
uint16_t read_ADC(void)
{
    ADCSRA |= (1 << ADSC); // start the conversion, voltage to int

    while (ADCSRA & (1 << ADSC))
        ; // wait for the conversion to complete (ADSC goes low)

    return ADC; // return the 10-bit result, as a int
}
uint8_t calc_rpm()
{
	uint8_t rpm_meas = 0;
	uint32_t sum = 0;
	cli();
	for (uint8_t k = 0; k < N_SAMPLES; k++) {
		sum += delta_ticks[k];
	}
	sei();
	uint32_t avg = (sum + (len >> 1)) / len; // integer rounded mean
	uint16_t dt_tick = (uint16_t)avg;
	if (dt_tick != 0)
	{
		uint32_t q = (K_rpm + (dt_tick >> 1)) / dt_tick; // Q?.n
		//if (q > MAX_RPM_Q)
		//{
			//q = MAX_RPM_Q;
		//}
		rpm_meas = (uint8_t)(q >> n); // right shift 7 -> real number
	}
	return rpm_meas;
}
int8_t updatePI(uint8_t speed_acc, uint8_t speed_ref)
{
	
	int16_t e = (int16_t)speed_ref - (int16_t)speed_acc; // signed Q15.0

	int32_t P_q = (int32_t)Kp_q * (int32_t)e; // Q23.8

	int32_t dI_q = (int32_t)Ki_q * (int32_t)e;

	int64_t u_try_q = (int64_t)P_q + (int64_t)I_q + (int64_t)dI_q; // Q?.n_ctr
	int32_t u_try = (int32_t) (u_try_q >> n_ctr);	  // integer PWM

	if (!((u_try >= U_MAX && e > 0) || (u_try <= U_MIN && e < 0)))
	{ // Conditional integration, only if not saturating
		I_q += dI_q;
	}
	// Clamp integrator to output range
	//int32_t I_min_q = ((int32_t)U_MIN) << n_ctr;
	int32_t I_min_q = -((int32_t)U_MAX << n_ctr);
	int32_t I_max_q = ((int32_t)U_MAX) << n_ctr;
	if (I_q > I_max_q) I_q = I_max_q;
	if (I_q < I_min_q) I_q = I_min_q;
	// Sum and saturate output
	int64_t u_q = P_q + (int64_t)I_q; 
	//u_q = (u_q + (1 << n_ctr-1)) >> n_ctr;
	u_q = (u_q + (1LL << (n_ctr - 1))) >> n_ctr;
	int32_t u = u_q; // integer PWM

	if (u > U_MAX) u = U_MAX;
	if (u < U_MIN) u = U_MIN;

	return (int8_t)(u);
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
	timer2_init();
	sei();
	volatile int j = 0;

	while (1)
	{
		if (new_cmd){
			cmd = received_data; // copy to local
			rpm_acc = calc_rpm();
			new_cmd = 0;

			
			if (cmd == 0) {
				set_LED(2, 0);
				set_LED(3, 1);
				rpm_acc = calc_rpm();
				USART_Transmit(rpm_acc);
			}
			else {
				pi_enabled = 0;
				I_q = 0; // reset integrator
				if (cmd > 120) cmd = 120;
				rpm_ref = cmd;
				pi_enabled = 1;
			}
		}
		if (pi_tick && pi_enabled && rpm_ref > 0 && rpm_ref < 120) {
			pi_tick = 0;
			rpm_acc = calc_rpm();
            raw_pot = read_ADC();   // reads the potentiometer, returns int between 0 – 1023, corresponding to 0 - 5 V

            int16_t centered = (int16_t)raw_pot - 512; // approx -512 < centered < +511

            int8_t offset =(centered * 10) / 512; // calc offset with rounding: offset = round(centered * 10 / 512)

            int8_t new_target = rpm_ref + offset;

			uint8_t check = (uint8_t)updatePI(rpm_acc, rpm_ref); // uses rpm_ref and rpm_meas
			OCR0A = check;
			if (j >= 15) {
				USART_Transmit(rpm_acc);
				USART_Transmit(check);
				j = 0;
			}
			j++;
		}
	}
}
ISR(TIMER2_OVF_vect) // fires every Ts
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
	USART_Transmit(received_data);
	new_cmd = 1;
}
