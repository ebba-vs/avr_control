#define F_CPU 1000000UL         // or 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>             // dtostrf
#include <stdint.h>             // INT16_MAX

#define EDGES_PER_REV      96UL
#define TIMER1_PRESCALER   8UL
#define Q_FRAC_BITS        7                 // Q8.7 -> 7 fractional bits
#define Q_SCALE            (1 << Q_FRAC_BITS) 
#define BAUD 9600
#define UBRR_VALUE ((F_CPU/16/BAUD) - 1)

volatile uint16_t dt_ticks = 0;              // ticks between encoder edges
volatile int16_t motorspeed_q = 0;           // RPM in Q8.7

// 60 * F_CPU * 2^7 / (prescaler * edges)
static const uint32_t K_RPM_Q =
    (uint32_t)((60ULL * F_CPU * (uint32_t)Q_SCALE) /
               (TIMER1_PRESCALER * EDGES_PER_REV));

int main(void)
{
    uart_init();
    init_LEDs();        // assume this exists elsewhere
    init_INTs();
    timer1_init();
    sei();

    while (1) {
        if(new_cmd){
			uart_send_char(received_data);
			_delay_ms(500);
			new_cmd = 0;
		}
        motorspeed_q = get_rpm_q8_7();
        uart_print_rpm(motorspeed_q);   // print to Serial Monitor
        _delay_ms(200);
    }
}

void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;      // <-- was UBRR0H

    DDRD |= (1 << PD1);                // TXD as output

   	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);            // Enable transmitter
    // UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 data bits, 1 stop bit
    UCSR0C = (0 << USBS0) | (3 << UCSZ00); //UCSZ01
}

void uart_send_char(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));  // Wait until buffer empty
    UDR0 = c;
}

void uart_send_string(const char *s)
{
    while (*s) uart_send_char(*s++);
}

void uart_print_rpm(int16_t rpm_q8_7)
{
    float rpm = (float)rpm_q8_7 / (float)Q_SCALE;  // Q8.7 -> float

    char buf[20];
    dtostrf(rpm, 5, 2, buf);   // width 5, 2 decimal places
    uart_send_string(buf);
    uart_send_string("\r\n");
}

void timer1_init(void)
{
    TCCR1A = 0;                  // normal mode
    TCCR1B = (1 << CS11);        // prescaler = 8
    TCNT1  = 0;
}

int init_INTs(void)
{
    // PD2 and PD3 as inputs
    DDRD &= ~((1 << DDD2) | (1 << DDD3));

    // internal pull-ups
    PORTD |= (1 << PD2) | (1 << PD3);

    // enable pin change on PD2 (PCINT18) and PD3 (PCINT19)
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);

    // enable PCINT group 2
    PCICR  |= (1 << PCIE2);

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
    uint32_t temp = K_RPM_Q / (uint32_t)ticks;

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
	received_data = UDR0;
	new_cmd = 1;
}


