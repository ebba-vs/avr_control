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
#include <stdio.h>
#include <stdlib.h> // dtostrf
#include <util/delay.h>

void init_LEDs(void);
void init_INTs(void);
void init_PWM(void);
int set_LED(int position, int value);
void set_PWM(int direction);
void USART_Transmit(const char c);
void USART_Transmit(const char c);
void USART_Init(unsigned int ubrr);
unsigned char get_rpm_q8_7(uint16_t dt_tick);

#define EDGES_PER_REV 96UL
#define TIMER1_PRESCALER 8UL
#define n 7 // Q8.7 -> 7 fractional bits
#define Q_SCALE (1 << n)
#define BAUD 2400
#define MYUBRR ((F_CPU / 16 / BAUD) - 1)
#define MAX_TICK_COUNT 15625 // equal to 5 rpm (min)
#define MIN_TICK_COUNT 651   // equal to 120 rpm (max)
#define MAX_RPM_Q 15744      // 123*2^7, max_rpm in fixed point

uint8_t duty = 255;
uint8_t flag = 0;

volatile uint16_t dt_ticks = 0;     // ticks between encoder edges
volatile uint16_t motorspeed_q = 0; // RPM in Q8.7

volatile uint8_t n_ticks = 0;
volatile uint32_t sum = 0;
volatile uint16_t dt_tick = 0; // ticks since last edge

volatile uint8_t received_data = 0;
volatile uint8_t new_cmd = 0;
volatile uint8_t motorspeed = 0;

volatile unsigned char rpm = 0;

volatile uint16_t last_tick = 0;

// 60 * F_CPU * 2^7 / (prescaler * edges)
static const uint32_t K_rpm =
    (uint32_t)((60ULL * F_CPU * (uint32_t)Q_SCALE) /
               (TIMER1_PRESCALER *
                EDGES_PER_REV)); // 78125 for ps=8, F_CPU=1Mhz

void USART_Init(unsigned int ubrr) {
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;

  DDRD |= (1 << PD1); // TXD as output

  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable transmitter
  UCSR0C = (0 << USBS0) | (3 << UCSZ00);                // 8N1
}

void USART_Transmit(const char c) {
  while (!(UCSR0A & (1 << UDRE0)))
    ; // Wait until buffer empty
  UDR0 = c;
}

void uart_send_string(const char *s) {
  while (*s)
    USART_Transmit(*s++);
}

void uart_print_rpm(uint8_t rpm) {
  // int8_t rpm = rpm_q8_7 >> n; // Q8.7 -> float

  char str[20];
  sprintf(str, "%d", rpm);
  uart_send_string(str);
  uart_send_string("\r\n");
}

void timer1_init() {
  TCCR1A = 0; // normal mode
  TCCR1B = 0;
  TCCR1B = (1 << CS11); // prescaler = 8
  TCNT1 = 0;
}
void init_LEDs() {

  DDRB |= (1 << PB7);
  DDRC |= (1 << PC4);
  DDRD |= (1 << PD7);
  PORTB &= ~(1 << PB7);
  PORTC &= ~(1 << PC4);
  PORTD &= ~(1 << PD7);
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

void init_INTs() {
  DDRD &= ~((1 << DDD2) | (1 << DDD3)); // PD2 and PD3 as inputs
  PCICR |= (1 << PCIE2);                // enable PCINT group 2
  PCMSK2 |=
      (1 << PCINT18) |
      (1 << PCINT19); // enable pin change on PD2 (PCINT18) and PD3 (PCINT19)
  PORTD |= (1 << PD2) | (1 << PD3); // internal pull-ups
}
void init_PWM() {
  // PD6 (OC0A) as output
  DDRD |= (1 << DDD6);

  TCCR0A = 0;
  TCCR0B = 0;

  TCCR0A |= (1 << WGM00);

  // Non-inverting PWM on OC0A: COM0A1:0 = 0b10
  TCCR0A |= (1 << COM0A1);
  TCCR0A &= ~(1 << COM0A0);

  // Prescaler: clk/8  (CS01 = 1)
  // F_PWM = F_CPU / (2 * N * 256) = 1MHz / (2*8*256) ? 244 Hz
  TCCR0B |= (1 << CS01); // no prescaling

  OCR0A = 100;
  set_LED(2, 1);
}

void set_PWM(int direction) {
  if (direction == 1) {
    if (OCR0A <= 255 - 5) { // avoid overflow
      set_LED(2, 1);
      OCR0A += 5;
    }
  } else if (direction == -1) {
    set_LED(2, 0);
    if (OCR0A >= 5) { // avoid underflow
      OCR0A -= 5;
    }
  }
}

// unsigned char get_rpm_q8_7(uint16_t cnt) {
// set_LED(3,1);
// uint16_t ticks;
//
// cli();
// ticks = dt_tick;
// sei();
//
// if (ticks == 0) {
// return 0;
//}
//
// 		uint32_t temp = K_rpm;
// temp = temp + (ticks >> 1);		// round
// uint16_t temp16 = temp / ticks;			// Division Q25.7 /
// Q16.0
//// uint16_t temp16 = (uint16_t) temp;		//Truncate Q8.7
//
//// saturate to int8_t range
// if (temp16 > INT16_MAX) {
// temp16 = INT16_MAX;
//}
// unsigned char rpm = (uint8_t) temp16;
////uint8_t rpm = temp16 >> n;		// Convert to integer Q16.0
// return rpm;		// Q8.0 rpm
//}
int updatePWM(int value) {
  // OCR0A = value; //Motor pwm
  // rpm=1; //Set for test with fixed PWM

  OCR0A = value; // Control led
  return value;
}
int main(void) {
  // int sp;
  // int8_t serialin, serialout;

  init_LEDs();
  init_PWM();

  USART_Init(MYUBRR);
  init_INTs();

  timer1_init();

  sei();

  while (1) {
    // USART_Transmit(a);
    //_delay_ms(1000);

    if (new_cmd) {
      uint8_t cmd = received_data; // copy to local
      // uart_print_rpm(motorspeed);
      // uart_print_rpm(motorspeed);

      if (cmd > 0) {
        set_PWM(1);
      } else {
        set_PWM(-1);
      }
      new_cmd = 0;
    }
  }
}
volatile uint8_t debug_counter = 0;

ISR(PCINT2_vect) {
  debug_counter++; // Count every interrupt

  set_LED(3, 1);
  uint16_t current_tick = TCNT1;
  // Unsigned subtraction handles overflow automatically
  uint16_t delta = current_tick - last_tick;

  if (delta > MAX_TICK_COUNT) {
    delta = MAX_TICK_COUNT;
  }
  if (delta < MIN_TICK_COUNT) {
    delta = MIN_TICK_COUNT;
  }
  sum += delta;
  n_ticks++;

  if (n_ticks >= 1) { // Reduced averaging to 1 for faster response
    dt_tick = sum / n_ticks;
    sum = 0;
    n_ticks = 0;
  }
  last_tick = current_tick;
}

ISR(USART_RX_vect) {
  set_LED(1, 0);
  received_data = (uint8_t)UDR0;

  // Send DEBUG COUNTER instead of RPM
  USART_Transmit(debug_counter);

  new_cmd = 1; // signal main
}
