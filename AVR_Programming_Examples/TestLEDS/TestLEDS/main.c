/*
 * TestINT.c
 *
 * EIEN65 Applied Mechatronics 2023
 *  Author: FranM
 */

// Includes and definitions
#define F_CPU 1000000      // Needed for the "delay" function
#include <avr/interrupt.h> //We need these definitions to be able to use interrupts
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

// Function declaration
int init_LEDs(void);
int init_INTs(void);
void init_PWM(void);
int set_LED(int position, int value);
void set_PWM(int direction);

volatile uint8_t prev_state = 0;

int main(void) {
  // Local variables declaration
  char ChannelA, ChannelB;
  int8_t TOP = 255;
  int8_t BOTTOM = 0;

  init_LEDs();

  // Keep interrupts disabled for now (pure polling test)
  init_INTs();
  init_PWM();
  sei();

  /* Uncomment these two lines to use interrupts instead of polling...
  init_INTs();
sei();    // set global interrupt flag -> enable ALL interrupts (but those
that are masked won't do anything)
  */

  while (1) {
    // Polling... - Comment out these four lines when running with interrupts.
    // ChannelA = PIND & (1<<PIND2);	// Bits 2 and 3 are the ones with the
    // sensor information ChannelB = PIND & (1<<PIND3); set_LED(2,ChannelA);
    // set_LED(3,ChannelB);

    // Flash an LED while waiting...
    set_LED(1, 1);
    //_delay_ms(1000);
  }
}

ISR(PCINT2_vect) {
  uint8_t this_state = (PIND & 0x0C); // mask for PD2 + PD3

  // Ignore if no change (bouncing or multiple interrupts)
  if (this_state == prev_state) {
    return;
  }

  // Forward sequence: 0 -> 8 -> 12 -> 4 -> 0
  if (prev_state == 0 && this_state == 8) {
    set_PWM(1);
  } else if (prev_state == 8 && this_state == 12) {
    set_PWM(1);
  } else if (prev_state == 12 && this_state == 4) {
    set_PWM(1);
  } else if (prev_state == 4 && this_state == 0) {
    set_PWM(1);
  }
  // Backward sequence: 0 -> 4 -> 12 -> 8 -> 0
  else if (prev_state == 0 && this_state == 4) {
    set_PWM(-1);
  } else if (prev_state == 4 && this_state == 12) {
    set_PWM(-1);
  } else if (prev_state == 12 && this_state == 8) {
    set_PWM(-1);
  } else if (prev_state == 8 && this_state == 0) {
    set_PWM(-1);
  }
  // Any other weird jump: ignore
  else {
    // do nothing
  }
  prev_state = this_state;

  // This is the exact same code as used when polling!
  set_LED(1, 0);

  if ((PIND & (1 << PIND2))) {
    set_LED(2, 1);
  } else {
    set_LED(2, 0);
  }
  if ((PIND & (1 << PIND3))) {
    set_LED(3, 1);
  } else {
    set_LED(3, 0);
  }
}
void set_PWM(int direction) {
  if (direction == 1) {
    if (OCR0A <= 255 - 5) { // avoid overflow
      OCR0A += 5;
    }
  } else if (direction == -1) {
    if (OCR0A >= 5) { // avoid underflow
      OCR0A -= 5;
    }
  }
}

int init_LEDs(void) {
  DDRB |= (1 << PB7);
  DDRC |= (1 << PC4); // Corresponding pins set as outputs 0b 0000 1111
  DDRD |= (1 << PD7);
  PORTB &= ~(1 << PB7);
  PORTC &= ~(1 << PC4); // Initially all LED pins set to 0
  PORTD &= ~(1 << PD7);

  return 1;
}

int init_INTs(void) {
  // PD2 and PD3 as inputs
  DDRD &= ~((1 << DDD2) | (1 << DDD3));

  // Optional: internal pull-ups if the encoder has open switches
  PORTD |= (1 << PD2) | (1 << PD3);

  // Enable pin change interrupts on PD2 (PCINT18) and PD3 (PCINT19)
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);

  // Enable pin change interrupt group 2 (PCINT[23:16] = Port D)
  PCICR |= (1 << PCIE2);

  return 1;
}

void init_PWM(void) {
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
  TCCR0B |= (1 << CS01);

  // Start with 0% duty
  OCR0A = 0;
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
