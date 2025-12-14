/*
 * AVR USART (Serial) Communication Functions
 * These functions initialize the hardware and handle basic data transfer.
 * Assumes an ATmega series microcontroller (e.g., ATmega328P).
 */

// Includes and definitions
#define F_CPU 1000000UL // Clock Speed: 1 MHz (Crucial for Baud Rate Calculation)
#include <avr/io.h>     // Required for AVR registers (UBRR0H, UCSR0A, etc.)
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define FOSC 1000000UL // Clock Speed (alternative definition for formula)
#define BAUD 2400      // Desired Baud Rate (must match your PC program!)
#define MYUBRR (FOSC / 16 / BAUD - 1)

#define CMD_BUF_SIZE 16

#define RPM_SAMPLES 16
#define PRESCALER 8L       // Timer 1 Prescaler
#define EDGES_PER_REV 96UL // Encoder constant

// Q4 format for rpm_calcualte
#define RPM_Q_SHIFT 4
#define ONE_Q4 (1UL << RPM_Q_SHIFT) // Scaling factor 16

// Q16.16 format for PI-controller
#define CONTROL_Q_SHIFT 16
#define ONE_Q16 (1UL << CONTROL_Q_SHIFT) // Scaling factor 65536
// Constants for PI-controller, scaled to Q16.16 format
const int32_t Ts_Q = 1311;                  // Sample Time Ts = 0.02. Value is 1311 when shifted << 16 (0.02 * 65536 = 1311)
const int32_t MAX_PWM_Q = (255L * ONE_Q16); // Max PWM stored in Q16.16, (255 * 65536)
const int32_t Kp_Q = (10L * ONE_Q16);       // Proportional part scaled to Q16.16
const int32_t Ki_Q = (3L * ONE_Q16);        // Integral part scaled to Q16.16

// TX Buffer for Non-Blocking Transmission
#define TX_BUFFER_SIZE 256
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;

int set_LED(int position, int value);
int init_INTs(void);
int init_LEDs(void);
int initPWM(void);
int init_Timer1_RPM(void);
int init_Timer2_Control(void);
int updatePWM(int value);
int init_ADC(void);
uint16_t read_ADC(void);
int compute_rpm(void);
int regulate_speed(int target, int measured);
void setup(void);
void handle_command(int cmd);
void USART_Init(unsigned int ubrr);
void USART_Transmit_String(const char *s);

volatile long encoderCount = 0;
volatile int target_rpm = 0;
volatile uint16_t raw_pot = 0; // reads the raw int from potenmtiometer

volatile uint16_t last_time = 0;

// Used in encoder ISR
volatile uint16_t delta_times[RPM_SAMPLES];
volatile uint8_t delta_idx = 0;
volatile uint8_t prevAB = 0;

volatile uint8_t controller_flag = 0; // Updates in ISR(TIMER2) flags control-loop

// UART for RX
volatile uint8_t new_cmd_flag = 0;
volatile uint8_t received_byte = 0;

volatile int base_target_rpm = 0; // the digital setpoint for rpm

volatile int motor_direction = 1; // 1 for forward, -1 for reverse

// initialize USART pins
void USART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// initialize Timer1 for rpm calc
int init_Timer1_RPM(void)
{
    TCCR1A = 0;
    TCCR1B = 0;

    // prescaler 8
    TCCR1B |= (1 << CS11);

    TCNT1 = 0;

    return 1;
}

// initialize Timer2
int init_Timer2_Control(void)
{
    PRR &= ~(1 << PRTIM2);              // Enable Timer2
    TCCR2A = (1 << WGM21);              // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS20); // Prescaler 128
    OCR2A = 155;                        // ~20 ms period at 1 MHz
    TCNT2 = 0;
    TIMSK2 |= (1 << OCIE2A); // Enable Compare Match A interrupt
    return 1;
}

// ska uppdatera controller_flag, hänger på Timer2
ISR(TIMER2_COMPA_vect)
{
    controller_flag = 1;
}

// initialize LEDs
int init_LEDs(void)
{
    DDRC |= (1 << PC3 | 1 << PC4);
    DDRD |= (1 << PD7);

    PORTC &= ~(0x0F);
    PORTD &= ~(1 << PD7);

    return 1;
}

// initialize external interrupts (for encoder)
int init_INTs(void)
{
    DDRC &= ~((1 << PC0) | (1 << PC1));
    PORTC |= (1 << PC0) | (1 << PC1);

    prevAB = ((PINC & (1 << PC0)) ? 2 : 0) | ((PINC & (1 << PC1)) ? 1 : 0);

    PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);
    PCICR |= (1 << PCIE1);

    sei();
    return 1;
}

// Initialize Analog-to-Digital Converter (ADC) for PC5
int init_ADC(void)
{
    DDRC &= ~(1 << DDC5); // enable PC5 as a input

    ADMUX = (1 << REFS0) | 5; // sets VRef to VCC and enables ADC on PC5

    ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC (ADEN) and sets prescaler to 8 -> 125 kHz, safe range 50k - 200k

    // Perform a dummy read to stabilize the ADC
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;

    return 1;
}

// Read the ADC value from a specified channel (0-5, PC5 is channel 5)
uint16_t read_ADC(void)
{
    ADCSRA |= (1 << ADSC); // start the conversion, voltage to int

    while (ADCSRA & (1 << ADSC))
        ; // wait for the conversion to complete (ADSC goes low)

    return ADC; // return the 10-bit result, as a int
}

// initialize PWM
int initPWM(void)
{
    // 1. Set PD5 and PD6 as outputs (OC0B and OC0A)
    DDRD |= ((1 << DDD5) | (1 << DDD6));

    TCCR0A = (1 << COM0A1) | (1 << COM0A0)   // COM0A = Inverting mode
             | (1 << COM0B1) | (1 << COM0B0) // COM0B = Inverting mode
             | (1 << WGM01) | (1 << WGM00);

    TCCR0B = (1 << CS00);

    OCR0A = 0;
    OCR0B = 0;

    return 1;
}

// updates PWM-signal
int updatePWM(int speed_value)
{
    // Clamp
    if (speed_value < 0)
        speed_value = 0;
    if (speed_value > 255)
        speed_value = 255;

    // With Hardware Inversion enabled in initPWM:
    // OCR = 0 -> Pin HIGH (Stop for your driver)
    // OCR = 255 -> Pin LOW (Full speed for your driver)

    if (speed_value == 0)
    {
        OCR0A = 0;
        OCR0B = 0; // BRAKE
    }
    else if (motor_direction == 1)
    {
        OCR0A = speed_value;
        OCR0B = 0;
    }
    else
    {
        OCR0A = 0;
        OCR0B = speed_value;
    }
    return speed_value;
}
// sets LEDs on/off
int set_LED(int position, int value)
{
    switch (position)
    {
    case 1:
        if (value == 0)
            PORTD &= ~(1 << PD7);
        else
            PORTD |= (1 << PD7);
        break;
    case 2:
        if (value == 0)
            PORTC &= ~(1 << PC3);
        else
            PORTC |= (1 << PC3);
        break;
    case 3:
        if (value == 0)
            PORTB &= ~(1 << PB0);
        else
            PORTB |= (1 << PB0);
        break;
    case 4:
        if (value == 0)
            PORTC &= ~(1 << PC4);
        else
            PORTC |= (1 << PC4);
        break;
    }
    return 1;
}

void handle_command(int cmd)
{

    int new_rpm = cmd;

    if (new_rpm > 130) // upper boundry
        new_rpm = 130;

    base_target_rpm = new_rpm;
    target_rpm = base_target_rpm; //
}

// RPM calc
int compute_rpm(void)
{
    uint32_t sum = 0;
    uint16_t safe_deltas[RPM_SAMPLES];

    // Atomic block to copy data
    cli();
    memcpy((void *)safe_deltas, (void *)delta_times, sizeof(delta_times));
    sei();

    for (int i = 0; i < RPM_SAMPLES; i++)
    {
        // Filter out noise or very long delays (stopped motor)
        if (safe_deltas[i] < 60000 && safe_deltas[i] > 0)
            sum += safe_deltas[i];
    }

    if (sum == 0)
        return 0;

    uint32_t avg_dt = sum / RPM_SAMPLES;

    uint32_t rpm_top = 60UL * F_CPU;
    uint32_t rpm_bottom = (uint32_t)PRESCALER * EDGES_PER_REV * avg_dt;

    if (rpm_bottom == 0)
        return 0;

    // Fixed-Point Arithmetic
    uint32_t scaled_top_rpm = rpm_top << RPM_Q_SHIFT;
    uint32_t rpm_q4 = scaled_top_rpm / rpm_bottom;

    // Converting Fixed-Point result back to integer (for the display!)
    uint32_t rounded_current_rpm = (rpm_q4 + (1 << (RPM_Q_SHIFT - 1))) >> RPM_Q_SHIFT; // RPM in FxP + (q-1) = 0.5, shifted back to integer rpm

    return rounded_current_rpm;
}

// PI-controller
int regulate_speed(int target, int measured)
{
    static int32_t integral_q = 0;

    // if target set to 0, reset integral_q
    if (target == 0)
    {
        integral_q = 0;
        return 0;
    }

    int error = target - measured;
    int32_t error_q = ((int32_t)error) << CONTROL_Q_SHIFT; // Error scaled to Q16.16

    int32_t P_q = (int32_t)(((int64_t)Kp_Q * (int64_t)error_q) >> CONTROL_Q_SHIFT); // P-part (64-bit multiplication for precision)

    // Calc I-part (but don't add to output until we check the windup)
    // note dubble downshift becuase we multiply Q16 * Q16 * Q16
    int32_t delta_i = (int32_t)(((int64_t)error_q * (int64_t)Ts_Q * (int64_t)Ki_Q) >> (CONTROL_Q_SHIFT + CONTROL_Q_SHIFT));

    // temporary calculation without new integral
    int32_t control_q_pre = P_q + integral_q;

    // Anti-windup, only increase the integral if we hit the max PWM
    bool saturating_high = (control_q_pre > MAX_PWM_Q);
    bool saturating_low = (control_q_pre < 0);

    // If we hit max pwm, we can only integrate if the error is negative, (hit the brake)
    // if we hit 0, we can only integrate if the error is positive, (hit the gas)
    if (!saturating_high && !saturating_low)
    {
        integral_q += delta_i;
    }
    else if (saturating_high && error < 0)
    {
        integral_q += delta_i;
    }
    else if (saturating_low && error > 0)
    {
        integral_q += delta_i;
    }

    // CLAMP. integral can never be greater than max pwm
    if (integral_q > MAX_PWM_Q)
        integral_q = MAX_PWM_Q;
    if (integral_q < 0)
        integral_q = 0;

    int32_t final_control_q = P_q + integral_q;

    // Clamp Output 0-255
    if (final_control_q < 0)
        final_control_q = 0;
    if (final_control_q > MAX_PWM_Q)
        final_control_q = MAX_PWM_Q;

    return (int)((final_control_q + (1 << (CONTROL_Q_SHIFT - 1))) >> CONTROL_Q_SHIFT);
}

// Encoder (ISR)
ISR(PCINT1_vect)
{
    uint8_t A = (PINC >> PC0) & 1;
    uint8_t B = (PINC >> PC1) & 1;
    uint8_t AB = (A << 1) | B;

    if (AB != prevAB)
    {
        if ((prevAB == 0b00 && AB == 0b01) ||
            (prevAB == 0b01 && AB == 0b11) ||
            (prevAB == 0b11 && AB == 0b10) ||
            (prevAB == 0b10 && AB == 0b00))
            encoderCount++;
        else
            encoderCount--;

        delta_times[delta_idx++] = TCNT1 - last_time;
        if (delta_idx >= RPM_SAMPLES)
            delta_idx = 0;
        last_time = TCNT1;
    }

    prevAB = AB;
}

// UART TX STRING
void USART_Transmit_String(const char *s)
{
    uint8_t i = 0;
    uint8_t next_tail;

    // Loop through every character in the input string
    while (s[i] != '\0')
    {

        do
        {
            next_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        } while (next_tail == tx_head); // Wait while the buffer is full (i.e., next_tail = tx_head)

        cli();

        tx_buffer[tx_tail] = s[i++];
        tx_tail = next_tail;

        // Ensure the Data Register Empty Interrupt is enabled to start transmission
        UCSR0B |= (1 << UDRIE0);

        sei();
    }
}

// USART output (ISR)
ISR(USART_UDRE_vect)
{
    if (tx_head != tx_tail)
    {
        UDR0 = tx_buffer[tx_head];
        tx_head = (tx_head + 1) % TX_BUFFER_SIZE;
    }
    else
    {
        // Buffer is empty, stop the interrupt
        UCSR0B &= ~(1 << UDRIE0);
    }
}

// USART input (ISR)
ISR(USART_RX_vect)
{
    received_byte = UDR0;

    new_cmd_flag = 1;
}

// All INITs
void setup(void)
{
    USART_Init(MYUBRR);
    init_INTs();
    init_LEDs();
    initPWM();
    init_Timer1_RPM();
    init_Timer2_Control();
    init_ADC();

    sei();
}

int main(void)
{
    setup();

    char msg_buffer[64]; // buffer for building messages to transmit

    while (1)
    {

        if (new_cmd_flag)
        {
            new_cmd_flag = 0;

            if (received_byte == 255) // if input was "255/c ", debugging
            {
                uint32_t current_rpm = compute_rpm();
                int target = target_rpm;
                uint8_t current_pwm = (motor_direction == 1) ? OCR0A : OCR0B;

                sprintf(msg_buffer, "RPM:%lu Tgt:%d Pot:%hu PWM:%u\n",
                        current_rpm, target, raw_pot, current_pwm); // builds a full message in msg_buffer to be sent to by transmit-method
            }

            else
            {
                handle_command(received_byte);
                sprintf(msg_buffer, "RPM set to: %u\n", received_byte); // builds a full message in msg_buffer to be sent to by transmit-method
            }

            USART_Transmit_String(msg_buffer);
        }

        if (controller_flag) // controller
        {

            controller_flag = 0; // reset flag

            raw_pot = read_ADC(); // reads the potentiometer, returns int between 0 – 1023, corresponding to 0 - 5 V

            int16_t centered = (int16_t)raw_pot - 512; // approx -512 < centered < +511

            int32_t offset = ((int32_t)centered * 10) / 512; // calc offset with rounding: offset = round(centered * 10 / 512)

            int new_target = base_target_rpm + (int)offset;

            // clamp
            if (new_target < 0)
                new_target = 0;
            if (new_target > 130)
                new_target = 130;

            target_rpm = new_target;

            int rpm = compute_rpm();
            int pwm = regulate_speed(target_rpm, rpm);
            updatePWM(pwm);
        }
    }
}