/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module contains the sampling of the pwm channel inputs
 * @file input.cpp
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include "input.h"
#include "control.h"
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* DEFINE SECTION */
// CH1 = D2 = INT0 = PD2
#define CH1_DDR		(DDRD)
#define CH1_PORT	(PORTD)
#define CH1			(1<<2)
// CH2 = D3 = INT1 = PD3
#define CH2_DDR		(DDRD)
#define CH2_PORT	(PORTD)
#define CH2			(1<<3)

/* MACRO_SECTION */
#define CH1_TRIGGER_AT_RISING_EDGE()  do { EICRA &= 0x0C; EICRA |= (1<<ISC01) | (1<<ISC00); } while(0)
#define CH1_TRIGGER_AT_FALLING_EDGE() do { EICRA &= 0x0C; EICRA |= (1<<ISC01); } while(0)
#define CH2_TRIGGER_AT_RISING_EDGE()  do { EICRA &= 0x03; EICRA |= (1<<ISC11) | (1<<ISC10); } while(0)
#define CH2_TRIGGER_AT_FALLING_EDGE() do { EICRA &= 0x03; EICRA |= (1<<ISC11); } while(0)

/* GLOBAL CONSTANTS */
static uint8_t const MIN_PULSES_PER_TIMER_CYCLE = 10; // 262 ms / 20 ms = 13 (-3 to give a little room for error)
static uint16_t const MIN_PULSE_WIDTH_US = 1000;
static uint16_t const MAX_PULSE_WIDTH_US = 2000;

/* TYPEDEFS */
typedef enum {RISING, FALLING} E_PULSE_STATE;
typedef struct {
	E_PULSE_STATE pulse_state;
	uint8_t pulses_received;
} s_pulse_property;

/* GLOBAL VARIABLES */
static s_pulse_property m_ch1_pulse = {RISING, 0};
static s_pulse_property m_ch2_pulse = {RISING, 0};
	
/* FUNCTIONS */

/**
 * @brief initializes the input module
 */
void input::init() {
	// set the channel pins as input ...
	CH1_DDR &= ~CH1;
	CH2_DDR &= ~CH2;
	// ... with pullup
	CH1_PORT |= CH1;
	CH2_PORT |= CH2;
	// set up the trigger
	CH1_TRIGGER_AT_RISING_EDGE();
	CH2_TRIGGER_AT_RISING_EDGE();
	// enable external interrupts
	EIMSK = (1<<INT1) | (1<<INT0);
	
	// clear timer
	TCNT1 = 0;
	// enable timer 1 overflow interrupt
	TIMSK1 = (1<<TOIE1);
	// prescaler = 8
	// fTimer = fCPU / 64 = 16 MHz / 64 = 250 kHz
	// tTimerStep = 4 us
	// 2^16 * tTimerStep = 262.144 ms
	TCCR1B = (1<<CS11) | (1<<CS10);
}

/** 
 * @brief int0 (ch1) interrupt service routine
 */
ISR(INT0_vect) {
	static uint16_t start = 0;
	static uint16_t stop = 0;
	
	if(m_ch1_pulse.pulse_state == RISING) {
		start = TCNT1;
		m_ch1_pulse.pulse_state = FALLING;
		CH1_TRIGGER_AT_FALLING_EDGE();
	} else if(m_ch1_pulse.pulse_state == FALLING) {
		stop = TCNT1;
		m_ch1_pulse.pulse_state = RISING;
		CH1_TRIGGER_AT_RISING_EDGE();
		
		m_ch1_pulse.pulses_received++;
		
		uint16_t const pulse_duration_in_timer_steps = stop - start;
		uint16_t const timerstep_duration_in_us = 4;
		uint16_t const pulse_duration_in_us = pulse_duration_in_timer_steps * timerstep_duration_in_us;
		
		// only update when the value is within acceptable bounds
		if(pulse_duration_in_us >= MIN_PULSE_WIDTH_US && pulse_duration_in_us <= MAX_PULSE_WIDTH_US) {
			control::update_channel_1(pulse_duration_in_us);
		}
	}
}

/** 
 * @brief int1 (ch2) interrupt service routine
 */
ISR(INT1_vect) {
	static uint16_t start = 0;
	static uint16_t stop = 0;
	
	if(m_ch2_pulse.pulse_state == RISING) {
		start = TCNT1;
		m_ch2_pulse.pulse_state = FALLING;
		CH2_TRIGGER_AT_FALLING_EDGE();
	}  else if(m_ch2_pulse.pulse_state == FALLING) {
		stop = TCNT1;
		m_ch2_pulse.pulse_state = RISING;
		CH2_TRIGGER_AT_RISING_EDGE();
		
		m_ch2_pulse.pulses_received++;
		
		uint16_t const pulse_duration_in_timer_steps = stop - start;
		uint16_t const timerstep_duration_in_us = 4;
		uint16_t const pulse_duration_in_ms = pulse_duration_in_timer_steps * timerstep_duration_in_us;
		
		// only update when the value is within acceptable bounds
		if(pulse_duration_in_ms >= MIN_PULSE_WIDTH_US && pulse_duration_in_ms <= MAX_PULSE_WIDTH_US) {
			control::update_channel_2(pulse_duration_in_ms);
		}
	}
}

/** 
 * @brief timer 1 overflow interrupt service routine
 */
ISR(TIMER1_OVF_vect) {
	// in case there has occured a loss of pulses ...
	bool const ch1_pulses_lost = m_ch1_pulse.pulses_received < MIN_PULSES_PER_TIMER_CYCLE;
	bool const ch2_pulses_lost = m_ch2_pulse.pulses_received < MIN_PULSES_PER_TIMER_CYCLE;
	if(ch1_pulses_lost || ch2_pulses_lost) {
		// inform the control unit about it
		control::channel_s_lost();
		// update the channel information
		m_ch1_pulse.pulse_state = RISING;
		CH1_TRIGGER_AT_RISING_EDGE();
		m_ch2_pulse.pulse_state = RISING;
		CH2_TRIGGER_AT_RISING_EDGE();
	}
	// clear the pulse counters
	m_ch1_pulse.pulses_received = 0;
	m_ch2_pulse.pulses_received = 0;
}