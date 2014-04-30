/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this moduls maps the receiver pulse width to an appropriate motor action
 * @file control.cpp
 */

#include "control.h"
#include "motor.h"
#include "linear_mapper.h"
#include "defines.h"

#include <avr/interrupt.h>

/* GLOBAL CONSTANT SECTION */
static uint8_t const DEADZONE = 10;
static uint16_t const CH1_PULSE_WIDTH_MIN_US = 1120;
static uint16_t const CH1_PULSE_WIDTH_MAX_US = 1910;
static uint16_t const CH2_PULSE_WIDTH_MIN_US = 1120;
static uint16_t const CH2_PULSE_WIDTH_MAX_US = 1910;

/* GLOBAL VARIABLE SECTION */
typedef struct {
	uint16_t pulse_width_us;
	uint16_t deadzone;
	bool is_updated;
} s_channel;
static s_channel m_channel_1 = {1500, DEADZONE, false};
static s_channel m_channel_2 = {1500, DEADZONE, false};

// the mappers map the channel input to motor speed values
static linear_mapper m_mapper_channel_1(CH1_PULSE_WIDTH_MIN_US, CH1_PULSE_WIDTH_MAX_US, (-1)*(1<<14), (1<<14));
static linear_mapper m_mapper_channel_2(CH2_PULSE_WIDTH_MIN_US, CH2_PULSE_WIDTH_MAX_US, (-1)*(1<<14), (1<<14));

/* PROTOTYPE SECTION */
int16_t abs(int16_t const val);


/* FUNCTION SECTION */

/** 
 * @brief updates channel 1 - only to be called within isr context
 */
void control::update_channel_1(uint16_t const ch1_pulse_width_us) {
	m_channel_1.pulse_width_us = ch1_pulse_width_us;
	m_channel_1.is_updated = true;
}	
	
/** 
 * @brief updates channel 2 - only to be called within isr context
 */
void control::update_channel_2(uint16_t const ch2_pulse_width_us) {
	m_channel_2.pulse_width_us = ch2_pulse_width_us;
	m_channel_2.is_updated = true;
}

/** 
 * @brief this function is called by the input module to indicate that channels have been lost - only to be called within isr context
 */
void control::channel_s_lost() {
	motor::set_direction(BREAK);
	motor::set_speed(0);
	m_channel_1.is_updated = false;
	m_channel_2.is_updated = false;
}
	
	
/** 
 * @brief the control task itself - it is scheduled to be runnable after both channels have updated their data value
 */
void control::run() {
	// ch1 is forward/backward
	// ch2 is left/right
	
	// perform the mapping and correct the result by shifting 6 right -> resulting area is between -255 and +255 
	int16_t ch1_motor_value = (m_mapper_channel_1.map((int16_t)(m_channel_1.pulse_width_us)) / (1<<6));
	int16_t ch2_motor_value = (m_mapper_channel_2.map((int16_t)(m_channel_2.pulse_width_us)) / (1<<6));
	
	// simple delta mixing
	ch1_motor_value += (ch2_motor_value / 2);
	
	// limit to +/- 255
	if(ch1_motor_value > 255) ch1_motor_value = 255;
	if(ch1_motor_value < -255) ch1_motor_value = -255;
	
	if(ch1_motor_value >= 0) {
		if((uint16_t)(ch1_motor_value) > m_channel_1.deadzone) {
			motor::set_speed((uint8_t)(ch1_motor_value));
#ifdef RIGHT_MOTOR
			motor::set_direction(FORWARD);
#endif
#ifdef LEFT_MOTOR
			motor::set_direction(BACKWARD);
#endif
			} else {
			motor::set_speed(0);
			motor::set_direction(BREAK);
		}
	} else {
		ch1_motor_value = abs(ch1_motor_value);
		if((uint16_t)(ch1_motor_value) > m_channel_1.deadzone) {
			motor::set_speed((uint8_t)(ch1_motor_value));
#ifdef RIGHT_MOTOR
			motor::set_direction(BACKWARD);
#endif
#ifdef LEFT_MOTOR
			motor::set_direction(FORWARD);
#endif
			} else {
			motor::set_speed(0);
			motor::set_direction(BREAK);
		}
	}

	// task has been executed, clear the runnable flags
	cli();
	m_channel_1.is_updated = false;
	m_channel_2.is_updated = false;
	sei();
}	
	
/** 
 * @brief returns true if the control task is runnable in the main thread
 */
bool control::is_runnable() {
	cli();
	bool const runnable = (m_channel_1.is_updated && m_channel_2.is_updated);
	sei();
	return runnable;
}

/** 
 * @brief returns |val|
 */
int16_t abs(int16_t const val) {
	int16_t res = 0;
	val < 0 ? res = (0 - val) : res = val;
	return res;
}