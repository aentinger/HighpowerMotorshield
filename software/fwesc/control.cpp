/**
* @author Alexander Entinger, MSc / LXRobotics
* @brief this moduls maps the receiver pulse width to an appropriate motor action
* @file control.cpp
* @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
*/

#include "control.h"
#include "motor.h"
#include "linear_mapper.h"
#include "defines.h"

#include <avr/interrupt.h>
#include <avr/io.h>

/* GLOBAL CONSTANT SECTION */
static uint8_t const DEADZONE = 20;
static uint16_t const CH1_PULSE_WIDTH_MIN_US = 1120;
static uint16_t const CH1_PULSE_WIDTH_MAX_US = 1910;
static uint16_t const CH2_PULSE_WIDTH_MIN_US = 1120;
static uint16_t const CH2_PULSE_WIDTH_MAX_US = 1910;

/* TYPEDEF SECTION */
typedef struct {
	uint16_t pulse_width_us;
	uint16_t neutral_pulse_width_us;
	uint16_t deadzone;
	bool is_updated;
} s_channel;

typedef struct {
	bool calibration_complete;
} s_control_data;

/* GLOBAL VARIABLE SECTION */
static s_channel m_channel_1 = {1500, 1500, DEADZONE, false};
static s_channel m_channel_2 = {1500, 1500, DEADZONE, false};
static s_control_data m_control_data = {false};

// the mappers map the channel input to motor speed values
static linear_mapper m_mapper_1_channel_1(CH1_PULSE_WIDTH_MIN_US, m_channel_1.neutral_pulse_width_us, (-1)*(1<<14), 0);
static linear_mapper m_mapper_2_channel_1(m_channel_1.neutral_pulse_width_us, CH1_PULSE_WIDTH_MAX_US, 0, (1<<14));
static linear_mapper m_mapper_1_channel_2(CH2_PULSE_WIDTH_MIN_US, m_channel_2.neutral_pulse_width_us, (-1)*(1<<14), 0);
static linear_mapper m_mapper_2_channel_2(m_channel_2.neutral_pulse_width_us, CH2_PULSE_WIDTH_MAX_US, 0, (1<<14));

/* PROTOTYPE SECTION */
int16_t abs(int16_t const val);
void calibrate();
void sort(uint16_t *data, uint8_t const length);
void swap(uint16_t *elem1, uint16_t *elem2);
uint16_t median(uint16_t const *data, uint8_t const length);


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
	
	if(m_control_data.calibration_complete) {
		
		// calibration is complete, perform the control
		
		// ch1 is forward/backward
		// ch2 is left/right
		
		int16_t ch1_motor_value = 0, ch2_motor_value = 0;
		
		// perform the mapping and correct the result by shifting 6 right -> resulting area is between -255 and +255
		if(m_channel_1.pulse_width_us < m_channel_1.neutral_pulse_width_us) {
			ch1_motor_value = (m_mapper_1_channel_1.map((int16_t)(m_channel_1.pulse_width_us)) / (1<<6));
		} else if(m_channel_1.pulse_width_us > m_channel_1.neutral_pulse_width_us) {
			ch1_motor_value = (m_mapper_2_channel_1.map((int16_t)(m_channel_1.pulse_width_us)) / (1<<6));
		} else {
			ch1_motor_value = 0;
		}
		if(m_channel_2.pulse_width_us < m_channel_2.neutral_pulse_width_us) {
			ch2_motor_value = (m_mapper_1_channel_2.map((int16_t)(m_channel_2.pulse_width_us)) / (1<<6));
		} else if(m_channel_2.pulse_width_us > m_channel_2.neutral_pulse_width_us) {
			ch2_motor_value = (m_mapper_2_channel_2.map((int16_t)(m_channel_2.pulse_width_us)) / (1<<6));
		} else {
			ch2_motor_value = 0;
		}
		
		// simple delta mixing
#ifdef RIGHT_MOTOR
		ch1_motor_value += ch2_motor_value;
#endif
#ifdef LEFT_MOTOR
		ch1_motor_value -= ch2_motor_value;
#endif
		
		// limit to +/- 255
		if(ch1_motor_value > 255) ch1_motor_value = 255;
		if(ch1_motor_value < -255) ch1_motor_value = -255;
		
		if(ch1_motor_value >= 0) {
			if((uint16_t)(ch1_motor_value) > m_channel_1.deadzone) {
				motor::set_speed((uint8_t)(ch1_motor_value));
				motor::set_direction(FORWARD);
			} else {
				motor::set_speed(0);
				motor::set_direction(BREAK);
			}
		} else {
			ch1_motor_value = abs(ch1_motor_value);
			if((uint16_t)(ch1_motor_value) > m_channel_1.deadzone) {
				motor::set_speed((uint8_t)(ch1_motor_value));
				motor::set_direction(BACKWARD);
			} else {
				motor::set_speed(0);
				motor::set_direction(BREAK);
			}
		}
	} else {
		// do the calibration
		calibrate();
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

/** 
 * @brief performs the calibration
 */
void calibrate() {
	// here we store the first 16 values of both channels in an array, drop the 2 highest and the 2 lowest values and get the average
	uint8_t const MAX_RECEIVED_PULSES = 16;
	static uint8_t received_pulses_cnt = 0;
	static uint16_t ch1[16] = {0};
	static uint16_t ch2[16] = {0};
	// insert received values into the channel arrays
	ch1[received_pulses_cnt] = m_channel_1.pulse_width_us;
	ch2[received_pulses_cnt] = m_channel_2.pulse_width_us;
	// update the cnt
	received_pulses_cnt++;
	if(received_pulses_cnt >= MAX_RECEIVED_PULSES) {
		// sort
		sort(ch1, MAX_RECEIVED_PULSES);
		sort(ch2, MAX_RECEIVED_PULSES);
		// drop the two lowest and highest values and average
		m_channel_1.neutral_pulse_width_us = median(ch1 + 2, MAX_RECEIVED_PULSES - 4);
		m_channel_2.neutral_pulse_width_us = median(ch2 + 2, MAX_RECEIVED_PULSES - 4);
		// recalibrate the linear mappers
		m_mapper_1_channel_1.init(CH1_PULSE_WIDTH_MIN_US, m_channel_1.neutral_pulse_width_us, (-1)*(1<<14), 0);
		m_mapper_2_channel_1.init(m_channel_1.neutral_pulse_width_us, CH1_PULSE_WIDTH_MAX_US, 0, (1<<14));
		m_mapper_1_channel_2.init(CH2_PULSE_WIDTH_MIN_US, m_channel_2.neutral_pulse_width_us, (-1)*(1<<14), 0);
		m_mapper_2_channel_2.init(m_channel_2.neutral_pulse_width_us, CH2_PULSE_WIDTH_MAX_US, 0, (1<<14));
		// update the calibration complete flag
		m_control_data.calibration_complete = true;
	}
}

/** 
 * @brief sorts the array ascending
 */
void sort(uint16_t *data, uint8_t const length) {
	bool elements_swapped = false;
	do {
		elements_swapped = false;
		uint8_t i=1; for(; i<length; i++) {
			if(data[i-1] > data[i]) {
				swap(&data[i-1], &data[i]);
				elements_swapped = true;
			}
		}		
	} while(elements_swapped);
}

/** 
 * @brief swaps to elements - required for sort
 */
void swap(uint16_t *elem1, uint16_t *elem2) {
	uint16_t const tmp = *elem1;
	*elem1 = *elem2;
	*elem2 = tmp;
}

/** 
 * @brief calculates the average of an array
 */
uint16_t median(uint16_t const *data, uint8_t const length) {
	return data[length/2];
}