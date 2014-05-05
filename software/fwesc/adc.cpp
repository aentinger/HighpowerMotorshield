/**
 * @author Alexander Entinger, MSc / LXRobotics GmbH
 * @brief module for accessing the adc
 * @file adc.cpp
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include "adc.h"
#include "motor.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/* MACRO SECTION */

// IS1 = A4 = ADC4 = PC4
#define MUX_ADC_TO_IS1() do { ADMUX &= 0xF0; ADMUX |= (1<<MUX2); } while(0)
// IS2 = A5 = ADC5 = PC5
#define MUX_ADC_TO_IS2() do { ADMUX &= 0xF0; ADMUX |= (1<<MUX2) | (1<<MUX0); } while(0)

/* TYPEDEF SECTION */

typedef enum {IS1 = 0, IS2 = 1} E_SELECTED_CURRENT_SENSOR;

/* PROTOTYPE SECTION */

void read_is_1();
void read_is_2();

/* GLOBAL VARIABLE SECTION */

static volatile E_SELECTED_CURRENT_SENSOR m_selected_current_sensor = IS1;

/* GLOBAL CONSTANTS */

// 46 A = 4,6 V
// 1 A = 0.1 V
// 1024 * 0.1 V / 5 V = 20.48 = 20
static uint16_t const CURRENT_SENSE_1_A = 20;
static uint16_t const CURRENT_SENSE_20_A = CURRENT_SENSE_1_A * 20;

/* FUNCTION SECTION */

/**
 * @brief initializes the adc module
 */
void adc::init() {
	// disable digital input buffers for adc5 and adc4
	DIDR0 = (1<<ADC5D) | (1<<ADC4D);
	// select AVcc as reference voltage
	ADMUX = (1<<REFS0);
	// enable adc
	ADCSRA = (1<<ADEN);
	// set prescaler to 128 -> fADC = 125 kHz
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	// perform a dummy readout
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC)) { asm(" NOP "); }
	(void)ADC;
	// clear interrupt flag
	ADCSRA |= (1<<ADIF);
	// enable adc complete interrupt
	ADCSRA |= (1<<ADIE);
	// mux to is 1
	read_is_1();
}

/** 
 * @brief adc interrupt service routine
 */
ISR(ADC_vect) {
	
	if(ADC > CURRENT_SENSE_20_A) {
		motor::disable();
	} else {
		motor::enable();
	}
		
	if(m_selected_current_sensor == IS1) read_is_2();
	else if(m_selected_current_sensor == IS2) read_is_1();
}

/** 
 * @brief sets up the adc for reading current sensor 1
 */
void read_is_1() {
	m_selected_current_sensor = IS1;
	// mux around
	MUX_ADC_TO_IS1();
	// start conversion
	ADCSRA |= (1<<ADSC);
}

/** 
 * @brief sets up the adc for reading current sensor 2
 */
void read_is_2() {
	m_selected_current_sensor = IS2;
	// mux around
	MUX_ADC_TO_IS2();
	// start conversion
	ADCSRA |= (1<<ADSC);
}