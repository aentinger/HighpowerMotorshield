/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief main file for lxrobotics highpower motorshield featherweight electronic speed controller
 * @file main.cpp
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

/* INCLUDE SECTION */
#include "motor.h"
#include "input.h"
#include "control.h"
#include "adc.h"
#include <avr/interrupt.h>

/* PROTOTYPE SECTION */
void init_application();

/* FUNCTION SECTION */

int main(void) {
    
	init_application();
	
	for(;;) {
		if(control::is_runnable()) {
			control::run();
		}
	}
}

/** 
 * @brief initializes the application
 */
void init_application() {
	
	// initialize the motor module
	motor::init();
	
	// initialize the input module
	input::init();
	
	// initialize the adc module
	adc::init();
	
	// globally enable interrupts
	sei();
}