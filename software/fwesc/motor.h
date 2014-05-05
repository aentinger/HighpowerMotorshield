/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module implements the control of the motor using the lxrobotics highpower motorshield
 * @file motor.h
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

typedef enum {FORWARD = 0, BACKWARD = 1, BREAK = 2} E_MOTOR_DIRECTION;

class motor {
public:
	/** 
	* @brief initializes this module
	 */
	static void init();

	/**
	* @brief set the direction of the motor
	*/
	static void set_direction(E_MOTOR_DIRECTION const dir);

	/**
	* @brief sets the speed of the motor - 255 is full speed, 0 is stop
	*/
	static void set_speed(uint8_t const speed);
	
	/**
	 * @brief disables the h bridge in case of e.g. over current
	 */
	static void disable();
	
	/**
	 * @brief enables the h bridge again
	 */
	static void enable();
	
private:
	/** 
	 * @brief Constructor
	 */
	motor() { }
};

#endif /* MOTOR_H_ */