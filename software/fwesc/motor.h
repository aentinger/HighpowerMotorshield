/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module implements the control of the motor using the lxrobotics highpower motorshield
 * @file motor.h
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
	
private:
	/** 
	 * @brief Constructor
	 */
	motor() { }
};

#endif /* MOTOR_H_ */