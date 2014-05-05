/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module contains the sampling of the pwm channel inputs
 * @file input.h
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#ifndef INPUT_H_
#define INPUT_H_

class input {
public:
	/**
	 * @brief initializes the input module
	 */
	static void init();
	
private:
	/** 
	 * @brief Constructor
	 */
	input() { }
	
};

#endif /* INPUT_H_ */