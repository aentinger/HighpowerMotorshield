/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module contains the sampling of the pwm channel inputs
 * @file input.h
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