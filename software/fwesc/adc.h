/**
 * @author Alexander Entinger, MSc / LXRobotics GmbH
 * @brief module for accessing the adc
 * @file adc.h
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#ifndef ADC_H_
#define ADC_H_

class adc {
public:
	/**
	 * @brief initializes the adc module
	 */
	static void init();
private:
	/** 
	 * @brief Constructor
	 */
	adc() { }
};

#endif /* ADC_H_ */