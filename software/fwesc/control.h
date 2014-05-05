/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this moduls maps the receiver pulse width to an appopriate motor action
 * @file control.h
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

class control {
public:
	/** 
	 * @brief updates channel 1 - only to be called within isr context
	 */
	static void update_channel_1(uint16_t const ch1_pulse_width_us);
	
	/** 
	 * @brief updates channel 2 - only to be called within isr context
	 */
	static void update_channel_2(uint16_t const ch2_pulse_width_us);
	
	/** 
	 * @brief this function is called by the input module to indicate that channels have been lost - only to be called within isr context
	 */
	static void channel_s_lost();
	
	/** 
	 * @brief the control task itself - it is scheduled to be runnable after both channels have updated their data value
	 */
	static void run();
	
	/** 
	 * @brief returns true if the control task is runnable in the main thread
	 */
	static bool is_runnable();

private:
	/** 
	 * @brief Constructor
	 */
	control() { }	
	
};
	
#endif /* CONTROL_H_ */