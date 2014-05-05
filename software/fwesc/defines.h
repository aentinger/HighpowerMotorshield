/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this file contains defines for the fw esc
 * @file defines.h
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#ifndef DEFINES_H_
#define DEFINES_H_

//#define LEFT_MOTOR
#define RIGHT_MOTOR

#if defined LEFT_MOTOR && defined RIGHT_MOTOR
	#error "Can only control one motor per esc"
#endif

#endif /* DEFINES_H_ */