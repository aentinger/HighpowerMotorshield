/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module implements the control of the LXRobotics Highpower Arduino Motorshield
 * @file LXR_highpower_motorshield.h
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#ifndef LXR_HIGHPOWER_MOTORSHIELD_H_
#define LXR_HIGHPOWER_MOTORSHIELD_H_

#include <stdint.h>

typedef enum {
  FWD = 0, BWD = 1} 
E_DIRECTION;

class LXR_highpower_motorshield {
public:
  /**
   * @brief initializes the motorshield
   */
  static void begin();

  /** 
   * @brief set the speed of the motor control
   * @param speed 0 => 0 speed, 255 => max speed
   */
   static void set_speed(uint8_t const speed);

  /** 
   * @brief returns the current motor speed
   */
   static uint8_t get_speed();

  /** 
   * @brief sets the direction of the motor
   * @param dir direction
   */
   static void set_direction(E_DIRECTION const dir);

  /** 
   * @brief returns the current direction
   */
   static E_DIRECTION get_direction();
   
   /**
    * @brief returns the current flow over the half brigde 1
    */
   static int get_current_half_brigde_1();
   
   /**
    * @brief returns the current flow over the half brigde 2
    */
   static int get_current_half_brigde_2();
};

#endif


