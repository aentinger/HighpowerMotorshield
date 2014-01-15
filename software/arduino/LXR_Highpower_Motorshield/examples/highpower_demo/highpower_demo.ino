/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this sketch demonstrates the usage of the LXRobotics Highpower Arduino Motorshield Library
 * @file highpower_demo.ino
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include "LXR_highpower_motorshield.h"

void setup() {
  LXR_highpower_motorshield::begin();
  Serial.begin(115200);
}

void loop() {
  LXR_highpower_motorshield::set_direction(FWD);
  ramp_up();
  ramp_down();
  LXR_highpower_motorshield::set_direction(BWD);
  ramp_up();
  ramp_down();
}

/**
 * @brief increases the speed from 0 to 255
 */
void ramp_up() {
  for(int s = 0; s < 255; s+=5) {
     LXR_highpower_motorshield::set_speed(s);
     delay(100);
     char buf[64];
     int const len = sprintf(buf, "Speed  = %d, I1 = %d, I2 = %d\n", LXR_highpower_motorshield::get_speed(), LXR_highpower_motorshield::get_current_half_brigde_1(), LXR_highpower_motorshield::get_current_half_brigde_2());
     Serial.write((uint8_t*)(buf), len);
   }
}

/**
 * @brief reduces the speed from 255 to 0
 */
void ramp_down() {
  for(int s = 255; s >= 0; s-=5) {
     LXR_highpower_motorshield::set_speed(s);
     delay(100);
     char buf[64];
     int const len = sprintf(buf, "Speed  = %d, I1 = %d, I2 = %d\n", LXR_highpower_motorshield::get_speed(), LXR_highpower_motorshield::get_current_half_brigde_1(), LXR_highpower_motorshield::get_current_half_brigde_2());
     Serial.write((uint8_t*)(buf), len);
  }
}
