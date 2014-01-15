/**
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module implements the control of the LXRobotics Highpower Arduino Motorshield
 * @file LXR_highpower_motorshield.cpp
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include "LXR_highpower_motorshield.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

// IN1 = D5 = PD5 = OC0B
#define IN1_DDR		(DDRD)
#define IN1_PORT	(PORTD)
#define IN1		(1<<5)
// IN2 = D6 = PD6 = OC0A
#define IN2_DDR		(DDRD)
#define IN2_PORT	(PORTD)
#define IN2		(1<<6)
// INH = D7 = PD7
#define INH_DDR		(DDRD)
#define INH_PORT	(PORTD)
#define INH		(1<<7)
// IS1 = A4
#define IS1             (4)
// IS1 = A4
#define IS2             (5)

typedef struct {
  uint8_t spd;
  E_DIRECTION dir;
} 
s_motor_params;
static volatile s_motor_params m_motor_params = {
  0, FWD};

/**
 * @brief initializes the motorshield
 */
void LXR_highpower_motorshield::begin() {
  // set inh to output and to low (halfbridges deactivated)
  INH_PORT &= ~INH;
  INH_DDR |= INH;
  // set inX pins to outputs with value low
  IN1_PORT &= ~IN1;
  IN1_DDR |= IN1;
  IN2_PORT &= ~IN2;
  IN2_DDR |= IN2;
  // clear TCCR2A from whatever might be still left there
  TCCR2A = 0x00;	
  // reset the timer value
  TCNT2 = 0;
  // enable compare and overflow interrupts
  TIMSK2 = (1<<OCIE2A) | (1<<TOIE2);
  // activate timer with prescaler 32 => f_PWM = 1,96 kHz
  TCCR2B = (1<<CS21) | (1<<CS20);
  // set direction
  LXR_highpower_motorshield::set_direction(m_motor_params.dir);
  // set speed
  LXR_highpower_motorshield::set_speed(m_motor_params.spd);
  // activate h brigde
  INH_PORT |= INH;
}

/** 
 * @brief set the speed of the motor control
 * @param speed 0 => 0 speed, 255 => max speed
 */
void LXR_highpower_motorshield::set_speed(uint8_t const speed) {
  m_motor_params.spd = speed;
  OCR2A = m_motor_params.spd;
}

/** 
 * @brief returns the current motor speed
 */
uint8_t LXR_highpower_motorshield::get_speed() {
  return m_motor_params.spd;
}

/** 
 * @brief sets the direction of the motor
 * @param dir direction
 */
void LXR_highpower_motorshield::set_direction(E_DIRECTION const dir) {
  m_motor_params.dir = dir;
}

/** 
 * @brief returns the current direction
 */
E_DIRECTION LXR_highpower_motorshield::get_direction() {
  return m_motor_params.dir;
}

/**
 * @brief returns the current flow over the half brigde 1
 */
int LXR_highpower_motorshield::get_current_half_brigde_1() {
  return analogRead(IS1);
}

/**
 * @brief returns the current flow over the half brigde 2
 */
int LXR_highpower_motorshield::get_current_half_brigde_2() {
  return analogRead(IS2);
}

/**
 * @brief ISR for timer 2 overflow
 */
ISR(TIMER2_OVF_vect) {
  if(m_motor_params.spd > 0) {
    if(m_motor_params.dir == FWD) IN1_PORT |= IN1;
    else if(m_motor_params.dir == BWD) IN2_PORT |= IN2;
  }
}

/**
 * @brief ISR for compare a interrupt
 */
ISR(TIMER2_COMPA_vect) {
  IN1_PORT &= ~IN1;	
  IN2_PORT &= ~IN2;
}

