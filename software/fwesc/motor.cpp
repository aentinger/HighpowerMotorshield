/** 
 * @author Alexander Entinger, MSc / LXRobotics
 * @brief this module implements the control of the motor using the lxrobotics highpower motorshield
 * @file motor.cpp
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include "motor.h"
#include <avr/io.h>
#include <avr/interrupt.h>

/* DEFINE SECTION */
// IN1 = D5 = PD5 = OC0B
#define IN1_DDR		(DDRD)
#define IN1_PORT	(PORTD)
#define IN1			(1<<5)
// IN2 = D6 = PD6 = OC0A
#define IN2_DDR		(DDRD)
#define IN2_PORT	(PORTD)
#define IN2			(1<<6)
// INH = D7 = PD7
#define INH_DDR		(DDRD)
#define INH_PORT	(PORTD)
#define INH			(1<<7)

/* MACRO SECTION */
#define DRIVE_FORWARD() do { TCCR0A &= 0x0F; TCCR0A |= (1<<COM0A1); IN1_PORT &= ~IN1; } while(0)
#define DRIVE_BACKWARD() do { TCCR0A &= 0x0F; TCCR0A |= (1<<COM0B1); IN2_PORT &= ~IN2; } while(0)
#define DRIVE_BREAK() do { TCCR0A &= 0x0F; IN1_PORT &= ~IN1; IN2_PORT &= ~IN2; } while (0)

/* GLOBAL VARIABLE SECTION */
typedef struct {
	uint8_t speed;
	E_MOTOR_DIRECTION dir;
} s_motor_state;
static s_motor_state m_motor_state = {0, BREAK};

/* FUNCTION SECTION */

/** 
 * @brief initializes this module
 */
void motor::init() {
	// set all motor pins to outputs
	IN1_DDR |= IN1;
	IN2_DDR |= IN2;
	INH_DDR |= INH;
	// set speed and direction
	motor::set_direction(m_motor_state.dir);
	motor::set_speed(m_motor_state.speed);
	// enable the motor
	INH_PORT |= INH;
	// enable phase correct timer mode
	TCCR0A |= (1<<WGM00);
	// enable timer with prescaler 64
	TCCR0B |= (1<<CS01) | (1<<CS00);
}

/**
 * @brief set the direction of the motor
 */
void motor::set_direction(E_MOTOR_DIRECTION const dir) {
	m_motor_state.dir = dir;
	if(m_motor_state.dir == FORWARD) DRIVE_FORWARD();
	else if(m_motor_state.dir == BACKWARD) DRIVE_BACKWARD();
	else if(m_motor_state.dir == BREAK) DRIVE_BREAK();
}

/**
 * @brief sets the speed of the motor - 255 is full speed, 0 is break
 */
void motor::set_speed(uint8_t const speed) {
	m_motor_state.speed = speed;
	OCR0A = m_motor_state.speed;
	OCR0B = m_motor_state.speed;
}

/**
 * @brief disables the h bridge in case of e.g. over current
 */
void motor::disable() {
	INH_PORT &= ~INH;
}

/**
 * @brief enables the h bridge again
 */
void motor::enable() {
	INH_PORT |= INH;
}
