/**
 * @author Alexander Entinger, MSc / LXRobotics GmbH
 * @brief this sketch utilizes the LXRobotics Highpower Motorshield to form a motor driver which is controlled over the serial interface using a custom protocol
 * @file serial_motor_driver.ino
 * @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
 */

#include "LXR_highpower_motorshield.h"

/* PROTOCOL DESIGN */
/* PC -> ARDUINO
   1 Byte ID
   1 Byte DIRECTION
   1 Byte SPEED
   1 BYTE CHECKSUM = ID xor DIRETCTION xor SPEED
 */
/* ARDUINO -> PC
   1 Byte ID
   1 Byte STATUS
   1 Byte CHECKSUM = ID xor STATUS
 */

/* DEFINE SECTION */

#define SERIAL_MOTOR_DRIVER_ID      (128)
#define SERIAL_TIMEOUT_MS           (250)
#define MOTOR_DIR_BACKWARD          (0)
#define MOTOR_DIR_FORWARD           (1)
#define STATUS_ERROR                (0)
#define STATUS_OK                   (1)

/* CONSTANT SECTION */
static int const recv_msg_size = 4;
static int const reply_msg_size = 3;

/* CODE SECTION */

void setup() {
  LXR_highpower_motorshield::begin();
  LXR_highpower_motorshield::set_direction(FWD);
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
}

void loop() {
  uint8_t msg_buffer[recv_msg_size];
  uint8_t return_msg[reply_msg_size];

  int const bytes_received = Serial.readBytes((char *)(msg_buffer), recv_msg_size);
  boolean const msg_received = (bytes_received == recv_msg_size);

  boolean is_msg_good = false;
  return_msg[0] = SERIAL_MOTOR_DRIVER_ID;
  return_msg[1] = STATUS_ERROR;

  if(msg_received) {
    // check if the message is valud
    boolean is_id_correct = msg_buffer[0] == SERIAL_MOTOR_DRIVER_ID;
    boolean is_dir_plausible = (msg_buffer[1] == MOTOR_DIR_BACKWARD) || (msg_buffer[1] == MOTOR_DIR_FORWARD);
    boolean is_checksum_valid = (msg_buffer[0] ^ msg_buffer[1] ^  msg_buffer[2]) == msg_buffer[3];
    if(is_id_correct && is_dir_plausible && is_checksum_valid) {
      // in case of message being valid set direction and speed accordingly
      if(msg_buffer[1] == MOTOR_DIR_FORWARD) {
        LXR_highpower_motorshield::set_direction(FWD);
      } else {
        LXR_highpower_motorshield::set_direction(BWD);
      }
      LXR_highpower_motorshield::set_speed(msg_buffer[2]);      
      is_msg_good = true;
      return_msg[1] = STATUS_OK;
    }
    // write return message
    return_msg[2] = return_msg[0] ^ return_msg[1];
    Serial.write(return_msg, reply_msg_size);
  } 
  
  if(!msg_received || !is_msg_good) {
    // emergency stop - the connection might is apparently disabled
    LXR_highpower_motorshield::set_speed(0);
  }
}


