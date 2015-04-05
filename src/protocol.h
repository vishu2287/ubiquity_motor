#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#define MOTOR_SYNC_BYTE0 0xff
#define MOTOR_SYNC_BYTE1 0xfe

enum motor_message_id
{
  MOTOR_MSG_REQUEST_SPEED = 0x01,
  MOTOR_MSG_REQUEST_ACCELERATION = 0x02,
  MOTOR_MSG_ODOMETER = 0x03,
};

struct motor_message
{
  unsigned char sync[2]; /* MOTOR_SYNC_BYTE0, MOTOR_SYNC_BYTE1 */
  unsigned char type;
  unsigned char addr;
  unsigned char data[4];
 // signed short data;
  unsigned char checksum;
};

#endif /* !PROTOCOL_H_ */
