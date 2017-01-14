#include "Communications.h"

// The identifier byte that is used in this serial protocol. Every
// message sent or received through serial must include this identifier
// byte in order for the receiver of the message to know when to begin
// interpreting the message.
static const char Communications::IDENTIFIER = 'B';
// The size of the message that is to be sent or read after the identifer byte
static const int Communications::MSG_SIZE = 4;

/*
  Reads a float from the serial port of the arduino and returns the value.
      
  Precondition: The sender must follow the protocol of sending the IDENTIFIER
    byte first before sending exactly four bytes that represents a single precision
    floating point value in byte format.

  Postcondition: Returns the floating point value read from the serial port. Returns a
    NAN value if no proper protocol message was received before timing out.
*/
static float Communications::readFloat() {
  unsigned char message[MSG_SIZE];

  char receivedByte = Serial.read();
  while (receivedByte != IDENTIFIER) {
    if (Serial.available() > 0) {
      receivedByte = Serial.read();
    }
  }

  while (Serial.available() < MSG_SIZE);
  int i = 0;
  int counter = 0;
  const int TIME_OUT = 1000;
  while (i < MSG_SIZE && counter < TIME_OUT) {
    if (Serial.available() > 0) {
      message[i++] = Serial.read();
    }
    counter++;
  }
  // didn't read correctly, signal to wait for resend
  if (i != MSG_SIZE) {
    return NAN;
  }
  return strRep_to_float(message);
}

/*
  Reads a geometry_msg::Twist message sent through serial.
      
  Precondition: The sender must follow the protocol of first sending the IDENTIFIER byte
    followed by the linear x, y, z components of the twist message in that order then
    send the angular x, y, z components of the twist message in that order.
  
  Modifies: twist_msg by writing the floats read from serial to it.

  Postcondition: Reads the required twist message values into twist_msg. Returns -1 if the
    reading was not successful in this case the resulting twist_msg will contain a NAN value,
    and possibly other garbage values. Returns 0 if the reading was successful.
*/
static int Communications::readTwistMsg(TwistMsg &twist_msg) {
  twist_msg.linear_x = readFloat();
  if (twist_msg.linear_x == NAN) {
    return -1;
  }
  twist_msg.linear_y = readFloat();
  if (twist_msg.linear_y == NAN) {
    return -1;
  }
  twist_msg.linear_z = readFloat();
  if (twist_msg.linear_z == NAN) {
    return -1;
  }

  twist_msg.angular_x = readFloat();
  if (twist_msg.angular_x == NAN) {
    return -1;
  }
  twist_msg.angular_y = readFloat();
  if (twist_msg.angular_y == NAN) {
    return -1;
  }
  twist_msg.angular_z = readFloat();
  if (twist_msg.angular_z == NAN) {
    return -1;
  }
}

/*
  Sends a floating point value through the serial port.
      
  Postcondition: The IDENTIFIER byte followed by four bytes that represents the single
    precision floating value value is sent through the serial port. The value sent
    will follow the little/big endian format of the arduino board.
*/
static void Communications::sendFloat(const float value) {
  unsigned char * p = (unsigned char *) & value;
  Serial.write(IDENTIFIER);
  for (int i = 0; i < sizeof(float); i++) {
    Serial.write(p[i]);
  }
}

/*
  Sends a TwistMsg through the serial port.

  Postcondition: The IDENTIFIER byte followed by the linear x, y, z then the angular x, y, z
    components are sent through serial in the listed order.
*/
static void Communications::sendTwistMsg(TwistMsg const &twist_msg) {
  sendFloat(twist_msg.linear_x);
  sendFloat(twist_msg.linear_y);
  sendFloat(twist_msg.linear_z);

  sendFloat(twist_msg.angular_x);
  sendFloat(twist_msg.angular_y);
  sendFloat(twist_msg.angular_z);
}

/*
  Converts a string that represents a float's bytes to a float.

  Precondition: str must contain four valid bytes that represents a float.
  
  Postcondition: returns a float that is interpreted in little/big endian format
    depending on the architecture of this CPU.
*/
static float Communications::strRep_to_float(const char * str) {
  float ff = 0;
  memcpy(&ff, str, sizeof(float));
  return ff;
}
