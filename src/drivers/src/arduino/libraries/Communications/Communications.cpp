#include "Communications.h"

// The identifier byte that is used in this serial protocol. Every
// message sent or received through serial must include this identifier
// byte in order for the receiver of the message to know when to begin
// interpreting the message.
const char Communications::IDENTIFIER = 'B';
// the maximum time out limit.
const unsigned long Communications::MAX_TIME_OUT = 1000000;

/*
 * Reads a sequence of bytes through serial and stores it into a c string.
 *  The sender must follow the protocol that first sends the IDENTIFIER byte then
 *  the message size before sending the payload.
 *
 * Poscondition: Returns a newly created c string that holds the payload bytes read.
 *  Returns NULL if the read was unsuccessful and sets the read_error flag to true. If
 *  the reading was successful then the read_error flag is set to false. Also sets the
 *  message size field to the size of the message, the message size field can be obtained
 *  with the function getMsgSize().
 */
char * Communications::readCString() {
  read_error_flag = true; // assume that there will be a reading error initially.
  unsigned long timer = MAX_TIME_OUT;
  char ch;
  do {
    if (Serial.available()) {
      ch = Serial.read();
    }
    if (timer-- == 0) {
      return NULL;
    }
  } while (ch != IDENTIFIER);
  
  // wait to read the next byte to get the size of the message.
  timer = MAX_TIME_OUT;
  while (Serial.available() <= 0) {
    if (timer-- == 0) {
      return NULL;
    }
  }
  
  unsigned char size = Serial.read();
  
  char * message = new char [size + 1]; // add one to account for null terminator.
  // must make sure that there are enough bytes in the port before reading.
  timer = MAX_TIME_OUT;
  while (Serial.available() < size) {
    if (timer-- == 0) {
      return NULL;
    }
  }
  int i;
  for (i = 0; i < size; i++) {
    message[i] = Serial.read();
  }
  message[i] = '\0';
  
  msgSize = size; // sets the size of the message that was read.
  read_error_flag = false; // no reading error.
  return message;
}

/*
 * Reads a geometry_msg::Twist message sent through serial.
 *     
 * Precondition: The sender must follow the protocol of first sending the IDENTIFIER byte
 *  followed by the size of the message then the linear x, y, z float components of the twist 
 *  message in that order then send the angular x, y, z float components of the twist message 
 *  in that order. The floats sent must follow the same little/big endian conventions of this
 *  CPU.
 * 
 * Modifies: twist_msg by writing the floats read from serial to it.
 *
 * Postcondition: Reads the required twist message values into twist_msg. Sets the read_error
 *  flag to true if there was an error in the reading, false otherwise.
 */
void Communications::readTwistMsg(TwistMsg &twist_msg) {
  char * message = readCString();
  if (message == NULL || msgSize != sizeof(float) * NUM_OF_FLOATS) {
    read_error_flag = true;
    return;
  }
  
  const char * msgPtr = message;
  for (int i = 0; i < NUM_OF_FLOATS; i++) {
    twist_msg.twistValues[i] = strRep_to_float(msgPtr);
    msgPtr += sizeof(float);
  }
  
  delete [] message;
  read_error_flag = false;
}

/*
 * Sends the exact representation of a single precision floating point value that this
 *  CPU uses to the serial port.
 *     
 * Postcondition: The value sent will follow the little/big endian format of this CPU.
 */
void Communications::writeFloat(const float value) {
  unsigned char * p = (unsigned char *) & value;
  for (int i = 0; i < sizeof(float); i++) {
    Serial.write(p[i]);
  }
}

/*
 * Sends a TwistMsg through the serial port, using the Communications protocol.
 *
 * Postcondition: The IDENTIFIER byte followed by the size of the message then the
 *  linear x, y, z then the angular x, y, z components are sent through serial in
 *  the listed order.
 */
void Communications::sendTwistMsg(TwistMsg const &twist_msg) {
  Serial.write(IDENTIFIER);
  Serial.write((unsigned char)(NUM_OF_FLOATS * sizeof(float))); // Six floats in a TwistMsg;
  
  for (int i = 0; i < NUM_OF_FLOATS; i++) {
    writeFloat(twist_msg.twistValues[i]);
  }
}

/*
 * Converts a string that represents a float's bytes to a float.
 *
 * Precondition: str must contain four valid bytes that represents a float.
 * 
 * Postcondition: returns a float that is interpreted in little/big endian format
 *   depending on the architecture of this CPU.
 */
float Communications::strRep_to_float(const char * str) {
  float ff = 0;
  memcpy(&ff, str, sizeof(float));
  return ff;
}

/*
 * Obtains the error flag value of the last read operation.
 *
 * Postcondition: Returns true if the last read was not successful, false otherwise.
 *  The value is undefined if no read was made before calling this function.
 */
bool Communications::read_error() {
  return read_error_flag;
}

/*
 * Obtains the message size of the last read operation.
 *
 * Postcondition: Returns the message size of the last read. The value is undefined if
 *  no read was made before calling this function.
 */
size_t Communications::getMsgSize() {
  return msgSize;
}
