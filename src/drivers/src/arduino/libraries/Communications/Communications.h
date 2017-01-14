#ifndef COM_H
#define COM_H

#include "Arduino.h"

/*
  A struct that holds the required values to represent a ros geometry twist msg.
*/
struct Twist {
  
  float linear_x;
  float linear_y;
  float linear_z;
  
  float angular_x;
  float angular_y;
  float angular_z;
};

typedef struct Twist TwistMsg;

/*
  This class provides static methods for arduino for sending and reading floating point values
  through its serial port.
  
*/
class Communications {
  public:
    /*
      Reads a float from the serial port of the arduino and returns the value.
      
      Precondition: The sender must follow the protocol of sending the IDENTIFIER
        byte first before sending exactly four bytes that represents a single precision
        floating point value in byte format.

      Postcondition: Returns the floating point value read from the serial port.
    */
    static float readFloat();
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
    static int readTwistMsg(TwistMsg &twist_msg);
    /*
      Sends a floating point value through the serial port.
      
      Postcondition: The IDENTIFIER byte followed by four bytes that represents the single
        precision floating value value is sent through the serial port. The value sent
        will follow the little/big endian format of the arduino board.
    */
    static void sendFloat(const float value);
    /*
      Sends a TwistMsg through the serial port.

      Postcondition: The IDENTIFIER byte followed by the linear x, y, z then the angular x, y, z
        components are sent through serial in the listed order.
    */
    static void Communications::sendTwistMsg(TwistMsg const &twist_msg);

    // The identifier byte that is used in this serial protocol. Every
    // message sent or received through serial must include this identifier
    // byte in order for the receiver of the message to know when to begin
    // interpreting the message.
    static const char IDENTIFIER;
    // The size of the message that is to be sent or read after the identifer byte
    static const int MSG_SIZE;
  private:
    /*
      Converts a string that represents a float's bytes to a float.

      Precondition: str must contain four valid bytes that represents a float.
      Postcondition: returns a float that is interpreted in little/big endian format
        depending on the architecture of this CPU.
    */
    static float strRep_to_float(const char * str);
};

#endif
