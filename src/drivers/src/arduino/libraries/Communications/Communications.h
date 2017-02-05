#ifndef COM_H
#define COM_H

#include "Arduino.h"
#define NUM_OF_FLOATS 6

/*
 * Specifications:
 *  - tested to work with the arudino uno communicating with ubuntu
 */


/*
  A struct that holds the required values to represent a ros geometry twist msg.
*/
struct Twist {
  
  float twistValues[NUM_OF_FLOATS];
};

typedef struct Twist TwistMsg;

/*
  This class provides static methods for arduino for sending and reading floating point values
  through its serial port.
  
*/
class Communications {
  public:
    // The identifier byte that is used in this serial protocol. Every
    // message sent or received through serial must include this identifier
    // byte in order for the receiver of the message to know when to begin
    // interpreting the message.
    static const char IDENTIFIER;
    
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
    char * readCString();
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
    void readTwistMsg(TwistMsg &twist_msg);
    /*
     * Sends a TwistMsg through the serial port, using the Communications protocol.
     *
     * Postcondition: The IDENTIFIER byte followed by the size of the message then the
     *  linear x, y, z then the angular x, y, z components are sent through serial in
     *  the listed order.
     */
    static void sendTwistMsg(TwistMsg const &twist_msg);
    /*
     * Obtains the error flag value of the last read operation.
     *
     * Postcondition: Returns true if the last read was not successful, false otherwise.
     *  The value is undefined if no read was made before calling this function.
     */
    bool read_error();
    /*
     * Obtains the message size of the last read operation.
     *
     * Postcondition: Returns the message size of the last read. The value is undefined if
     *  no read was made before calling this function.
     */
    size_t getMsgSize();
  private:
    /*
     * Converts a string that represents a float's bytes to a float.
     *
     * Precondition: str must contain four valid bytes that represents a float.
     * 
     * Postcondition: returns a float that is interpreted in little/big endian format
     *   depending on the architecture of this CPU.
     */
    static float strRep_to_float(const char * str);
    /*
     * Sends the exact representation of a single precision floating point value that this
     *  CPU uses to the serial port.
     *     
     * Postcondition: The value sent will follow the little/big endian format of this CPU.
     */
    static void writeFloat(const float value);

    bool read_error_flag;
    size_t msgSize;
    static const unsigned long MAX_TIME_OUT;
};

#endif
