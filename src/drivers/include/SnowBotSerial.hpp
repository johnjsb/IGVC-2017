#ifndef SNOWBOT_SERIAL
#define SNOWBOT_SERIAL

#include <SerialStream.h>
#include <geometry_msgs/Twist.h>
using namespace LibSerial;

/*
  This class provides methods to communicate with an Arduino Uno using a specific protocol.
*/
class SnowBotSerial {
  
  public:
    // The identifier byte that is used in this serial protocol. Every
    // message sent or received through serial must include this identifier
    // byte in order for the receiver of the message to know when to begin
    // interpreting the message.
    static const char IDENTIFIER;
    // The VTIME as defined by POSIX termios
    static const short TIME_OUT;
    
    /*
     * Constructs a SnowBotSerial object that is connected to a serial port with
     * baud rate specified by baud_rate.
     * 
     * Precondition: port must be a valid port name.
     */
    SnowBotSerial(const std::string port, const SerialStreamBuf::BaudRateEnum baud_rate);
    
    /*
     * Sends a message to serial using a write protocol that first sends the IDENTIFIER byte,
     * then the size number of bytes that this object will send before sending the payload.
     * 
     * Postcondition: the serial would have sent from left to right order the bytes:
     *  IDENTIFIER, size, msgToSend[0], msgToSend[1], ... , msgToSend[n].
     */
    void sendString(const std::string msgToSend);

    /*
     * Reads data from the serial port using a read protocol that first finds and reads
     *  an IDENTIFIER byte then reading the following byte to determine the size of the message
     *  before finally reading and storing the actual data.
     *
     * Postcondition: returns a string read using the snowbot protocol, modifies the read_error
     *  flag to indicate whether the returned string is valid or invalid.
     */
    std::string readString();
    
    /*
     * Sends a twist msgs to serial using the snowbot protocol as used by sendString().
     *
     * Postcondition: The twist msg will be sent as six single precision floating point numbers,
     *  following the IDENTIFIER byte and the size byte. The floating point numbers will be sent
     *  as there exact representation according to the little/big endianness of this CPU.
     *
     */
    void sendTwistMsg(const geometry_msgs::Twist::ConstPtr& twist_msg);
    
    /*
     * Gets the error status of the last read readString() call. The value is
     *  undefined if no previous readString() call had been made.
     *
     * Postcondition: Returns true if the last readString() was successful and the
     *  value returned by it is valid, otherwise returns false.
     *
     */
    bool read_error();
    
    // Changes the serial port that this object is connected to newPort.
    void switchPort(const std::string newPort);

    // observer functions for reading the private fields
    std::string getPort();
    SerialStreamBuf::BaudRateEnum getBaudRate();
    
  private:
    /*
     * Sends the bytes that make up a single precision floating point number to serial. This
     *  function does not follow the sending protocol.
     *
     * postcondition: value will be sent using the little/big endian convention of this CPU's
     *   architecture.
     *
     */
    void writeFloat(const float value);
  
    SerialStream serial_stream;
    std::string port;
    SerialStreamBuf::BaudRateEnum baud_rate;
    bool read_error_flag;
};

#endif
