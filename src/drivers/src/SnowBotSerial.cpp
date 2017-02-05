#include <SnowBotSerial.hpp>
using namespace LibSerial;

// The identifier byte that is used in this serial protocol. Every
// message sent or received through serial must include this identifier
// byte in order for the receiver of the message to know when to begin
// interpreting the message.
const char SnowBotSerial::IDENTIFIER = 'B';
// The VTIME as defined by POSIX termios
const short SnowBotSerial::TIME_OUT = 100;

/*
 * Constructs a SnowBotSerial object that is connected to a serial port with
 * baud rate specified by baud_rate.
 * 
 * Precondition: port must be a valid port name.
 */
SnowBotSerial::SnowBotSerial(const std::string port, const SerialStreamBuf::BaudRateEnum baud_rate) {
  this->port = port;
  serial_stream.Open(port);

  this->baud_rate = baud_rate;
  serial_stream.SetBaudRate(baud_rate);
}

/*
 * Reads data from the serial port using a read protocol that first finds and reads
 *  an IDENTIFIER byte then reading the following byte to determine the size of the message
 *  before finally reading and storing the actual data.
 *
 * Postcondition: returns a string read using the snowbot protocol, modifies the read_error
 *  flag to indicate whether the returned string is valid or invalid.
 */
std::string SnowBotSerial::readString() {
  /*
   * If VMIN = 0 and VTIME > 0 then this is a pure timed read. This is an overall
   * timer, not an intercharacter one. This means that if nothing is read in
   * (TIME_OUT * 1/10) of a sec then the control is returned back to the caller.
   */
  serial_stream.SetVMin(0);
  serial_stream.SetVTime(TIME_OUT);
  
  // assume first that the reading will not be successful and only changing
  // this status at the end if the reading was truely successful.
  read_error_flag = true;
  
  std::string message;
  unsigned long counter = 100000; // a time out counter.
  
  char ch = serial_stream.get();
  while (ch != IDENTIFIER) {
    if (counter-- == 0) {
      return message;
    }
    ch = serial_stream.get();
  }

  unsigned char size = serial_stream.get(); // this is the size of the message.
  
  serial_stream.SetVMin(size);
  message.reserve(size);
  
  for(std::size_t i = 0; i < size; i++){
    message.push_back(serial_stream.get());
  }

  if (message.length() == (size_t)size) {
    read_error_flag = false;
  }
  return message;
}

/*
 * Sends a message to serial using a write protocol that first sends the IDENTIFIER byte,
 * then the size number of bytes that this object will send before sending the payload.
 * 
 * Postcondition: the serial would have sent from left to right order the bytes:
 *  IDENTIFIER, size, msgToSend[0], msgToSend[1], ... , msgToSend[n].
 */
void SnowBotSerial::sendString(const std::string msgToSend) {
  serial_stream << IDENTIFIER << (unsigned char)(msgToSend.length());
  for (int i = 0; i < msgToSend.length(); i++) {
    serial_stream << msgToSend[i];
  }
}

/*
 * Sends the bytes that make up a single precision floating point number to serial. This
 *  function does not follow the sending protocol.
 *
 * postcondition: value will be sent using the little/big endian convention of this CPU's
 *  architecture.
 *
 */
void SnowBotSerial::writeFloat(const float value) {
  unsigned char * p = (unsigned char *) & value;
  for (int i = 0; i < sizeof(float); i++) {
    serial_stream << p[i];
  }
}

/*
 * Sends a twist msgs to serial using the snowbot protocol as used by sendString().
 *
 * Postcondition: The twist msg will be sent as six single precision floating point numbers,
 *  following the IDENTIFIER byte and the size byte. The floating point numbers will be sent
 *  as there exact representation according to the little/big endianness of this CPU.
 *
 */
void SnowBotSerial::sendTwistMsg(const geometry_msgs::Twist::ConstPtr& twist_msg) {
  int numberOfFloats = 6; // twist message has 6 floating point numbers.
  serial_stream << IDENTIFIER << (unsigned char)(sizeof(float) * numberOfFloats);
  
  writeFloat((float) twist_msg->linear.x);
  writeFloat((float) twist_msg->linear.y);
  writeFloat((float) twist_msg->linear.z);
  
  writeFloat((float) twist_msg->angular.x);
  writeFloat((float) twist_msg->angular.y);
  writeFloat((float) twist_msg->angular.z);
}

SerialStreamBuf::BaudRateEnum SnowBotSerial::getBaudRate() {
  return baud_rate;
}

void SnowBotSerial::switchPort(const std::string newPort) {
  serial_stream.Close();
  port = newPort;
  serial_stream.Open(port);
}

std::string SnowBotSerial::getPort() {
  return port;
}

/*
 * Gets the error status of the last read readString() call. The value is
 *  undefined if no previous readString() call had been made.
 *
 * Postcondition: Returns true if the last readString() was successful and the
 *  value returned by it is valid, otherwise returns false.
 *
 */
bool SnowBotSerial::read_error() {
  return read_error_flag;
}

