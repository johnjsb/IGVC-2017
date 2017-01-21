#include <SnowBotSerial.hpp>
using namespace LibSerial;

// The size of the message that is to be sent or read after the identifer byte.
const int SnowBotSerial::MSG_SIZE = 4;
// The identifier byte that is used in this serial protocol. Every
// message sent or received through serial must include this identifier
// byte in order for the receiver of the message to know when to begin
// interpreting the message.
const char SnowBotSerial::IDENTIFIER = 'B';
// The VTIME as defined by POSIX termios
const short SnowBotSerial::TIME_OUT = 100;

/*
  Constructs a SnowBotSerial object that has a serial port that is connected to port with
  baud rate specified by baud_rate.
  
  Precondition: port must be a valid port name.
*/
SnowBotSerial::SnowBotSerial(const std::string port, const SerialStreamBuf::BaudRateEnum baud_rate) {
  this->port = port;
  serial_stream.Open(port);

  this->baud_rate = baud_rate;
  serial_stream.SetBaudRate(baud_rate);
}

/*
  Reads data from the serial port using a read protocol that first finds and reads
  an IDENTIFIER byte before reading the next MSG_SIZE number of bytes.

  Throws: const char * exception with message "overtime exception" if function is unable
    to detect proper protocol message before timing out.
  
  Postcondition: returns a message with size MSG_SIZE.
*/
std::string SnowBotSerial::readProtocol() {
  /*
    If VMIN = 0 and VTIME > 0 then this is a pure timed read. This is an overall
    timer, not an intercharacter one. This means that if nothing is read in
    (TIME_OUT * 1/10) of a sec then the control is returned back to the caller.
  */
  serial_stream.SetVMin(0);
  serial_stream.SetVTime(TIME_OUT);
  
  char ch = serial_stream.get();

  while (ch != IDENTIFIER) {
    ch = serial_stream.get();
  }

  // sets the minimum number of characters that must be read from the stream.
  serial_stream.SetVMin(MSG_SIZE);

  std::string message;
  message.reserve(MSG_SIZE);
  for(std::size_t i = 0; i < MSG_SIZE; i++){
    message.push_back(serial_stream.get());
  }

  return message;
}

/*
  Sends a message to serial using a write protocol that first sends the IDENTIFIER byte
  before sending MSG_SIZE number of bytes.
  
  Postcondition: the serial would have sent from left to right order the bytes:
    IDENTIFIER, msgToSend[0], msgToSend[1], ... , msgToSend[n].
*/
void SnowBotSerial::writeProtocol(const std::string msgToSend) {
  serial_stream << IDENTIFIER;
  for (int i = 0; i < msgToSend.length(); i++) {
    serial_stream << msgToSend[i];
  }
}

/*
  Sends a single precision floating point number through serial using the protocol used
  in write protocol.

  postcondition: value will be sent using the little/big endian convention of this CPU's
    architecture.

*/
void SnowBotSerial::writeFloat(const float value) {
  int len = sizeof(float);
  unsigned char * p = (unsigned char *) & value;
  serial_stream << IDENTIFIER;
  for (int i = 0; i < len; i++) {
    serial_stream << p[i];
  }
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

