#include <Communications.h>
TwistMsg twist_msg;
Communications caller;

void setup() {
  Serial.begin(9600);
}

void loop() {
  caller.readTwistMsg(twist_msg);
  if (!caller.read_error()) {
    Communications::sendTwistMsg(twist_msg);
  }
}

