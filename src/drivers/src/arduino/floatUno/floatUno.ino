#include <Communications.h>
TwistMsg twist_msg;

void setup() {
  Serial.begin(9600);
}

void loop() {

  Communications::readTwistMsg(twist_msg);
  Communications::sendTwistMsg(twist_msg);
}

