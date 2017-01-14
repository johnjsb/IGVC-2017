#include <Communications.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Check serial for availability first.
  TwistMsg twist_msg;
  if (Serial.available() > 0) {
    int error = Communications::readTwistMsg(twist_msg);
    if (error != -1) {
      Communications::sendTwistMsg(twist_msg);
    }
  }
}

