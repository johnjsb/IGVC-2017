/*
 * Created By: Nick Wu
 * Modified By: Gareth Ellis
 * Created On: April 1st, 2016
 * Last Modified: October 29, 2016
 * Description: The firmware for the GPS arduino
 * Notes: - This file has requirements which are located in the "libraries"
 *        folder. Please make sure your workspace is set to "src/firmware"
 *        (These instructions are for the IGVC-2017 repository, but should
 *        carry over to future repositiories)
 *
 * Format of messages sent to computer (anything in brackets is a variable): GPS\0\0,(lat),(lon),(fix (0 or 1))
 */
//Data format: D(lat),(long),(fix),(x),(y),(z),(headingDegrees)
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

//CHANGE THIS CONSTANT DEPENDING ON YOUR CURRENT LOCATION
const float DECLINATION_ANGLE = 0.2793; //Find declination here: http://www.magnetic-declination.com/
#define GPSECHO  false

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

boolean usingInterrupt = false; //keeps track of using interrupt, off by default!
void useInterrupt(boolean);

void setup() {
  Serial.begin(115200);
  gps_setup();
  Serial.println("Setup finished, ready to go!");
}

void loop() {
  gps_check_new_data();
  char input = Serial.read();  
  send_gps_over_serial();
  Serial.read();
  Serial.flush();
}
void gps_setup() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  useInterrupt(true);
  delay(1000);
}

void gps_check_new_data() {
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
}

void send_gps_over_serial() {
  // Send the identifier (must be 5 characters)
  Serial.print("GPS\0\0");
  // Send the longitude, latitude, and gps_fix
  Serial.print(","); Serial.print(GPS.latitudeDegrees, 6);
  Serial.print(","); Serial.print(GPS.longitudeDegrees, 6);
  Serial.print(","); Serial.print((int)GPS.fix);
  Serial.println();
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
