// #include <Arduino.h>
// #include <elapsedMillis.h>
// #include <TinyGPS++.h>
// #include <time.h>
// #include <SoftwareSerial.h>
// #include <SPI.h>
// #include <SPIMemory.h>
// #include <time.h>
// #include <TimeLib.h>
// #include <EEPROM.h>
// #include <avr/sleep.h>
// #include <definitions.h>
// #include <Wire.h>
// #include <codes.h>
// #include <ArduinoJson.h>

// void setup(){
//     Serial.begin(9600);
//     Serial1.begin(9600);
//     SPI.begin();
//     delay(1000);
//     Serial.println(F("SYSTEM INIT.."));
//     pinMode(GPS_PIN, OUTPUT);
//     digitalWrite(GPS_PIN, HIGH);
// }

// void loop(){
//     while(Serial.available() > 0){
//         Serial1.write(Serial.read());
//     }
//     while(Serial1.available() > 0){
//         Serial.write(Serial1.read());
//     }
// }