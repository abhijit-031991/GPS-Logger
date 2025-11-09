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

// ///// LIBRARY DECLARATIONS /////
// TinyGPSPlus gps;
// SPIFlash flash(LCS);
// elapsedMillis mTime;
// elapsedMillis bTime;

// // ///// DEVICE DEFINITIONS /////

// // GPS Storage Variables //
// float lat;                        // Storing last known Latitude
// float lng;                        // Storign last known Longitude

// // Flash Adressess //
// uint32_t wAdd;                // Write Address Parameter
// uint32_t rAdd;                // Read Address Parameter

// // GPS Control Variables //
// int gpsFrequency = 15;            // GPS Frequency in minutes *** USER CONFIG ***
// int gpsTimeout = 60;              // GPS Timesout after 'x' seconds *** USER CONFIG ***
// int gpsHdop = 5;                  // GPS HODP Parameter *** USER CONFIG ***

// // Device Variables //
// uint16_t cnt;                     // No. of data points available for download

// // Other Variables //
// bool wipe = false;
// bool gui = false;
// bool sleeping = false;
// unsigned long rtccounter = 0;


// //...................................//
// //           FUNCTIONS               //
// //...................................//
// void updateEeprom(int gf, int gt, int gh, uint16_t c, uint32_t w, uint32_t r){
//   meta metaData;
//   metaData.count = c;
//   metaData.gfrq = gf;
//   metaData.gto = gt;
//   metaData.hdop = gh;
//   metaData.ra = r;
//   metaData.wa = w;
//   EEPROM.put(eepromAddress, metaData);
// }

// void sendSerialMessage(byte code){
//     StaticJsonDocument<256> doc;
//     String data;
//     doc["Msg"] = code;
//     doc["ID"] = tag;
//     serializeJson(doc, data);
//     Serial.println(data);
// }

// void eepromSettings(){  
//   meta metaData;
//   EEPROM.get(eepromAddress, metaData);
//   if (metaData.gfrq == -1 || metaData.gto == -1 || metaData.gto == -1)
//   {
//     Serial.println(F("No Settings on EEPROM"));
//     Serial.println(F("Writing Default Settings EEPROM"));
//     metaData.count = cnt;
//     metaData.gfrq = gpsFrequency;
//     metaData.gto = gpsTimeout;
//     metaData.hdop = gpsHdop;
//     metaData.ra = 0;
//     metaData.wa = 0;
//     EEPROM.put(eepromAddress, metaData);

//   }else{

//     gpsFrequency = metaData.gfrq;
//     gpsTimeout = metaData.gto;
//     gpsHdop = metaData.hdop;
//     rAdd = metaData.ra;
//     wAdd = metaData.wa;
//     cnt = metaData.count;
//     Serial.println(F("#### CURRENT SETTINGS ####"));
//     Serial.println();
//     Serial.print(F("GPS FREQUENCY : ")); Serial.print(gpsFrequency); Serial.println(F(" Min/s"));
//     Serial.print(F("GPS TIMEOUT : ")); Serial.print(gpsTimeout); Serial.println(F(" seconds"));
//     Serial.print(F("GPS HDOP : ")); Serial.println(gpsHdop);
//     Serial.print(F("DATA POINTS : ")); Serial.println(cnt);
//     Serial.println();Serial.println();
//   }
   
// }

// void recGPS(bool setup){
//   mTime = 0;
//   double hdopstrt = 10.00;
//   uint32_t tempTime = 0;
//   if (setup)
//   {
//     tempTime = 1800000;
//   }else{
//     tempTime = gpsTimeout * 60000;
//   }
  
//   digitalWrite(GPS_PIN, HIGH);
//   while ((mTime/1000) <= tempTime)
//   {
//     // Serial.println(currentTime-xa);
//     while (Serial1.available() > 0)
//     {
//       if (!gps.encode(Serial1.read()))
//       {
//         if (!gps.location.isValid())
//         {
//           Serial.println(F("Acquiring"));
//         }      
//       }else{
//         // Serial.println(gps.location.isUpdated());
//         // Serial.print(F("Location Age:"));
//         // Serial.println(gps.location.age());
//         // Serial.print(F("Time Age:"));
//         // Serial.println(gps.time.age());
//         Serial.print(F("HDOP Age:"));
//         Serial.println(gps.hdop.age());
//         // Serial.print(F("Satellites:"));
//         // Serial.println(gps.satellites.value());
//         Serial.print(F("HDOP:"));
//         Serial.println(gps.hdop.value());
//         if (gps.hdop.hdop() != 0.00)
//         {
//           hdopstrt = gps.hdop.hdop();
//           Serial.println(hdopstrt);
//         }
//       }    
//     }
//     if (hdopstrt < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000 && gps.hdop.age() < 50)
//     {
//       break;
//     }  
//   }   

//   digitalWrite(GPS_PIN, LOW);
//   data dat;

//   if (gps.location.age() < 60000)
//   {
//     //pack data into struct
//     lat = gps.location.lat();
//     lng = gps.location.lng();
//     dat.lat = gps.location.lat();
//     dat.lng = gps.location.lng();
//   }else{
//     // pack data into struct with lat long = 0
//     dat.lat = 0;
//     dat.lng = 0;
//   }
//     setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
//     dat.datetime = (uint32_t)now();
//     dat.locktime = mTime/1000;
//     dat.hdop = gps.hdop.hdop();
//     dat.count = cnt;
//     dat.id = tag;
//     dat.x = 0;
//     dat.y = 0;
//     dat.z = 0;
    
//     Serial.println(dat.datetime);
//     Serial.println(dat.lat);
//     Serial.println(dat.lng);
//     Serial.println(dat.locktime);
//     Serial.println(dat.hdop);
//     Serial.println(dat.count);

//   if (!setup)
//   {
//     if (flash.powerUp())
//     {
//       Serial.println(F("Powered Up"));
//       delay(500);
//       Serial.println((int)sizeof(dat));
//       wAdd = flash.getAddress(sizeof(dat));
//       Serial.println(wAdd);
//       if (flash.writeAnything(wAdd, dat))
//       {
//         Serial.println(F("Write Successful"));
//         cnt = cnt + 1;
//       }else
//       {
//         Serial.println(F("Write Failed"));
//         Serial.println(flash.error(VERBOSE));
//       }     
//     }else
//     {
//       Serial.println(F("Power Up Failed"));
//     }   
//     flash.powerDown();

//     updateEeprom(gpsFrequency, gpsTimeout, gpsHdop, cnt, wAdd, rAdd);
//   }else{
//     sendSerialMessage(GPS_SUCCESS);
//   }

// }

// void read_send(){ 
//   data dat;
//   StaticJsonDocument<256> doc;
//   String databuf;

//   if (flash.powerUp())
//   {
//     if (flash.readAnything(rAdd, dat))
//     {
//       // Serial.print(dat.datetime);
//       // Serial.print(F(","));
//       // Serial.print(dat.hdop);
//       // Serial.print(F(","));
//       // Serial.print(dat.lat, 6);
//       // Serial.print(F(","));
//       // Serial.print(dat.lng, 6);
//       // Serial.print(F(","));
//       // Serial.print(dat.hdop);
//       // Serial.print(F(","));
//       // Serial.println(dat.locktime);     
//       doc["ID"] = dat.id;
//       doc["DT"] = dat.datetime;
//       doc["LAT"] = dat.lat;
//       doc["LNG"] = dat.lng;
//       doc["HDOP"] = dat.hdop;
//       doc["LCKTM"] = dat.locktime;
//       doc["CNT"] = dat.count;
//       doc["X"] = dat.x;
//       doc["Y"] = dat.y;
//       doc["Z"] = dat.z;
//       serializeJson(doc, databuf);
//       Serial.println(databuf);
//     }else
//     {
//       Serial.println(F("Read Failed"));
//     }    
//   }else{
//     Serial.println(F("Flash Power Up Failed"));
//   }
//   delay(10);
      
// }

// void deviceCalibration(){
//     bool fixStatus = false;
//     Serial.println("Starting device calibration...");
//     if (flash.powerUp())
//     {
//       if (flash.getJEDECID())
//       {
//         Serial.println("Flash memory detected successfully.");
//         sendSerialMessage(FLASH_SUCCESS); // Indicate flash memory success
//       } else {
//         Serial.println("Failed to detect flash memory.");
//         sendSerialMessage(FLASH_ERROR); // Indicate flash memory error
//       }
//     }  

//     // recGPS(true);
//     sendSerialMessage(GPS_SUCCESS); // Indicate GPS success for calibration purposes

//     Serial.println("Device calibration completed.");
// }

// void waitForSerialCommand(uint32_t timeout){
//   Serial.println(F("Waiting for serial commands..."));
//     mTime = 0;
//     uint16_t bytesAvailable = 0;
//     while (mTime < timeout)
//     {
//       bytesAvailable = Serial.available();
//       if(bytesAvailable == sizeof(reqPing)){
//         reqPing incomingPing;
//         Serial.readBytes((char*)&incomingPing, sizeof(incomingPing));
//         if (incomingPing.tag == tag)
//         {
//           if (incomingPing.request == FINISH_CALIBRATION)
//           {
//             break;
//           }
//         }
//         if (incomingPing.request == DATA_DOWNLOAD_ALL)
//         {
//           uint32_t tempRAdd = 0;
//           while (tempRAdd > wAdd)
//           {
//             read_send();
//             tempRAdd = tempRAdd + sizeof(data);
//           }         
//         }

//         if (incomingPing.request == DATA_DOWNLOAD_NEW)
//         {
//           while (rAdd < wAdd)
//           {
//             read_send();
//             rAdd = rAdd + sizeof(data);
//           }
//         }      
        
//       }
//       if (bytesAvailable == sizeof(setttings))
//       {
//         setttings incomingSettings;
//         Serial.readBytes((char*)&incomingSettings, sizeof(incomingSettings));
//         if (incomingSettings.tag == tag)
//         {
//           gpsFrequency = incomingSettings.gpsFrq;
//           gpsTimeout = incomingSettings.gpsTout;
//           gpsHdop = incomingSettings.hdop;
//           updateEeprom(gpsFrequency, gpsTimeout, gpsHdop, cnt, wAdd, rAdd);
//           sendSerialMessage(SETTINGS_UPDATED);
//         }else{
//           sendSerialMessage(SETTINGS_UPDATE_ERROR);
//         } 
//       }
//     }              
//   }

// void RTC_init(void)
// {
//   /* Initialize RTC: */
//   while (RTC.STATUS > 0)
//   {
//     ;                                   /* Wait for all register to be synchronized */
//   }
//   RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

//   RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

//   RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 16384, resulting in 32.768kHz/16384 = 2Hz */
//   | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
// }

// ISR(RTC_PIT_vect)
// {
//   RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
//   rtccounter = rtccounter + 1;
// }

// void setup(){

//     Serial.begin(9600);
//     Serial1.begin(115200);
//     SPI.begin();
//     delay(1000);
//     Serial.println(F("SYSTEM INIT.."));

//     if(!flash.powerUp()){
//         Serial.println(flash.error(VERBOSE));
//     }
//     if (!flash.powerDown())
//     {
//       Serial.println(flash.error(VERBOSE));
//     }
//     if(!flash.powerUp()){
//         Serial.println(flash.error(VERBOSE));
//     }
    
//     if(!flash.begin(MB(64))){
//         Serial.println(F("Flash did not begin"));
//         Serial.println(flash.error(VERBOSE));
//     } 
//     Serial.println(flash.getManID());
//     if(!flash.powerUp()){
//         Serial.println(F("PWR UP Failed!"));
//     }
//     if (wipe == true)
//     {
//         Serial.println(F("WIPING FLASH"));
//         if(flash.eraseChip()){
//         Serial.println(F("Memory Wiped"));  
//         }else
//         {
//         Serial.println(flash.error(VERBOSE));
//         }
//     }   
//     if(flash.powerDown()){
//         digitalWrite(FCS, HIGH);
//     }else{
//         Serial.println(flash.error(VERBOSE));
//     }

//     deviceCalibration();
//     waitForSerialCommand(180000); // Wait for 3 minutes for serial commands
//     eepromSettings();
    
//     RTC_init();
//     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//     sleep_enable();
// }

// void loop(){

//   if (sleeping == true)
//   {
//     if (rtccounter < gpsFrequency*60)
//     {
//       sleeping = true;
//     }else{
//       sleeping = false;
//     }
//   }else{
//     recGPS(false);
//     sleeping = true;
//     rtccounter = 0;
//   }
//   Serial.flush();
//   sleep_cpu();
      
// }