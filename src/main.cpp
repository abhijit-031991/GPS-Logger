#include <Arduino.h>
#include <elapsedMillis.h>
#include <TinyGPS++.h>
#include <time.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <time.h>
#include <TimeLib.h>
#include <Sleep_n0m1.h>

#define SERIALMENU_MINIMAL_FOOTPRINT true

///// LIBRARY DECLARATIONS /////
TinyGPSPlus gps;
SPIFlash flash(1);
elapsedMillis mTime;
SoftwareSerial gps_serial(19,20);
Sleep slp;

///// PIN DEFINITIONS /////
#define GPS_PIN A0

///// DEVICE DEFINITIONS /////
const uint16_t tag = 10401;

// GPS Storage Variables //
float lat;                        // Storing last known Latitude
float lng;                        // Storign last known Longitude

// Flash Adressess //
uint32_t wAdd = 0;                // Write Address Parameter
uint32_t rAdd = 0;                // Read Address Parameter

// GPS Control Variables //
int gpsFrequency = 60;            // GPS Frequency in minutes *** USER CONFIG ***
int gpsTimeout = 60;              // GPS Timesout after 'x' seconds *** USER CONFIG ***
int gpsHdop = 5;                  // GPS HODP Parameter *** USER CONFIG ***

// Device Variables //
uint16_t cnt;                     // No. of data points available for download

// Other Variables //
bool wipe = false;
bool gui = false;


//...................................//
//           FUNCTIONS               //
//...................................//

void recGPS(){
  mTime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println(gpsTimeout*1000);
  while (mTime <= (unsigned)gpsTimeout*1000)
  {
    while (gps_serial.available())
    {
      if (gps.encode(gps_serial.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println("Acquiring");
        }else{
          Serial.println(gps.location.isUpdated());
          Serial.print("Location Age:");
          Serial.println(gps.location.age());
          Serial.print("Time Age:");
          Serial.println(gps.time.age());
          Serial.print("Date Age:");
          Serial.println(gps.date.age());
          Serial.print("Satellites:");
          Serial.println(gps.satellites.value());
          Serial.print("HDOP:");
          Serial.println(gps.hdop.hdop());
        }       
      }      
    }
    if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000)
    {
      break;
    }  
  }   

  digitalWrite(GPS_PIN, LOW);
  struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
    }dat;

  if (gps.location.age() < 60000)
  {
    //pack data into struct
    lat = gps.location.lat();
    lng = gps.location.lng();
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
  }else{
    // pack data into struct with lat long = 0
    dat.lat = 0;
    dat.lng = 0;
  }
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    dat.datetime = (uint32_t)now();
    dat.locktime = mTime/1000;
    dat.hdop = gps.hdop.hdop();
    
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    wAdd = flash.getAddress(sizeof(dat));
    // Serial.println(sizeof(dat));
    Serial.println(wAdd);
    if (flash.writeAnything(wAdd, dat))
    {
      Serial.println(F("Write Successful"));
      cnt = cnt + 1;
    }else
    {
      Serial.println(F("Write Failed"));
      Serial.println(flash.error(VERBOSE));
    }     
  }else
  {
    Serial.println(F("Power Up Failed"));
  }   
  flash.powerDown();

}

void read_send(){ 
  struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
    }dat;

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      // dat.id = tag;
      Serial.print(dat.datetime);
      Serial.print(F(","));
      Serial.print(dat.hdop);
      Serial.print(F(","));
      Serial.print(dat.lat);
      Serial.print(F(","));
      Serial.print(dat.lng);
      Serial.print(F(","));
      Serial.println(dat.locktime);
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }
      
}

void setup() {
  // put your setup code here, to run once:
  enablePower(POWER_ADC);
  enablePower(POWER_SERIAL0);
  enablePower(POWER_SERIAL1);
  enablePower(POWER_SPI);
  enablePower(POWER_WIRE);

  Serial.begin(9600);
  gps_serial.begin(9600);
  delay(1000);

  Serial.print(F("#### Device ID : "));
  Serial.print(tag);
  Serial.println(F(" ####"));
 
  // Enable & Start Flash //
  
  if(flash.powerUp()){
    Serial.println(F("Powered Up1"));
  }
  if(!flash.begin()){
    Serial.println(F("Flash again"));
    Serial.println(flash.error(VERBOSE));
  } 
  Serial.println(flash.getManID());
  if(flash.powerUp()){
    Serial.println(F("Powered Up"));
  }else
  {
    Serial.println(F("PWR UP Failed!"));
  }
  if (wipe == true)
  {
    Serial.println(F("WIPING FLASH"));
    if(flash.eraseChip()){
    Serial.println(F("Memory Wiped"));  
    }else
    {
      Serial.println(flash.error(VERBOSE));
    }
  }else{
    rAdd = flash.getAddress(16);
  }    
  if(flash.powerDown()){
    Serial.println("Powered Down");
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

  
  
  delay(100);
  disablePower(POWER_ADC);
  disablePower(POWER_SERIAL0);
  disablePower(POWER_SERIAL1);
  disablePower(POWER_SPI);
  disablePower(POWER_WIRE); 
  
  sleepMode(SLEEP_POWER_DOWN);
  sleep();
}

void loop() {
  // put your main code here, to run repeatedly:
  enablePower(POWER_ADC);
  enablePower(POWER_SERIAL0);
  enablePower(POWER_SERIAL1);
  enablePower(POWER_SPI);
  enablePower(POWER_WIRE);

  recGPS();

  delay(100);
  disablePower(POWER_ADC);
  disablePower(POWER_SERIAL0);
  disablePower(POWER_SERIAL1);
  disablePower(POWER_SPI);
  disablePower(POWER_WIRE); 


  slp.pwrDownMode();
  slp.sleepDelay(gpsFrequency*60000);

}