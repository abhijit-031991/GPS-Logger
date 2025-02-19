#include <Arduino.h>
#include <elapsedMillis.h>
#include <TinyGPS++.h>
#include <time.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <time.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <PermaDefs.h>
#include <LoRa.h>
#include <Wire.h>

///// LIBRARY DECLARATIONS /////
TinyGPSPlus gps;
SPIFlash flash(LCS);
elapsedMillis mTime;
elapsedMillis bTime;

// ///// DEVICE DEFINITIONS /////

// GPS Storage Variables //
float lat;                        // Storing last known Latitude
float lng;                        // Storign last known Longitude

// Flash Adressess //
uint32_t wAdd;                // Write Address Parameter
uint32_t rAdd;                // Read Address Parameter

// GPS Control Variables //
int gpsFrequency = 15;            // GPS Frequency in minutes *** USER CONFIG ***
int gpsTimeout = 60;              // GPS Timesout after 'x' seconds *** USER CONFIG ***
int gpsHdop = 5;                  // GPS HODP Parameter *** USER CONFIG ***

// Device Variables //
uint16_t cnt;                     // No. of data points available for download

// Other Variables //
bool wipe = false;
bool gui = false;
bool sleeping = false;
unsigned long rtccounter = 0;


//...................................//
//           FUNCTIONS               //
//...................................//
void updateEeprom(int gf, int gt, int gh, uint16_t c, uint32_t w, uint32_t r){
  meta metaData;
  metaData.count = c;
  metaData.gfrq = gf;
  metaData.gto = gt;
  metaData.hdop = gh;
  metaData.ra = r;
  metaData.wa = w;
  EEPROM.put(eepromAddress, metaData);
}

void recGPS(){
  mTime = 0;
  double hdopstrt = 10.00;
  digitalWrite(GPS_PIN, HIGH);
  while ((mTime/1000) <= gpsTimeout)
  {
    // Serial.println(currentTime-xa);
    while (Serial1.available() > 0)
    {
      if (!gps.encode(Serial1.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println(F("Acquiring"));
        }      
      }else{
        // Serial.println(gps.location.isUpdated());
        // Serial.print(F("Location Age:"));
        // Serial.println(gps.location.age());
        // Serial.print(F("Time Age:"));
        // Serial.println(gps.time.age());
        Serial.print(F("HDOP Age:"));
        Serial.println(gps.hdop.age());
        // Serial.print(F("Satellites:"));
        // Serial.println(gps.satellites.value());
        Serial.print(F("HDOP:"));
        Serial.println(gps.hdop.value());
        if (gps.hdop.hdop() != 0.00)
        {
          hdopstrt = gps.hdop.hdop();
          Serial.println(hdopstrt);
        }
      }    
    }
    if (hdopstrt < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000 && gps.hdop.age() < 50)
    {
      break;
    }  
  }   

  digitalWrite(GPS_PIN, LOW);
  struct data{
    uint16_t count;
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
    dat.count = cnt;
    
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);
    Serial.println(dat.count);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    Serial.println((int)sizeof(dat));
    wAdd = flash.getAddress(sizeof(dat));
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

  updateEeprom(gpsFrequency, gpsTimeout, gpsHdop, cnt, wAdd, rAdd);

}

void read_send(){ 
  struct data{
    uint16_t count;
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
      Serial.print(dat.datetime);
      Serial.print(F(","));
      Serial.print(dat.hdop);
      Serial.print(F(","));
      Serial.print(dat.lat, 6);
      Serial.print(F(","));
      Serial.print(dat.lng, 6);
      Serial.print(F(","));
      Serial.print(dat.hdop);
      Serial.print(F(","));
      Serial.println(dat.locktime);     
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }else{
    Serial.println(F("Flash Power Up Failed"));
  }
  delay(100);
      
}

void showTitle(){
  Serial.println(F("#### ArcTrack GPS Logger ####"));
  Serial.println();
  Serial.print(F("#### Device ID : "));
  Serial.print(tag);
  Serial.println(F(" ####"));
  Serial.println();
  Serial.print(F("#### Firmware Version : "));
  Serial.print(firmwareVersion);
  Serial.println(F(" ####"));
  Serial.println();
}

void showMenu(){
    Serial.println();
    Serial.println(F("1 - Download All Data"));
    Serial.println(F("2 - Download New Data"));
    Serial.println(F("3 - Wipe All Data"));
    Serial.println(F("4 - Change GPS Frequency"));
    Serial.println(F("5 - Change GPS Timeout"));
    Serial.println(F("6 - Change HDOP Value"));
    Serial.println(F("7 - Sleep Mode - Requires Reset to Restart"));
    Serial.println(F("9 - Exit"));
    Serial.println();
    Serial.println(F("Select Number and Press Enter/Send"));
    Serial.println();Serial.println();
}

void eepromSettings(){  
  meta metaData;
  EEPROM.get(eepromAddress, metaData);
  if (metaData.gfrq == -1 || metaData.gto == -1 || metaData.gto == -1)
  {
    Serial.println(F("No Settings on EEPROM"));
    Serial.println(F("Writing Default Settings EEPROM"));
    metaData.count = cnt;
    metaData.gfrq = gpsFrequency;
    metaData.gto = gpsTimeout;
    metaData.hdop = gpsHdop;
    metaData.ra = 0;
    metaData.wa = 0;
    EEPROM.put(eepromAddress, metaData);

  }else{

    gpsFrequency = metaData.gfrq;
    gpsTimeout = metaData.gto;
    gpsHdop = metaData.hdop;
    rAdd = metaData.ra;
    wAdd = metaData.wa;
    cnt = metaData.count;
    Serial.println(F("#### CURRENT SETTINGS ####"));
    Serial.println();
    Serial.print(F("GPS FREQUENCY : ")); Serial.print(gpsFrequency); Serial.println(F(" Min/s"));
    Serial.print(F("GPS TIMEOUT : ")); Serial.print(gpsTimeout); Serial.println(F(" seconds"));
    Serial.print(F("GPS HDOP : ")); Serial.println(gpsHdop);
    Serial.print(F("DATA POINTS : ")); Serial.println(cnt);
    Serial.println();Serial.println();
  }
   
}

void interface(){
  bTime = 0;
  Serial.println(F("Press Z followed by Enter to Continue"));  
  while (bTime < 30000)
  {    
      if (Serial.available())
      {
          char x = Serial.read();
          Serial.println(x);
          if (x == 'z')
          {
              gui = true;
              showMenu();
              break;
          }        
      }    
  }
  if (gui == true)
  {
    bTime = 0;
    while (bTime < 300000)
    {
      int x = 11;
      if (Serial.available())
      {
          x = Serial.parseInt();
          Serial.println(x);
      }
      if (x == 1)
      {
          if (flash.powerUp())
          {
          Serial.println(F("Flash Powered On"));
          }
          if (flash.getAddress(16))
          {
            wAdd = flash.getAddress(16);
          }
          
          Serial.println(F("Starting Download"));
          rAdd = 0;
          do{
          read_send();
          rAdd = rAdd + 16;            
          } while (rAdd <= wAdd);
          Serial.println(F("Download Complete")); 
          if (flash.powerDown())
          {
          Serial.println(F("Flash Powered Off"));
          }
          
          delay(1000); 
          Serial.println(); 
          showMenu();         
      }
      if (x == 2)
      {
          Serial.println(F("Starting Download"));
          do{
          read_send();
          rAdd = rAdd + 16;            
          } while (rAdd <= wAdd);
          Serial.println(F("Download Complete")); 
          if (flash.powerDown())
          {
          Serial.println(F("Flash Powered Off"));
          }
          
          delay(1000); 
          Serial.println(); 
          showMenu();           
      }
      if (x == 3)
      {
          if (flash.powerUp())
          {
          Serial.println(F("WIPING FLASH MEMORY!!"));
          if(flash.eraseChip()){
          Serial.println(F("Memory Wiped"));  
          wAdd = 0;
          rAdd = 0;
          cnt = 0;
          }else
          {
              Serial.println(flash.error(VERBOSE));
          }
          }    
          if(flash.powerDown()){
          Serial.println("Powered Down");
          digitalWrite(FCS, HIGH);
          }else{
          Serial.println(flash.error(VERBOSE));
          }            
          delay(1000);
          Serial.println(); 
          showMenu();           
      }
      if (x == 4)
      {
          Serial.println(F("Enter New GPS Frequency"));
          while (!Serial.available())
          {}
          int a = Serial.parseInt();
          gpsFrequency = a;
          Serial.print(F("New GPS Frequency = "));Serial.println(gpsFrequency);
          delay(1000);
          Serial.println(); 
          showMenu();
      }
      if (x == 5)
      {
          Serial.println(F("Enter New GPS Timeout"));
          while (!Serial.available())
          {}
          int a = Serial.parseInt();
          gpsTimeout = a;
          Serial.print(F("New GPS Timeout = "));Serial.println(gpsTimeout);
          delay(1000);
          Serial.println(); 
          showMenu();           
      }
      if (x == 6)
      {
          Serial.println(F("Enter New HDOP Value"));
          while (!Serial.available())
          {}
          int a = Serial.parseInt();
          gpsHdop = a;
          Serial.print(F("New GPS HDOP = "));Serial.println(gpsHdop); 
          delay(1000);
          Serial.println(); 
          showMenu();          
      }if (x == 7)
      {
          Serial.println(F("Entring Sleep Mode"));
          Serial.flush();
          // RTC_init();
          set_sleep_mode(SLEEP_MODE_PWR_DOWN);
          sleep_enable();
          sleep_cpu();

      }
      if (x == 9)
      {
          gui = false;
          Serial.println(F("Exiting Menu"));
          Serial.println();
          delay(1500); 
          break;          
      }                    
    }
  }
  updateEeprom(gpsFrequency, gpsTimeout, gpsHdop, cnt, wAdd, rAdd);   
}

void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 16384, resulting in 32.768kHz/16384 = 2Hz */
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
  rtccounter = rtccounter + 1;
}

void setup(){

    Serial.begin(115200);
    Serial1.begin(9600);
    SPI.begin();
    delay(1000);
    
    Serial.println(F("SYSTEM INIT.."));

    if(!flash.powerUp()){
        Serial.println(flash.error(VERBOSE));
    }
    if (!flash.powerDown())
    {
      Serial.println(flash.error(VERBOSE));
    }
    if(!flash.powerUp()){
        Serial.println(flash.error(VERBOSE));
    }
    
    if(!flash.begin(MB(64))){
        Serial.println(F("Flash did not begin"));
        Serial.println(flash.error(VERBOSE));
    } 
    Serial.println(flash.getManID());
    if(!flash.powerUp()){
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
    }   
    if(flash.powerDown()){
        digitalWrite(FCS, HIGH);
    }else{
        Serial.println(flash.error(VERBOSE));
    }
    
    showTitle();
    eepromSettings();
    interface(); 
    
    digitalWrite(GPS_PIN, HIGH);
    Serial.println(F("GPS STARTED"));
        do{ 
          while (Serial1.available() > 0)
          {
            if (gps.encode(Serial1.read()))
            {
              if (!gps.location.isValid())
              {
                Serial.println(F("Not Valid"));
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
        }while(!gps.location.isValid());
      if (gps.location.age() < 60000)
      {
        //pack data into struct
        lat = gps.location.lat();
        lng = gps.location.lng();
      }
      if (gps.time.isValid())
      {
        setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
        
      }
        
      digitalWrite(GPS_PIN, LOW);
    
    RTC_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    Serial.println(F("SYSTEM READY"));
}

void loop(){

  if (sleeping == true)
  {
    if (rtccounter < gpsFrequency*60)
    {
      sleeping = true;
    }else{
      sleeping = false;
    }
  }else{
    recGPS();
    sleeping = true;
    rtccounter = 0;
  }
  Serial.flush();
  sleep_cpu();
      
}