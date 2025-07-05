//// Pin Definitions ////

#define GPS_PIN PIN_PD7
// #define RTC_PIN PIN_PD7
#define RINT PIN_PD6
#define AINT1 PIN_PD4
#define AINT2 PIN_PD5
#define LCS PIN_PA7
#define FCS PIN_PC2
#define LRST PIN_PA3
#define LDIO0 PIN_PF5
#define LDIO1 PIN_PF4

// EEPROM Struct // Contains Basics and setting info

struct meta
  {
    int gfrq;
    int gto;
    int hdop;
    uint16_t count;
    uint32_t wa;
    uint32_t ra;
  };


// EEPROM MetaData Address //

int eepromAddress = 1;

// Device ID //

const uint16_t tag = 10539;

// Firmware Version //
const float firmwareVersion = 3;