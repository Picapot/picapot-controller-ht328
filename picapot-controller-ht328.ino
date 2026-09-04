//board=GyverCore:avr:nano:bod=bod_2_7,boot=no,clock=external_16,co=disabled,compiler_version=avrgcc5,es=disabled ,init=enable,serial=default,timers=yes_millis

//   <!--------------------------------------------------------------------------------------------->
//   <!--    PICAPOT 328 IRRIGATION CONTROLLER                                                    -->
//   <!--    Copyright (c) Fabrizio Ranieri                                                       -->
//   <!--    Hardware and instructions at www.picapot.com                                         -->
//   <!--                                                                                         -->
//   <!--    Licensed under the Open Community License v1.1 with the General Attribution          -->
//   <!--    add-on v1 (OCL v1.1 + GAtt v1). See the LICENSE file for the applicable terms.       -->
//   <!--                                                                                         -->
//   <!--    This software is provided without warranty of any kind, to the maximum extent        -->
//   <!--    permitted by applicable law.                                                         -->
//   <!--------------------------------------------------------------------------------------------->
#include <avr/wdt.h>              // watchdog library https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
#include <avr/sleep.h>            // sleep mode library https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
#include <avr/power.h>            // power control library https://www.nongnu.org/avr-libc/user-manual/group__avr__power.html 
#include <Wire.h>                 // I2C protocol library https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h
#include <OneWire.h>              // 1-Wire protocol library https://github.com/PaulStoffregen/OneWire
#include <EEPROM.h>               // EEPROM read/write library https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/EEPROM
#include <Dusk2Dawn.h>            // daily dusk and dawn library https://github.com/Picapot/Dusk2Dawn fork of https://github.com/dmkishi/Dusk2Dawn 
#include "SSD1306AsciiSpi.h"      // library for the screen  https://github.com/Picapot/SSD1306Ascii fork of https://github.com/greiman/SSD1306Ascii

// USER SETTINGS ////////////////////////////////////////////////////////
#define DIAGNOSTIC 0                  // 0 = exclude diagnostic screen 1 = include it
#define SCREEN_CONTRAST 32            // Range: 0–255
#define SKIP_ICE 5                    // Skip watering when temperature is below this limit (°C); 0 = disabled
#define SKIPDAYS_WINTER_PERC 25       // Auto mode: below this %, skip 2 days between waterings and use max 1 watering/day
#define SKIPDAYS_EARLY_SPRING_PERC 45 // Auto mode: below this %, skip 1 day between waterings and use max 1 watering/day
#define SKIPDAYS_LATE_SPRING_PERC 63  // Auto mode: below this %, water daily; in the 45-66% range WT2 runs when today's max temperature is >= the threshold or the sensor is unavailable; from 67%, water twice per day
#define DOUBLE_WATERING_TEMP 35       // Celsius; in the 45-66% season range skip WT2 only when a valid maximum temperature for today is below this value
#define DEFAULT_LONGITUDE +14         // Default longitude after reset (Malta); reset by restarting without the CR2032 backup battery
#define DEFAULT_LATITUDE +35          // Default latitude after reset (Malta)
#define DEFAULT_TIMEZONE +4           // Default timezone after reset; expressed in 15-minute increments; +4 = +01:00 (e.g. Malta); Venezuela (-04:30) = -18
#define DST_DISABLED 0
#define DST_EUROPE 1
#define DST_USA 2
#define DST_AUSTRALIA 3
#define DST_NEW_ZEALAND 4
#define DEFAULT_DAYLIGHT_SAVING DST_EUROPE // Default daylight saving mode after reset
#define AUTORESTORE 1                 // Automatically restore system after memory failure if backup is available; 0 = no, 1 = yes. Date and time must be adjusted, otherwise the watering schedule will be incorrect.

// SYSTEM SETTINGS ////////////////////////////////////////////////////////
#define SCREEN_TYPE &Adafruit128x64   // Use &SH1106_128x64 for SH1106 displays and &Adafruit128x64 for SSD1306/SSD1309 displays
#define CLOCK_ADDRESS 0x68            // I2C address of the DS1307 real-time clock
#define BLINK_FREQ 200                // Blinking interval in ms for edit mode
#define EDIT_FREQ 70                  // Button processing interval; controls auto increment/decrement speed when buttons 2 and 3 are held in edit mode
#define DEBOUNCE_DELAY 25             // Button debouncing delay in ms
#define SYSTEM_DELAY 5                // Delay in ms after each wire bulk read/write operation
#define SOIL_SENSOR_MIN 1             // Lower bound offset for the soil sensor value
#define SOIL_SENSOR_MAX 65535         // Upper bound offset for the soil sensor value
#define SOIL_SENSOR_INVERTED 0        // 0 = low value means low moisture, 1 = low value means high moisture

// PINS ////////////////////////////////////////////////////////
#define PIN_INT0 2                   // Interrupt-0 connected to the RTC 1Hz square wave output
#define PIN_LED 0                    // Red LED
#define PIN_ONE_WIRE 7               // I/O pin for the humidity sensor (1-Wire protocol)
#define PIN_SPI_CS  9                // SPI screen CS pin
#define PIN_SPI_RST A1               // SPI screen RST pin
#define PIN_SPI_DC  10               // SPI screen DC pin    
#define PIN_BTN1 1                   // Button 1 (left)
#define PIN_BTN2 4                   // Button 2 (middle)
#define PIN_BTN3 3                   // Button 3 (right)
#define PIN_OUT 5                    // MOSFET gate (controls the solenoid valve)    
#define PIN_BAT_CHECK A2             // Analog input for battery voltage measurement
#define PIN_DS18B20_VC 8             // Temperature sensor power supply


//MACROS ////////////////////////////////////////////////////////
#define SWAP(a, b) { int16_t t = a; a = b; b = t; }

//FONTS ////////////////////////////////////////////////////////
// Free Font LCD5x7 revisited using GLCD Font Creator https://www.mikroe.com/glcd-font-creator. 
// Some characters have been changed to icons:
// '=TOP ARROW  ,=BOTTOM ARROW  *=WATER DROP  @=DEGREE CELSIUS  $=DEGREE FAHRENHEIT
// "=BATTERY LEFT SIDE  ;=RIGHT SIDE  _=EMPTY SEGMENT  `=FULL SEGMENT
// Some chars are reserved and used as text formatting elements:
// &=PLACEHOLDER  [=1x PIXEL SPACE  ]=4x PIXEL SPACE  ^=LINEFEED
GLCDFONTDECL(lcd5x7m) = {
  0x00, 0x00, 0x05, 0x07, 0x20, 0x41, // 0, 0, width, height, first char, char count(65)
  0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x4F, 0x00, 0x00,   0x00, 0x00, 0x00, 0x3E, 0x3E,   0x04, 0x04, 0x04, 0x04, 0x04,   0x02, 0x05, 0x7E, 0x14, 0x04, // SPACE ! " # $
  0x23, 0x13, 0x08, 0x64, 0x62,   0x36, 0x49, 0x55, 0x22, 0x50,   0x0C, 0x06, 0x7F, 0x06, 0x0C,   0x00, 0x1C, 0x22, 0x41, 0x00,   0x00, 0x41, 0x22, 0x1C, 0x00, // % & ' ( )
  0x38, 0x46, 0x47, 0x5C, 0x38,   0x08, 0x08, 0x3E, 0x08, 0x08,   0x18, 0x30, 0x7F, 0x30, 0x18,   0x08, 0x08, 0x08, 0x08, 0x08,   0x00, 0x60, 0x60, 0x00, 0x00, // * + , - .
  0x20, 0x10, 0x08, 0x04, 0x02,   0x3E, 0x51, 0x49, 0x45, 0x3E,   0x00, 0x42, 0x7F, 0x40, 0x00,   0x42, 0x61, 0x51, 0x49, 0x46,   0x21, 0x41, 0x45, 0x4B, 0x31, // / 0 1 2 3
  0x18, 0x14, 0x12, 0x7F, 0x10,   0x27, 0x45, 0x45, 0x45, 0x39,   0x3C, 0x4A, 0x49, 0x49, 0x30,   0x01, 0x71, 0x09, 0x05, 0x03,   0x36, 0x49, 0x49, 0x49, 0x36, // 4 5 6 7 8
  0x06, 0x49, 0x49, 0x29, 0x1E,   0x00, 0x00, 0x24, 0x00, 0x00,   0x3E, 0x00, 0x00, 0x00, 0x00,   0x08, 0x14, 0x22, 0x41, 0x00,   0x00, 0x14, 0x14, 0x14, 0x00, // 9 : ; < =
  0x00, 0x41, 0x22, 0x14, 0x08,   0x02, 0x01, 0x51, 0x09, 0x06,   0x02, 0x05, 0x3A, 0x44, 0x44,   0x7E, 0x11, 0x11, 0x11, 0x7E,   0x7F, 0x49, 0x49, 0x49, 0x36, // > ? @ A B
  0x3E, 0x41, 0x41, 0x41, 0x22,   0x7F, 0x41, 0x41, 0x22, 0x1C,   0x7F, 0x49, 0x49, 0x49, 0x41,   0x7F, 0x09, 0x09, 0x09, 0x01,   0x3E, 0x41, 0x49, 0x49, 0x7A, // C D E F G
  0x7F, 0x08, 0x08, 0x08, 0x7F,   0x00, 0x41, 0x7F, 0x41, 0x00,   0x20, 0x40, 0x41, 0x3F, 0x01,   0x7F, 0x08, 0x14, 0x22, 0x41,   0x7F, 0x40, 0x40, 0x40, 0x40, // H I J K L
  0x7F, 0x02, 0x0C, 0x02, 0x7F,   0x7F, 0x04, 0x08, 0x10, 0x7F,   0x3E, 0x41, 0x41, 0x41, 0x3E,   0x7F, 0x09, 0x09, 0x09, 0x06,   0x3E, 0x41, 0x51, 0x21, 0x5E, // M N O P Q
  0x7F, 0x09, 0x19, 0x29, 0x46,   0x46, 0x49, 0x49, 0x49, 0x31,   0x01, 0x01, 0x7F, 0x01, 0x01,   0x3F, 0x40, 0x40, 0x40, 0x3F,   0x1F, 0x20, 0x40, 0x20, 0x1F, // R S T U V
  0x3F, 0x40, 0x30, 0x40, 0x3F,   0x63, 0x14, 0x08, 0x14, 0x63,   0x07, 0x08, 0x70, 0x08, 0x07,   0x61, 0x51, 0x49, 0x45, 0x43,   0x00, 0x7F, 0x41, 0x41, 0x00, // W X Y Z [
  0x02, 0x04, 0x08, 0x10, 0x20,   0x00, 0x41, 0x41, 0x7F, 0x00,   0x04, 0x02, 0x01, 0x02, 0x04,   0x22, 0x22, 0x22, 0x22, 0x22,   0x3E, 0x3E, 0x3E, 0x3E, 0x3E  // \ ] ^ _ `

};


//Time font
GLCDFONTDECL(lucida10x14) = {
0x00, 0x00, 0x0B, 0x10, 0x2F, 0x0C, // 0, 0, width (11), height (16), first char, char count(12)
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char SPACE
0xF8, 0xFE, 0x06, 0x03, 0x83, 0xC3, 0x63, 0x33, 0x1E, 0xFE, 0xF8, 0x07, 0x1F, 0x1E, 0x33, 0x31, 0x30, 0x30, 0x30, 0x18, 0x1F, 0x07,  //  Code for char 0  
0x00, 0x00, 0x0C, 0x0C, 0x0E, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x3F, 0x3F, 0x30, 0x30, 0x30, 0x00,  //  Code for char 1  
0x1C, 0x1E, 0x07, 0x03, 0x03, 0x83, 0xC3, 0xE3, 0x77, 0x3E, 0x1C, 0x30, 0x38, 0x3C, 0x3E, 0x37, 0x33, 0x31, 0x30, 0x30, 0x30, 0x30,  //  Code for char 2  
0x0C, 0x0E, 0x07, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xE7, 0x7E, 0x3C, 0x0C, 0x1C, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x39, 0x1F, 0x0E,  //  Code for char 3  
0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x3F, 0x3F, 0x03, 0x03,  //  Code for char 4  
0x3F, 0x7F, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0xE3, 0xC3, 0x83, 0x0C, 0x1C, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x38, 0x1F, 0x0F,  //  Code for char 5  
0xC0, 0xF0, 0xF8, 0xDC, 0xCE, 0xC7, 0xC3, 0xC3, 0xC3, 0x80, 0x00, 0x0F, 0x1F, 0x39, 0x30, 0x30, 0x30, 0x30, 0x30, 0x39, 0x1F, 0x0F,  //  Code for char 6  
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xC3, 0xF3, 0x3F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x30, 0x3C, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00,  //  Code for char 7  
0x00, 0xBC, 0xFE, 0xE7, 0xC3, 0xC3, 0xC3, 0xE7, 0xFE, 0xBC, 0x00, 0x0F, 0x1F, 0x39, 0x30, 0x30, 0x30, 0x30, 0x30, 0x39, 0x1F, 0x0F,  //  Code for char 8  
0x3C, 0x7E, 0xE7, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xE7, 0xFE, 0xFC, 0x00, 0x00, 0x30, 0x30, 0x30, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x00,  //  Code for char 9  
0x00, 0x00, 0x00, 0x70, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00  //  Code for char :     

};

// GLOBAL CONSTANTS ////////////////////////////////////////////////////////
// Settings array with default values. Each setting has a comment with: array position, DS1307 memory hex address, (default value), [lower-upper bounds], description and [global variable name]
 // Default settings array; each entry includes: index, DS1307 address, (default), [min–max], description, and [variable name]
#define SETTINGS_COUNT 50
const byte defSettings[SETTINGS_COUNT] PROGMEM = {
  0x00,                           // 0  0x08  (0)   [0-1]     Temperature unit: 0 = Celsius, 1 = Fahrenheit [tempUnit]
  DEFAULT_TIMEZONE+48,            // 1  0x09  (52)  [0-104]   Timezone (-12.00 to +14.00) in 15-minute steps; default 52 = +01:00 (Malta) [timezone]
  DEFAULT_LATITUDE+90,            // 2  0x0A  (125) [0-180]   Latitude (-90 to +90); default Malta (+35, +14) [latitude]
  (DEFAULT_LONGITUDE+180)%256,    // 3  0x0B  (194) [0-255]   Longitude low byte (-180 to +180) [longitude]
  (DEFAULT_LONGITUDE+180)/256,    // 4  0x0C  (0)   [0-1]     Longitude high byte [longitude]
  0x01,                           // 5  0x0D  (1)   [1-60]    Screen saver timeout (minutes) [screenSaver]
  0x00,                           // 6  0x0E  (0)   [0-2]     Sensor type: 0 = internal DS18B20, 1 = external DS18B20, 2 = external Picapot [sensorType]
  DEFAULT_DAYLIGHT_SAVING,        // 7  0x0F  (1)   [0-4]     Daylight saving mode: 0 = OFF, 1 = EU, 2 = US, 3 = AU, 4 = NZ [daySav]
  0x00,                           // 8  0x10  (0)   [0-8]     Season compensation (0–400%) 0=OFF [adjWater]
  0xF0,                           // 9  0x11  (240) [0-255]   RTC calibration low byte (-240 to +240) [clockCalibration]
  0x00,                           // 10 0x12  (0)   [0-1]     RTC calibration high byte [clockCalibration]
  0x00,                           // 11 0x13  (0)   [0-4]     Watering mode: 0=off, 1=fixed time, 2=auto dusk-to-dawn, 3=advanced dusk-to-dawn, 4=interval [wateringMode]
  0x07,                           // 12 0x14  (7)   [0-23]    watering 1 hour [alm1Time]
  0x00,                           // 13 0x15  (0)   [0-59]    watering 1 minute [alm1Time]
  0x04,                           // 14 0x16  (4)   [0-20]    Watering duration index (wateringDurations) [wateringDuration]
  0x00,                           // 15 0x17  (0)   [0-99]    Empty last-watering marker [wateringLog[0]]
  0x01,                           // 16 0x18  (1)   [1-12]    watering 1 last trigger month [wateringLog[0]]
  0x01,                           // 17 0x19  (1)   [1-31]    watering 1 last trigger day [wateringLog[0]]
  0x02,                           // 18 0x1A  (2)   [0-3]     watering daylight offset: 0=dawn+1h→dusk−2h, 1=dawn+2h→dusk−3h, 2=dawn+3h→dusk−4h(auto), 3=dawn+4h→dusk−5h [d2dMode]
  0x13,                           // 19 0x1B  (19)  [0-23]    watering 2 hour [alm2Time]
  0x00,                           // 20 0x1C  (0)   [0-59]    watering 2 minute [alm2Time]
  0x09,                           // 21 0x1D  (9)   [0-24]    Fixed time watering 1 hour [exact1h]
  0x00,                           // 22 0x1E  (0)   [0-99]    Empty event-log marker [wateringLog[1]]
  0x01,                           // 23 0x1F  (1)   [1-12]    most recent event month [wateringLog[1]]
  0x01,                           // 24 0x20  (1)   [1-31]    most recent event day [wateringLog[1]]
  0x01,                           // 25 0x21  (1)   [0-7]     Days to skip: 0=OFF, 1=AUTO, 2=1 day … 7=6 days [skpDays]
  0x00,                           // 26 0x22  (0)   [0-99]    Skip based on humidity [skpHumidity]; 0 = OFF
  0x4A,                           // 27 0x23  (74)  [0-255]   Minimum daylight minutes (low byte); Malta = 586 [minDaylightMinutes]
  0x02,                           // 28 0x24  (2)   [0-255]   Minimum daylight minutes (high byte)
  0x68,                           // 29 0x25  (104) [0-255]   Maximum daylight minutes (low byte); Malta = 872 [maxDaylightMinutes]
  0x03,                           // 30 0x26  (3)   [0-255]   Maximum daylight minutes (high byte)
  0x00,                           // 31 0x27  (0)   [0-100]   Seasonal water adjustment result [adjWaterResult]
  0x00,                           // 32 0x28  (0)   [0-9]     Chosen interval index (wateringIntervals) for interval mode [almInterval]
  0x0E,                           // 33 0x29  (14)  [0-26]    Heatwave threshold (°C); 0=OFF, 1–26 → 30–55°C [heatWave]
  0x00,                           // 34 0x36  (0)   [0-59]    Fixed time watering 1 minute [exact1m]
  0x12,                           // 35 0x37  (18)  [0-24]    Fixed time watering 2 hour [exact2h]
  0x00,                           // 36 0x38  (0)   [0-59]    Fixed time watering 2 minute [exact2m]
  0x03,                           // 37 0x39  (3)   [1-9]     Heat multiplier (10%–90%) [heatMultiplier]
  0x00,                           // 38 0x3A  (0)   [0-23]    Last watering 1 hour [wateringLog[0]]
  0x00,                           // 39 0x3B  (0)   [0-59]    Last watering 1 minute [wateringLog[0]]
  0x00,                           // 40 0x3C  (0)   [0-255]   Last watering 1 duration high byte [wateringLog[0]]
  0x00,                           // 41 0x3D  (0)   [0-255]   Last watering 1 duration low byte [wateringLog[0]]
  0x00,                           // 42 0x3E  (0)   Hour 0-23 in bits 0-4; LogEventType 0-6 in bits 5-7 [wateringLog[1]]
  0x00,                           // 43 0x3F  (0)   [0-59]    Most recent event minute [wateringLog[1]]
  0x00,                           // 44 0x40  (0)   [0-255]   Most recent event duration high byte; zero for skips [wateringLog[1]]
  0x00,                           // 45 0x41  (0)   [0-255]   Most recent event duration low byte; zero for skips [wateringLog[1]]
  0x00,                           // 46 0x42  (0)   [0-23]    start hour for mode=interval [intervalStartH]
  0x00,                           // 47 0x43  (0)   [0-59]    start minute for mode=interval [intervalStartM]
  0x18,                           // 48 0x44  (24)  [0-24]    End hour for interval mode; 24 means no end limit [intervalEndH]
  0x00                            // 49 0x45  (0)   [0-59]    end minute for mode=interval [intervalEndM]
};

struct dateTime {
  uint8_t second = 0;
  uint8_t minute = 0;
  uint8_t hour = 12;
  uint8_t dow = 2;
  uint8_t day = 1;
  uint8_t month = 1;
  uint8_t year = 0;
  uint8_t durationHigh = 0;
  uint8_t durationLow = 0;  
};

enum LogEventType : uint8_t {
  LOG_WATERING = 0,
  LOG_SKIP_DAYS,
  LOG_SKIP_MOISTURE,
  LOG_SKIP_ICE,
  LOG_SKIP_REPEAT,
  LOG_SKIP_DOUBLE,
  LOG_MANUAL
};
// Event encoding: bits 0..4 of hour store the hour (0..23), while bits 5..7 store
// LogEventType (0..7). The duration fields always contain seconds and are zero for skips.
const char logEventCodes[] PROGMEM = "WDMIRE";

// GLOBAL VARIABLES ////////////////////////////////////////////////////////
dateTime alm1Time, alm2Time; // only the hour/minute parts are used and they contain the watering scheduled time
dateTime dtm; // datetime of the clock, it is refreshed every interrupt
dateTime dtm_ds; // datetime of the clock including daylight saving
dateTime wateringLog[7]; // [0] = last real watering; [1..6] = six most recent watering/skip events
byte wateringMode, wateringDuration, d2dMode, almInterval, exact1h, daySav, screenSaver, skpHumidity, skpDays, exact2h, exact1m, heatWave, batteryLevel, tempUnit, sensorType; //options variables
int timezone, latitude, longitude, minDaylightMinutes, maxDaylightMinutes, adjWater, adjWaterResult, exact2m, clockCalibration, heatMultiplier; //options variables
int InternalVoltage;
byte manualWateringDuration = 4;
int temperatureSensorValue, temperatureSensorValueRaw, minTempValue = 850, maxTempValue;
unsigned int soilSensorValue, soilSensorValueRaw;
byte sensorErr=1; // count of consecutive communication errors with the sensor. Status is error until the first good sample.
byte sensorCount = 0;
byte intSensorIdx = 0, extSensorIdx = 0;
dateTime minTempDateTime, maxTempDateTime; // hour/minute parts of the structure containing the recording time of the min/max temperature of the day, day/month/year parts containing the date of the last temperature check
dateTime minHumDateTime, maxHumDateTime;
unsigned int minHumValue = 65535, maxHumValue, sleepCt = 0, idleCt = 0;
byte internalSensorAddress[8];
byte temperatureSensorAddress[2][8];
unsigned int soilMoistureSample[10] = {0}; // the soil moisture sensor value is an average of the last 10 samples
int temperatureSample[10] = {250}; // the temperature value is an average of the last 10 samples
byte sampleCt1=1; // acquired temperature samples
byte sampleCt2=1; // acquired humidity samples
byte intervalStartH, intervalStartM, intervalEndH, intervalEndM;
//The user interface of picapot 328 is organized in 8 screens.
const byte blinkPosCt[8] = {7, 11, 4, 7, 0, 0, 0, 1}; // total input cells of every screen, some screens have no edit mode and are set to zero.
byte calibration[30] = {0}; // clock calibration 240 bit array
const int8_t wateringDDAdjust[4] = {6, 12, 18, 24}; // values in minutes*10 of the dusk2dawn watering option
bool blinkStatus = false; // this value is switched every BLINK_FREQ
bool manualStop = false; // User can stop watering pressing the button 1 for more than 2 seconds in the main screen
bool editMode = false; // start in view mode
bool isSummerTime = false; //summertime option
bool btn1Keep = false, btn2Keep = false, btn3Keep = false; // become true when a button is kept pressed for more than 2 seconds
bool screenSleep = false, refreshNow = false; //sleep mode and refresh flag of the screen
byte AlmTrig = 0; // contains the currently firing watering. 0=none  1=watering-1  2=watering-2  3=manual
bool btn1Status = true, btn2Status = true, btn3Status = true; // pressing status of the buttons
bool lastBtn1Status = false, lastBtn2Status = false, lastBtn3Status = false; // button debounce status
bool skipButtonProcess = false; // flag to skip process of the button that woke up from sleep mode
bool isFirstLoop = true;
byte btn1 = 1, btn2 = 1, btn3 = 1; // processing status of the buttons: 0=pressed  1=not pressed  2=press process completed
byte pressCt = 0;
byte activeScreen = 1, blinkPos = 0; // current screen, current input cell of a screen in edit mode
static const int dater[] = {0,31,59,90,120,151,181,212,243,273,304,334}; // used by datediff function
unsigned long ms = 0, curAlmDuration = 0, lastBlink = 0; // used to keep track of the elapsed time in milliseconds
unsigned long lastPin1Debounce = 0, lastPin2Debounce = 0, lastPin3Debounce = 0; // button debounce timing
char cstr1[12]; // used to convert long type numbers when passed to the function Print. If more values are passed to the same function, each value must use a different cstr array
char cstr2[12]; // a long can have maximum 10 digits (2147483647), plus 1 digit sign if present, plus 1 digit for the string terminator = 12 digits total
char cstr3[12];
char cstr4[12];
char cstr5[17]; //extra 5 bytes used for the season progressbar
volatile byte intRTC = 0;
volatile byte intBTN = 0;

// STRINGS
const char *months[12] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
const char *weekDays[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
const int wateringIntervals[10] = {5, 10, 15, 30, 60, 120, 240, 480, 720, 1440}; // value in minutes of the watering intervals  
const int wateringDurations[21] = {15, 30, 60, 90, 120, 150, 180, 240, 300, 360, 420, 480, 540, 600, 900, 1200, 1500, 1800, 2400, 3000, 3600}; // value in seconds of the watering durations
const char *wateringModes[5] = {"OFF", "FIXED]TIME", "AUTO]DUSK2DAWN", "ADV.]DUSK2DAWN", "INTERVAL"};
const char *sensorTypes[3] = {"INTERNAL", "DS18B20", "PS100"};
const char *d2dModes[4] = {"DAWN+1H DUSK-1H", "DAWN+2H DUSK-2H", "DAWN+3H DUSK-3H", "DAWN+4H DUSK-4H"};
const char *dstModes[5] = {"OFF", "EU", "US", "AU", "NZ"};

const char almDurOpts_0[] PROGMEM = "15]SEC";
const char almDurOpts_1[] PROGMEM = "30]SEC";
const char almDurOpts_2[] PROGMEM = "1]MIN";
const char almDurOpts_3[] PROGMEM = "1.5]MIN";
const char almDurOpts_4[] PROGMEM = "2]MIN";
const char almDurOpts_5[] PROGMEM = "2.5]MIN";
const char almDurOpts_6[] PROGMEM = "3]MIN";
const char almDurOpts_7[] PROGMEM = "4]MIN";
const char almDurOpts_8[] PROGMEM = "5]MIN";
const char almDurOpts_9[] PROGMEM = "6]MIN";
const char almDurOpts_10[] PROGMEM = "7]MIN";
const char almDurOpts_11[] PROGMEM = "8]MIN";
const char almDurOpts_12[] PROGMEM = "9]MIN";
const char almDurOpts_13[] PROGMEM = "10]MIN";
const char almDurOpts_14[] PROGMEM = "15]MIN";
const char almDurOpts_15[] PROGMEM = "20]MIN";
const char almDurOpts_16[] PROGMEM = "25]MIN";
const char almDurOpts_17[] PROGMEM = "30]MIN";
const char almDurOpts_18[] PROGMEM = "40]MIN";
const char almDurOpts_19[] PROGMEM = "50]MIN";
const char almDurOpts_20[] PROGMEM = "1]HOUR";
const char* const almDurOpts[] PROGMEM = {almDurOpts_0, almDurOpts_1, almDurOpts_2, almDurOpts_3, almDurOpts_4, almDurOpts_5, almDurOpts_6, almDurOpts_7, almDurOpts_8, almDurOpts_9, almDurOpts_10, almDurOpts_11, almDurOpts_12, almDurOpts_13, almDurOpts_14, almDurOpts_15, almDurOpts_16, almDurOpts_17, almDurOpts_18, almDurOpts_19,  almDurOpts_20};

//first char sets the line number (0-7) where to print
const char sysmsg00[] PROGMEM = "0SCR OK";
const char sysmsg01[] PROGMEM = "2MEM OK";
const char sysmsg02[] PROGMEM = "1MEM FAIL";
const char sysmsg03[] PROGMEM = "0RESET";
const char sysmsg04[] PROGMEM = "0RESTORED";
const char sysmsg05[] PROGMEM = "1TIME LOST";
const char sysmsg06[] PROGMEM = "";
const char sysmsg07[] PROGMEM = "";
const char sysmsg08[] PROGMEM = "0RESTART";
const char sysmsg09[] PROGMEM = "4CLK ER";
const char sysmsg10[] PROGMEM = "4CLK OK";
const char sysmsg11[] PROGMEM = "6SNS ER";
const char sysmsg12[] PROGMEM = "6SNS OK";
const char sysmsg13[] PROGMEM = "0CALIB";
const char sysmsg14[] PROGMEM = "0E1";           //SaveSettings, UploadSettings
const char sysmsg15[] PROGMEM = "0E2";           //DownloadSettings
const char sysmsg16[] PROGMEM = "0]]]]SKIP (REPEAT)";
const char sysmsg17[] PROGMEM = "0]]]]]]]SKIP (ICE)";
const char sysmsg18[] PROGMEM = "0]]]]]SKIP (MOIST)";
const char sysmsg19[] PROGMEM = "";                 //unused; keep the slot to preserve message IDs
const char sysmsg20[] PROGMEM = "0STOP WATER FIRST";
const char sysmsg21[] PROGMEM = "1(HOLD LEFT BTN";
const char sysmsg22[] PROGMEM = "2ON TIME SCREEN)";
const char sysmsg23[] PROGMEM = "0^";           //blank line
const char sysmsg24[] PROGMEM = "0E3";           //GetDateTime
const char sysmsg25[] PROGMEM = "0E4";           //SetDateTime
const char sysmsg26[] PROGMEM = "0E5";           //ActivateClockOscillator
const char sysmsg27[] PROGMEM = "0SCHEDULE";
const char sysmsg28[] PROGMEM = "0LOCATION";
const char sysmsg29[] PROGMEM = "0OPTIONS";
const char sysmsg30[] PROGMEM = "0EVENTS";
const char sysmsg31[] PROGMEM = "0DIAGNOSTIC";
const char sysmsg32[] PROGMEM = "0SENSOR";
const char sysmsg33[] PROGMEM = "0MANUAL";
const char sysmsg34[] PROGMEM = "5SELECT]DURATION";
const char sysmsg35[] PROGMEM = "6HOLD]LEFT]BUTTON";
const char sysmsg36[] PROGMEM = "7WWW.PICAPOT.COM";
const char sysmsg37[] PROGMEM = "0]]]]]SKIP (DOUBLE)";
const char *const sysmsg[] PROGMEM = {sysmsg00, sysmsg01, sysmsg02, sysmsg03, sysmsg04, sysmsg05, sysmsg06, sysmsg07, sysmsg08, sysmsg09, sysmsg10, sysmsg11, sysmsg12, sysmsg13, sysmsg14, sysmsg15, sysmsg16, sysmsg17, sysmsg18, sysmsg19, sysmsg20, sysmsg21, sysmsg22, sysmsg23, sysmsg24, sysmsg25, sysmsg26, sysmsg27, sysmsg28, sysmsg29, sysmsg30, sysmsg31, sysmsg32, sysmsg33, sysmsg34, sysmsg35, sysmsg36, sysmsg37};

// GLOBAL CLASSES ////////////////////////////////////////////////////////
SSD1306AsciiSpi Screen;   // create an instance of the SPI screen class
OneWire oneWire(PIN_ONE_WIRE); // initialize the OneWire protocol used to communicate with the sensor
// End of declarations -----------------------------------------

// Preserve the reset cause across the normal C/C++ startup initialization.
uint8_t mcusrMirror __attribute__((section(".noinit")));

// Disable the watchdog before global constructors and setup() are executed.
// After a watchdog reset, the ATmega328P may otherwise restart with a timeout
// of approximately 16 ms. The .init3 section determines execution order,
// regardless of this function's position in the source file.
void disableWatchdogEarly()
  __attribute__((naked))
  __attribute__((section(".init3")));

void disableWatchdogEarly()
{
  mcusrMirror = MCUSR;
  MCUSR = 0;       // WDRF must be cleared before disabling the watchdog
  wdt_disable();
}



////////////////////////////////////////////////////  SETUP  ////////////////////////////////////////////////////////////////////
void setup() {
  byte temperatureSensorAddressTmp[8] = {0};
  byte tmp[SETTINGS_COUNT+1]; 
  
  // Reset the system if the main program does not complete a loop within
  // approximately 8 seconds.
  wdt_enable(WDTO_8S);
  wdt_reset();
  memset(tmp, 0, sizeof(tmp));
  Wire.begin();
  delay(10);
  pinMode(PIN_DS18B20_VC, OUTPUT); 
  digitalWrite(PIN_DS18B20_VC, HIGH); //switch on the temp sensor
  pinMode(PIN_LED, OUTPUT); // prepare the red led pin
  Blink(150, 300); //blink the led for the first time to communicate that the microcontroller is in good working order and it has started the setup process
 
  Screen.begin(SCREEN_TYPE, PIN_SPI_CS, PIN_SPI_DC, PIN_SPI_RST); // intialize the SPI screen class
  Screen.setContrast(SCREEN_CONTRAST); 
  Screen.clear();
  printMsg(0); //SCR OK
  delay(300);
  
  analogReference(DEFAULT);
  pinMode(PIN_INT0, INPUT_PULLUP); // receives the interrupt from the clock square wave 1Hz. The system goes to sleep mode after processing a loop(), and it is woke up by this signal every second
  pinMode(PIN_BTN1, INPUT_PULLUP); // The button 1 is the setup button. In sleep mode it has an interrupt attached to wake up
  pinMode(PIN_BTN2, INPUT_PULLUP); // The button 2 is the first navigation button. In sleep mode it has an interrupt attached to wake up
  pinMode(PIN_BTN3, INPUT_PULLUP); // The button 3 is the second navigation button. In sleep mode it has an interrupt attached to wake up
  PCICR  |= bit (PCIE2); // Enable pin-change interrupts for the three buttons and clear any pending interrupt flag.
  PCMSK2 |= bit (PCINT17);
  PCMSK2 |= bit (PCINT19);
  PCMSK2 |= bit (PCINT20);
  PCIFR  |= bit (PCIF2);    
  pinMode(PIN_OUT, OUTPUT); // This pin is used to send the signal to the MOSFET that switches on the normally closed (NC) solenoid valve
  digitalWrite(PIN_OUT, LOW); // start with the valve off (closed)
  pinMode(PIN_BAT_CHECK, INPUT); //this pin is used to sense the battery voltage
  delay(10);
  // Before continuing check settings validity using the CRC algorithm
  // If the CRC is wrong, the user must decide if (1) writing the default settings, (2) recover from the backup (if present) or (3) rebooting the system. 
  // CRC can be wrong for one of the following reasons:
  // 1) settings has never been written yet to the DS1307 memory
  // 2) the DS1307 backup battery is empty or missing or replaced and the system is rebooting
  // 3) a catastrophic failure of the DS1307 memory
  bool hasBackup;
  DownloadSettings(tmp);
  if (oneWire.crc8(tmp, SETTINGS_COUNT) != tmp[SETTINGS_COUNT])
  {
    Screen.clear();
    printMsg(2); //MEMORY FAIL!
    DownloadSettingsEeprom(tmp);
    hasBackup=oneWire.crc8(tmp, SETTINGS_COUNT) == tmp[SETTINGS_COUNT];
    Screen.set1X();
    Print(0, 4, false, F(" RESET]]& RETRY^"), hasBackup ? "RESTORE" : "       ", "", "", "", "");
    Screen.set2X();
    Print(0, 6, false, F(" ]]|  ][[[&]][  |^"), hasBackup ? "," : " ", "", "", "", "");
    while (1 == 1) {
      if (!hasBackup || digitalRead(PIN_BTN1) == 0) { 
        Screen.clear();
        printMsg(3); //RESET
        memcpy_P(tmp, defSettings, SETTINGS_COUNT);
        UploadSettings(tmp);
        dtm.year = 26; // RTC default date is 1 Jan 2026; year 0 is reserved for empty log entries.
        SetDateTime(dtm);
        for (byte x = 0; x <= SETTINGS_COUNT; x++ ) { //delete the eeprom backup
            EEPROM.write(100+x, 255);
        }
        while (1) {}; // block the execution and wait for watchdog reset. settings will be checked again after reboot 
      }
      if ((digitalRead(PIN_BTN2) == 0 || AUTORESTORE==1) && hasBackup) {     //restore the settings from the eeprom backup. 
        Screen.clear();
        UploadSettings(tmp);
        printMsg(4); //SETTINGS RESTORED
        printMsg(5); //DATETIME LOST
        while (1) {}; // block the execution and wait for watchdog reset
      }        

      if (digitalRead(PIN_BTN3) == 0) {
        Screen.clear();
        printMsg(8); //RESTART
        while (1) {}; // block the execution and wait for watchdog reset
      }
      delay(SYSTEM_DELAY);
      resetWdt(); // extend the watchdog reset
    }
  }

  Settings2Variables(tmp); // convert the settings array to global variables
  minTempDateTime.year = 20; //minimum and maximum values of the day for temperature and moisture is reset at every reboot
  maxTempDateTime.year = 20; //minimum and maximum values of the day for temperature and moisture is reset at every reboot
  minHumDateTime.year = 20;
  maxHumDateTime.year = 20;  
  CreateCalibrationMatrix(); // Initialize the array of the Clock calibration functionality
  //adjWaterResult=50;
  InternalVoltage = readVcc();  
  printMsg(1); //MEM OK
  delay(300);
  ActivateClockOscillator(); //activate the clock oscillator
  delay(33);
  //activate the DS1307 1Hz pulse, used as an interrupt to wake up from sleep mode and as a timer for refreshing the screen when active
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x07);
  Wire.write((uint8_t)dec2bcd(10));
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(9); //CLK ERR
    while (1) {};
  } else {
    printMsg(10); //CLK OK
    delay(300);
  }
  resetWdt();
  batteryLevel = getBatteryLevel();


  //1-wire sensor
  oneWire.target_search(0x28);
  if ( !oneWire.search(temperatureSensorAddressTmp)) {
    printMsg(11); //SNS ERR
    delay(300);
  }
  else {
    if (oneWire.reset()) {
      bool allFF = true;
      for (int i = 50; i <= 57; i++) {
        if (EEPROM.read(i) != 0xFF) {
          allFF = false;
          break;
        }
      }
      if (allFF) {
        oneWire.reset_search();
        oneWire.target_search(0x28);
        if (oneWire.search(temperatureSensorAddressTmp)) {
          for (int i = 0; i < 8; i++) {
            EEPROM.update(50 + i, temperatureSensorAddressTmp[i]);
          }
        }
      }


      for (int i = 0; i < 8; i++) {
        internalSensorAddress[i] = EEPROM.read(50 + i);
      }
      getTempSensorAddress();

      //set 12 bits resolution on the internal ds18b20
      oneWire.select(internalSensorAddress);
      oneWire.write(0x4E, 0);
      oneWire.write(0x00, 0);
      oneWire.write(0x00, 0);
      oneWire.write(0x7F, 0);  

      // Request the first conversion for the selected DS18B20.
      // PS100 doesn't need it; an internal DS18B20 used as fallback does.
      if ((sensorType < 2 || extSensorIdx == intSensorIdx) && oneWire.reset()) {
        oneWire.select(
          temperatureSensorAddress[
            sensorType == 0 ? intSensorIdx : extSensorIdx
          ]
        );
        oneWire.write(0x44, 0);
      }

      printMsg(12); //SNS OK
      delay(600);
    }
    else
    {
      printMsg(11); //SNS ERR
      delay(300);
    }
  }
  Blink(150, 300);
  Blink(150, 300);
  Blink(150, 300);
  Screen.clear();
  attachInterrupt(INT0, ClockInterrupt, FALLING); // sets the 1Hz square wave of the clock DS1307 as an external interrupt (ClockInterrupt)
}
// End of setup
// =========================================================================================================================================




void loop()
{
  long i = 0;
  resetWdt(); // restart the 8 seconds counter of the watchdog
  byte _intBTN = intBTN; // copy volatile flags to local variables
  byte _intRTC = intRTC;
  if (_intBTN > 0) { // clear volatile flag
    intBTN = 0;
  }
  if (_intRTC > 0) { // clear volatile flag
    intRTC = 0;
  }
  if (_intBTN > 0) { // if a button is pressed, wake up from screen sleep mode,
    sleepCt = 0;
    if (screenSleep == true) { // only if the system is actually sleeping
      SwitchOnScreen();
      skipButtonProcess = true;
    }
  }
  if ((_intRTC > 0) || (_intBTN > 0) || (editMode == true)) { //one of the two interrupts has been triggered, or edit mode is ON
    dtm = GetDateTime(); // DS1307 keeps datetime without daylight saving which is added when printing at screen

    isSummerTime=IsSummerTime(dtm);
    if (isSummerTime)
    {
      dtm_ds = Add1Hour(dtm); 
    } else {
      dtm_ds = dtm;
    }
  }
  if (isFirstLoop)
  {
      CalculateNextWatering();
      isFirstLoop=false;
  }

  if ((_intRTC > 0) || (_intBTN > 0)) { // one of the two interrupts has been triggered
    if (_intRTC > 0) { // the interrupt comes from the 1Hz square wave
      Blink(1, 0); // blink the led for 1 millisecond

      if ((dtm_ds.second%2)==1) //every two seconds check the sensor
      {
        int tmptmp=0;
        if (sensorType > 0 && extSensorIdx == intSensorIdx && dtm_ds.second == 1) {
          getTempSensorAddress(); // retry the external sensor once per minute while using the internal fallback
        }
        getTemperatureAndHumidity_DS18B20(temperatureSensorAddress[sensorType == 0 ? intSensorIdx : extSensorIdx]);            

        //average of temperature
        if (sensorErr==0)
        {
          for (byte x = 9; x > 0; x--) {
            temperatureSample[x] = temperatureSample[x - 1];
          }
          temperatureSample[0] = temperatureSensorValueRaw;
          i = 0;
          for (byte x = 0; x < sampleCt1; x++) {
            i += temperatureSample[x];
          }
          temperatureSensorValue = i / sampleCt1;
          if (dateDiff(minTempDateTime, dtm_ds) != 0 || temperatureSensorValue < minTempValue) {
            minTempDateTime = dtm_ds;
            minTempValue = temperatureSensorValue;
          }
          if (dateDiff(maxTempDateTime, dtm_ds) != 0 || temperatureSensorValue > maxTempValue) {
            maxTempDateTime = dtm_ds;
            maxTempValue = temperatureSensorValue;
          }
          if (sampleCt1<10)
          {
            sampleCt1++;
          }
        }



        if (sensorErr==0)
        {
          //humidity
          i = soilSensorValueRaw;
          if (i>0)
          {
            if (i < SOIL_SENSOR_MIN)
            {
              i = SOIL_SENSOR_MIN;
            }
            i = i - SOIL_SENSOR_MIN;
            if (i > (SOIL_SENSOR_MAX - SOIL_SENSOR_MIN))
            {
              i = SOIL_SENSOR_MAX - SOIL_SENSOR_MIN;
            }
            for (byte x = 9; x > 0; x-- ) {
              soilMoistureSample[x] = soilMoistureSample[x - 1];
            }
            soilMoistureSample[0] = i;
            i = 0;
            for (byte x = 0; x < sampleCt2; x++ ) {
              i = i + soilMoistureSample[x];
            }
            soilSensorValue = i / sampleCt2;   
            //record the minimum and maximum of the day
            if (dateDiff(minHumDateTime, dtm_ds) != 0 || soilSensorValue < minHumValue) {
              minHumDateTime = dtm_ds;
              minHumValue = soilSensorValue;
            }
            if (dateDiff(maxHumDateTime, dtm_ds) != 0 || soilSensorValue > maxHumValue) {
              maxHumDateTime = dtm_ds;
              maxHumValue = soilSensorValue;
            }  
            if (sampleCt2<10)
            {
              sampleCt2++;
            }   
          }
          
        }

        if (sensorErr == 10)
        {
          memset(temperatureSample, 0, sizeof(temperatureSample));
          memset(soilMoistureSample, 0, sizeof(soilMoistureSample));

          temperatureSensorValue = 250;
          soilSensorValue = 0;
          soilSensorValueRaw = 0;

          sampleCt1 = 1;
          sampleCt2 = 1;

          getTempSensorAddress();
        }
      }


      
      //  every six minutes apply clock calibration using the clock calibration array (more info from the function CreateCalibrationMatrix).
      if (((dtm.hour * 60 + dtm.minute) % 6) == 0 && dtm.second == 20) { // apply the calibration after a few seconds to avoid concurrency with waterings
        if ((ReadCalibrationBit((dtm.hour * 60 + dtm.minute) / 6) == true) && (clockCalibration != 0)) {
          if (screenSleep == true) {
            SwitchOnScreen();
            delay(SYSTEM_DELAY);
          }
          sleepCt = (screenSaver*60) - 8; //switch ON the screen for 8 seconds during calibration
          Screen.clear();
          if (clockCalibration < 0) {
            dtm.second = 19;
          } else {
            dtm.second = 21;
          }
          SetDateTime(dtm);
          printMsg(13); //CALIB
          resetWdt();
          delay(3000);
        }
        resetWdt();
        // every six minutes save the day min/max temperature to the clock memory and recalculate adjWaterResult
        CalculateNextWatering();
        SaveSettings(false);
        // refresh battery level
        InternalVoltage = readVcc();
        batteryLevel = getBatteryLevel();
        //delete skip message
        if (activeScreen==1 & AlmTrig == 0)
        {
            printMsg(23);
        }
      } 
      else
      {
        if (activeScreen == 7)
        {
          InternalVoltage = readVcc();
          batteryLevel = getBatteryLevel();
        }
      }        // end of clock calibration
    } // end of 1Hz square wave processing

    if (AlmTrig == 0 && screenSleep == false) { // if no watering is triggered, increase the counter for the sleep mode
      sleepCt++;
    }
    CheckWaterings(); // check if there is a watering to switch ON
    SwitchOffWatering(); // check if there is a watering to switch OFF
    if (editMode == false) { // screen is refreshed every interrupt if not in edit mode
      refreshNow = true;
    }
  } // end of interrupts processing
  
  if (sleepCt > (screenSaver*60)) { // Switch off the screen after n seconds of inactivity
    screenSleep = true;
    sleepCt = 0;
    editMode = false;
    blinkPos = 0;
    pressCt = 0;
    activeScreen = 1;
    Screen.clear();
    Screen.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
    Screen.ssd1306WriteCmd(SH1106_SET_PUMP_MODE);
    Screen.ssd1306WriteCmd(SH1106_PUMP_OFF);
  }
  if (screenSleep == false) { // buttons are monitored every loop (don't process buttons during sleep mode and don't process buttons that woke up from sleep mode)
    MonitorButtons();
    if ((millis() - ms) > EDIT_FREQ) { // buttons are processed only every 70 ms (to have a proper input increase/decrease speed, when they are kept pressed)
      ms = millis();
      ProcessButtons();
      if ((editMode == true))  { // screen is refreshed only when necessary
        refreshNow = true;
      }
    }
    if ((millis() - lastBlink) > BLINK_FREQ) { // This to have the blink effect in edit mode (200 ms)
      blinkStatus = !blinkStatus;
      lastBlink = millis();
      if ((editMode == true) || ((AlmTrig != 0) && activeScreen == 1)) { // screen is refreshed only when necessary
        refreshNow = true;
      }
    }
  }
  if (screenSleep == false)
  {
    if (refreshNow == true) { // screen is not refreshed every loop but only when necessary
      PrintScreen(activeScreen);
      refreshNow = false;
    }
  }

  if (screenSleep == true) { // unless the screen is active, go to sleep mode, till the next 1Hz (1 second) square wave signal
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    noInterrupts ();
    power_all_disable();
    sleep_bod_disable();
    interrupts ();
    sleep_cpu();
    sleep_disable(); //wake up here
    power_all_enable();

  }

  delay(2);
} // End of loop
// ========================================================================================




void ClockInterrupt() { // 1Hz square wave signal from the clock
  intRTC++;
}


ISR (PCINT2_vect) { // buttons pin status change
  intBTN++;
}




void Settings2Variables(byte* data) { // convert the settings from an array of bytes to global variables
  tempUnit = data[0];
  timezone = data[1] - 48;
  latitude = data[2] - 90;
  longitude = (data[3] + data[4] * 256) - 180;
  screenSaver = data[5];
  sensorType = data[6];
  daySav = data[7] <= DST_NEW_ZEALAND ? data[7] : DEFAULT_DAYLIGHT_SAVING;
  adjWater = data[8];
  clockCalibration = (data[9] + data[10] * 256) - 240;
  wateringMode = data[11];
  alm1Time.hour = data[12];
  alm1Time.minute = data[13];
  wateringDuration = data[14];
  wateringLog[0].year = data[15];
  wateringLog[0].month = data[16];
  wateringLog[0].day = data[17];
  d2dMode = data[18];
  alm2Time.hour = data[19];
  alm2Time.minute = data[20];
  exact1h = data[21];
  wateringLog[1].year = data[22];
  wateringLog[1].month = data[23];
  wateringLog[1].day = data[24];
  skpDays = data[25];
  skpHumidity = data[26];
  minDaylightMinutes = data[27] + (data[28] * 256);
  maxDaylightMinutes = data[29] + (data[30] * 256);
  adjWaterResult = data[31];
  almInterval = data[32];
  heatWave = data[33];
  exact1m = data[34];
  exact2h = data[35];
  exact2m = data[36];
  heatMultiplier = data[37];
  wateringLog[0].hour = data[38];
  wateringLog[0].minute = data[39];
  wateringLog[0].durationHigh = data[40];
  wateringLog[0].durationLow = data[41];
  wateringLog[1].hour = data[42];
  wateringLog[1].minute = data[43];
  wateringLog[1].durationHigh = data[44];
  wateringLog[1].durationLow = data[45];  
  intervalStartH = data[46];
  intervalStartM = data[47];  
  intervalEndH = data[48];
  intervalEndM = data[49];  
}

void Variables2Settings(byte* data) { // convert the settings from global variables to an array of bytes
  data[0] = tempUnit;
  data[1] = timezone + 48;
  data[2] = latitude + 90;
  data[3] = (longitude + 180) % 256;
  data[4] = (longitude + 180) / 256;
  data[5] = screenSaver;
  data[6] = sensorType; 
  data[7] = daySav;
  data[8] = adjWater;
  data[9] = (clockCalibration + 240) % 256;
  data[10] = (clockCalibration + 240) / 256;
  data[11] = wateringMode;
  data[12] = alm1Time.hour;
  data[13] = alm1Time.minute;
  data[14] = wateringDuration;
  data[15] = wateringLog[0].year;
  data[16] = wateringLog[0].month;
  data[17] = wateringLog[0].day;
  data[18] = d2dMode;
  data[19] = alm2Time.hour;
  data[20] = alm2Time.minute;
  data[21] = exact1h;
  data[22] = wateringLog[1].year;
  data[23] = wateringLog[1].month;
  data[24] = wateringLog[1].day;
  data[25] = skpDays;
  data[26] = skpHumidity;
  data[27] = minDaylightMinutes % 256;
  data[28] = minDaylightMinutes / 256;
  data[29] = maxDaylightMinutes % 256;
  data[30] = maxDaylightMinutes / 256;
  data[31] = adjWaterResult;
  data[32] = almInterval;
  data[33] = heatWave;
  data[34] = exact1m;
  data[35] = exact2h;
  data[36] = exact2m;
  data[37] = heatMultiplier;
  data[38] = wateringLog[0].hour;
  data[39] = wateringLog[0].minute;
  data[40] = wateringLog[0].durationHigh;
  data[41] = wateringLog[0].durationLow;
  data[42] = wateringLog[1].hour;
  data[43] = wateringLog[1].minute;
  data[44] = wateringLog[1].durationHigh;
  data[45] = wateringLog[1].durationLow;
  data[46] = intervalStartH;
  data[47] = intervalStartM;
  data[48] = intervalEndH;
  data[49] = intervalEndM;
}


void AddLogEvent(byte eventType, unsigned int durationSeconds) {
  dateTime event = dtm_ds;
  event.hour = (event.hour & 0x1F) | (eventType << 5);

  if (eventType == LOG_WATERING || eventType == LOG_MANUAL) {
    event.durationHigh = durationSeconds / 256;
    event.durationLow = durationSeconds % 256;
    if (eventType == LOG_WATERING) {
      wateringLog[0] = event;
      wateringLog[0].hour &= 0x1F; // Last automatic watering stores only the hour, without the event type.
    }
  }
  else {
    event.durationHigh = 0;
    event.durationLow = 0;
  }

  for (byte i = 6; i > 1; i--) {
    wateringLog[i] = wateringLog[i - 1];
  }
  wateringLog[1] = event;
}


void FinishSkippedWatering() {
  delay(2500);
  AlmTrig = 0;
  sleepCt = 0;
  manualStop = false;
  CalculateNextWatering();
}



//check and trig waterings
void CheckWaterings() { 
  byte almNow = 0;
  bool skipDoubleWatering = false;
  bool isD2D = (wateringMode == 2 || wateringMode == 3);
  int daysToSkip = 0;

  int daysSinceLastWatering = dateDiff(wateringLog[0], dtm_ds);
  if (isD2D && daysSinceLastWatering>2 && (wateringLog[0].year!=0)) //if the system restars after more than 2 days of inactivity, and watering is in dusk/dawn mode, adjust the watering time
  {
    CalculateNextWatering();
  }
  
  if (wateringMode != 0  && alm1Time.hour < 24) { // check watering 1
    long alarmDiff = SecondDiff(alm1Time, dtm_ds);
    if (alarmDiff == 0 || alarmDiff == 1) { // use a timeframe of two seconds to check if the watering matches with the time
      almNow = 1;
    }
    if (SecondDiff(dtm_ds, alm1Time) == 10 && screenSleep == true) { // switch on the screen 10 seconds before the watering
      SwitchOnScreen();
    }
  }

  if (wateringMode != 0  && wateringMode != 4 && alm2Time.hour < 24) { // check watering 2
    long alarmDiff = SecondDiff(alm2Time, dtm_ds);
    if (alarmDiff == 0 || alarmDiff == 1) { // use a timeframe of two seconds to check if the watering matches with the time
      almNow = 2;
    }
    if (SecondDiff(dtm_ds, alm2Time) == 10 && screenSleep == true) { // switch on the screen 10 seconds before the watering is triggered
      SwitchOnScreen();
    }
  }

  long secDif=SecondDiff(wateringLog[0], dtm_ds);

  if ((!editMode) && AlmTrig == 0 && ((almNow == 1) || (almNow == 2))) { // one of the waterings is triggered
    AlmTrig = almNow; //set the global variable
    if (screenSleep == true) {
      SwitchOnScreen();
    }

    if (wateringMode == 2 || skpDays == 1) // skipdays is in auto mode
    {     
      //adjWaterResult contains a value indicating from 1 to 100 in which season of the year we are (calculated using the length of the day)
      daysToSkip = (adjWaterResult < SKIPDAYS_WINTER_PERC) ? 3 : (adjWaterResult < SKIPDAYS_EARLY_SPRING_PERC) ? 2 : (adjWaterResult < SKIPDAYS_LATE_SPRING_PERC) ? 0 : 0; 
      bool isMidSeason = adjWaterResult >= SKIPDAYS_EARLY_SPRING_PERC && adjWaterResult < SKIPDAYS_LATE_SPRING_PERC;
      bool validTodayMaxTemp = sensorErr == 0 && dateDiff(maxTempDateTime, dtm_ds) == 0;
      bool skipWt2ForTemperature = isMidSeason && validTodayMaxTemp && maxTempValue < (DOUBLE_WATERING_TEMP * 10);
      bool skipSecondWatering = adjWaterResult < SKIPDAYS_EARLY_SPRING_PERC || skipWt2ForTemperature;
      skipDoubleWatering = (daysSinceLastWatering == 0) && (isD2D || (wateringMode==1 && alm1Time.hour < 24 && alm2Time.hour < 24)) && skipSecondWatering;
    }
    else 
    {
      daysToSkip = skpDays; // skipdays value has been specified by the user, value 2 = 1 day
    }

    // If the watering is triggered with a wrong date that was set to the future, and then the date is set back correctly, the next watering will not be triggered until the first wrong date is reached (if skipdays is ON)
    // To fix this issue, we set the last triggered watering date to the current date.
    if (daysSinceLastWatering < 0)
    {
      wateringLog[0].day = dtm_ds.day;
      wateringLog[0].month = dtm_ds.month;
      wateringLog[0].year = dtm_ds.year;
      SaveSettings(false);
      daysSinceLastWatering = 1;
      secDif = 900;
    } // end of future date correction

    if (!skipDoubleWatering) // in auto mode, WT2 may be suppressed according to season and today's maximum temperature
    {
      if (!(isD2D && daysSinceLastWatering == 0 && secDif < 900)) // In D2D mode, prevent repeated triggers caused by very short watering durations.
      { 
        if ((wateringMode == 4) || ((wateringMode == 1 || wateringMode == 3) && (skpDays == 0)) || (daysToSkip <= daysSinceLastWatering)) // skip days is off, or mode=interval or skip days condition is true
        {
          if  (skpHumidity == 0 || sensorErr > 0 || sensorType<2 || extSensorIdx == intSensorIdx || soilSensorValueRaw == 0 || Moist(soilSensorValue) < (skpHumidity*10)) // skip humidity
          {
            if ((SKIP_ICE == 0) || sensorErr > 0 || (temperatureSensorValue >= (SKIP_ICE*10))) // skip ice
            {
              // WATERING!
              curAlmDuration = millis(); // used to keep track of the elapsed seconds  
              blinkPos = 0; // reset blinking cell position
              editMode = false; // start in view mode
              if (activeScreen != 1)
              {
                Screen.clear();
              }
              activeScreen = 1; // show default screen
              unsigned long m=WateringDuration(AlmTrig)/1000;
              AddLogEvent(LOG_WATERING, m);
              digitalWrite(PIN_OUT, HIGH); //open the valve
            }
            else
            {
              AddLogEvent(LOG_SKIP_ICE, 0);
              printMsg(17); //SKIP ICE
              FinishSkippedWatering();
            }
          }
          else
          {
            AddLogEvent(LOG_SKIP_MOISTURE, 0);
            printMsg(18); //SKIP HUMID
            FinishSkippedWatering();
          }
        }
        else {
          AddLogEvent(LOG_SKIP_DAYS, 0);
          Print(0, 0, false, F("SKIPDAY &/&^"), lngToChar(cstr1, daysSinceLastWatering, false), lngToChar(cstr2, daysToSkip - 1, false), "", "", ""); //SKIP DAYS
          FinishSkippedWatering();
        }
      }
      else
      {
        AddLogEvent(LOG_SKIP_REPEAT, 0);
        printMsg(16); //SKIP REPEAT
        FinishSkippedWatering();
      }    
    }
    else
    {
      AddLogEvent(LOG_SKIP_DOUBLE, 0);
      printMsg(37); //SKIP DOUBLE
      FinishSkippedWatering();
    }     
    SaveSettings(false); // watering state and events are stored only in the DS1307 RAM
  }
}


unsigned long WateringDuration(byte alm)
{
  unsigned long t;
  if (((wateringMode == 1 || wateringMode == 3) && adjWater == 0) || alm == 3) { // 0 = automatic adjustment is OFF
    t = wateringDurations[((alm == 1 || alm == 2) ? wateringDuration : manualWateringDuration)];
    t = t * 1000; 
  }
  else {
    
    t = wateringDurations[wateringDuration]; 
    t = t * (100+((adjWaterResult * (wateringMode == 2 ? 3 : adjWater) * 50)/100)) * 10; // automatic adjustment of the watering period, if skip watering=auto then water adjust=150%
  }

  if (heatWave != 0 && alm != 3 && sensorErr == 0) { // if option heat is ON and the limit temperature has been reached, apply the adjustment to the watering period
    if (maxTempValue >= ((heatWave + 29)*10)) {
      t = (t * (100 + (heatMultiplier*10))) / 100;
    }
  }
  return t;
}

void SwitchOffWatering() {
  if (AlmTrig == 1 || AlmTrig == 2 || AlmTrig == 3) {
    unsigned long mss = WateringDuration(AlmTrig);
    if ((manualStop) || ((millis() - curAlmDuration) > mss)) { // manual stop or watering elapsed
      bool stoppedManually = manualStop;
      if (stoppedManually) {
        unsigned int elapsedSeconds = (millis() - curAlmDuration) / 1000;

        wateringLog[1].durationHigh = elapsedSeconds / 256;
        wateringLog[1].durationLow = elapsedSeconds % 256;
        if (AlmTrig != 3) {
          wateringLog[0].durationHigh = wateringLog[1].durationHigh;
          wateringLog[0].durationLow = wateringLog[1].durationLow;
        }
      }
      sleepCt = 0;
      manualStop = false;
      if (AlmTrig != 3) {    
        CalculateNextWatering();
      }
      if (AlmTrig != 3 || stoppedManually) SaveSettings(false);
      AlmTrig = 0;
      Screen.clear();
      digitalWrite(PIN_OUT, LOW); // switch-off the gate signal
    }
  }
}



void CalculateNextWatering() {
  byte hh, mm;
  int tmm;
  int mm1, mm2;
  int nowMinutes, startMinutes, intervalMinutes;
  unsigned long f;
  float tmz = timezone;
  Dusk2Dawn D2D(latitude, longitude, tmz / 4);
  mm1 = D2D.sunrise(dtm.year + 2000, dtm.month, dtm.day, isSummerTime);
  mm2 = D2D.sunset(dtm.year + 2000, dtm.month, dtm.day, isSummerTime);
  if ((mm1 > 1440) || (mm2 > 1440)) {
    mm1 = 0;
    mm2 = 1440;
  }
  nowMinutes = dtm.hour * 60 + dtm.minute;
  if (isSummerTime) {
    nowMinutes += 60;
    if (nowMinutes >= 1440) nowMinutes -= 1440;
  }
  byte offset = (wateringMode == 2 ? wateringDDAdjust[2] : wateringDDAdjust[d2dMode]);
  for (byte alm = 1; alm < 3; alm++) {
    // ===== watering 1 =====
    if (alm == 1) {
      if (wateringMode == 1) { // fixed time
        alm1Time.hour = exact1h;
        alm1Time.minute = exact1m;
      }
      else if (wateringMode == 4) { // interval
        int endMinutes;
        int nextToday;
        bool hasEndLimit;

        intervalMinutes = wateringIntervals[almInterval];
        startMinutes = intervalStartH * 60 + intervalStartM;

        hasEndLimit = (intervalEndH < 24);

        if (hasEndLimit) {
          endMinutes = intervalEndH * 60 + intervalEndM;
          if (endMinutes < startMinutes) endMinutes = startMinutes;

          if (intervalMinutes >= 1440) {
            tmm = startMinutes;
          }
          else if (nowMinutes < startMinutes) {
            tmm = startMinutes;
          }
          else {
            nextToday = startMinutes + (((nowMinutes - startMinutes) / intervalMinutes) + 1) * intervalMinutes;

            if (nextToday <= endMinutes) {
              tmm = nextToday;
            }
            else {
              tmm = startMinutes;
            }
          }
        }
        else {
          if (intervalMinutes >= 1440) {
            tmm = startMinutes;
          }
          else {
            int delta = nowMinutes - startMinutes;
            if (delta < 0) delta += 1440;

            tmm = startMinutes + ((delta / intervalMinutes) + 1) * intervalMinutes;

            tmm %= 1440;
          }
        }
        splitMinutes(tmm, alm1Time.hour, alm1Time.minute);
      }
      else { // dawn/dusk
        tmm = mm1;
        tmm = tmm + (offset * 10);
        if (tmm < 0 || tmm > 1439) tmm = 0;
        splitMinutes(tmm, alm1Time.hour, alm1Time.minute);
      }
    }
    // ===== watering 2 =====
    else {
      if (wateringMode == 1) { // fixed time
        alm2Time.hour = exact2h;
        alm2Time.minute = exact2m;
      }
      else { // dawn/dusk (come prima)
        tmm = mm2;
        tmm = tmm - (offset * 10);
        if (tmm < 0 || tmm > 1439) tmm = 0;
        splitMinutes(tmm, alm2Time.hour, alm2Time.minute);
      }
    }
  }
  if (maxDaylightMinutes > minDaylightMinutes) {
    f = (((mm2 - mm1) - minDaylightMinutes) * 100) / (maxDaylightMinutes - minDaylightMinutes);// seasonal watering adjust
  }
  else {
    f = 100;
  }
  adjWaterResult = (byte)(f);
}

int StepMenu(int value, int minValue, int maxValue, bool down) {
  if (down) {
    return value == minValue ? maxValue : value - 1;
  }
  return value == maxValue ? minValue : value + 1;
}


void ProcessButtons() {
  if (btn1Status == 0 || btn2Status == 0 || btn3Status == 0) { // if one of the buttons is pressed
    pressCt = pressCt + 1; // increase the timer for entering the edit mode
    sleepCt = 0; // restart the timer for entering the sleep mode.
    idleCt = 0;
  }
  else {
    pressCt = 0; // restart the timer for exiting the edit mode
    idleCt = idleCt + 1;
  }

  if (btn1 == 0) {
    btn1 = 2;
    if (activeScreen == 2 && blinkPos == 1 && wateringMode == 0) { // skip blink positions if mode=OFF
      blinkPos = 1;
    } 
    else if (activeScreen == 2 && blinkPos == 2 && wateringMode == 1 && exact1h == 24) { // skip watering 1 minutes if hours=OFF
      blinkPos = 4;
    }
    else if (activeScreen == 2 && blinkPos == 4 && wateringMode == 1 && exact2h == 24) { // skip watering 2 minutes if hours=OFF
      blinkPos = 6;
    }
    else if (activeScreen == 2 && blinkPos == 10 && wateringMode == 1) { // skip blink positions if mode=exact
      blinkPos = 1;
    }
    else if (activeScreen == 2 && blinkPos == 1 && (wateringMode ==2 || wateringMode ==3)) { // skip blink positions of watering hours if the watering is automatic
      blinkPos = 8;
    } 
    else if (activeScreen == 2 && blinkPos == 8 && wateringMode == 2) { // skip blink positions if mode=auto
      blinkPos = 1;
    }
    else if (activeScreen == 2 && blinkPos == 2 && wateringMode == 4) { // skip blink positions if mode=interval
      blinkPos = 4;
    }  
    else if (activeScreen == 2 && blinkPos == 5 && wateringMode != 4) { // skip blink positions if mode=interval
      blinkPos = 8;
    }   
    else if (activeScreen == 2 && blinkPos == 9 && wateringMode == 4) { // skip blink positions if mode=interval
      blinkPos = 1;
    }   
    else if (activeScreen == 2 && blinkPos == 6 && wateringMode == 4 && intervalEndH == 24) { // skip blink positions if mode=interval
      blinkPos = 8;
    }               
    else if (activeScreen == 4 && blinkPos == 3 && heatWave == 0) { // skip blink positions if heatWave=OFF
      blinkPos = 5;
    }
    else {
      blinkPos = blinkPos + 1;
    }
    refreshNow = true;
    if (blinkPos == (blinkPosCt[activeScreen - 1] + 1)) {
      blinkPos = 1;
    }
  }

  if (btn2 == 0 || btn3 == 0) {
    refreshNow = true;
    if (editMode == false) { // In view mode the navigation buttons are used to switch between screens.
      if (skipButtonProcess) //don't process the button that woke up from sleep mode
      {
        skipButtonProcess = false;
        refreshNow = false;
      }
      else
      {
        if (activeScreen == 1 && btn2 == 0) {
          activeScreen = sizeof(blinkPosCt);
        }
        else if (activeScreen == sizeof(blinkPosCt) && btn3 == 0) {
          activeScreen = 1;
        }
        else {
          activeScreen = activeScreen + ((btn2 == 0) ? -1 : + 1);
        }
#if DIAGNOSTIC == 0
        if (activeScreen == 6) {
          activeScreen = activeScreen + ((btn2 == 0) ? -1 : + 1);
        }
#endif
        Screen.clear();
      }

    }
    else {
      //***********************************************************
      if (activeScreen == 1) { // SCREEN DATETIME
        bool down = (btn2 == 0);

        if (blinkPos == 1) {
          dtm.dow = StepMenu(dtm.dow, 1, 7, down);
        }
        else if (blinkPos == 2) {
          dtm.day = StepMenu(dtm.day, 1, 31, down);
        }
        else if (blinkPos == 3) {
          dtm.month = StepMenu(dtm.month, 1, 12, down);
        }
        else if (blinkPos == 4) {
          dtm.year = StepMenu(dtm.year, 0, 99, down);
        }
        else if (blinkPos == 5) {
          dtm.hour = StepMenu(dtm.hour, 0, 23, down);
        }
        else if (blinkPos == 6) {
          dtm.minute = StepMenu(dtm.minute, 0, 59, down);
        }
        else if (blinkPos == 7) {
          dtm.second = StepMenu(dtm.second, 0, 59, down);
        }

        SetDateTime(dtm);
        CalculateNextWatering();
      }
  
      else if (activeScreen == 2) { // SCREEN SCHEDULE
        bool down = (btn2 == 0);

        if (blinkPos == 1) {
          wateringMode = StepMenu(wateringMode, 0, 4, down);
          CalculateNextWatering();
        }
        else if (blinkPos == 2) {
          if (wateringMode==4)
          {
            almInterval = StepMenu(almInterval, 0, 9, down);
          }
          else
          {
            exact1h = StepMenu(exact1h, 0, 24, down);
          }
        }
        else if (blinkPos == 3) {
          exact1m = StepMenu(exact1m, 0, 59, down);
        }
        else if (blinkPos == 4) {
          if (wateringMode == 4)
          {
              intervalStartH = StepMenu(intervalStartH, 0, 23, down);
          }
          else
          {
            exact2h = StepMenu(exact2h, 0, 24, down);
          }
        }
        else if (blinkPos == 5) {
          if (wateringMode == 4)
          {
              intervalStartM = StepMenu(intervalStartM, 0, 59, down);
          }
          else
          {
            exact2m = StepMenu(exact2m, 0, 59, down);
          }
        }        
        else if (blinkPos == 6) {
              intervalEndH = StepMenu(intervalEndH, 0, 24, down);
        }
        else if (blinkPos == 7) {
              intervalEndM = StepMenu(intervalEndM, 0, 59, down);
        }
        else if (blinkPos == 8) {
          wateringDuration = StepMenu(wateringDuration, 0, 20, down);
        }
        else if (blinkPos == 9) {
          adjWater = StepMenu(adjWater, 0, 8, down);
          CalculateNextWatering();
        }
        else if (blinkPos == 10) {
          skpDays = StepMenu(skpDays, 0, 7, down);
        }
        else if (blinkPos == 11) {
          d2dMode = StepMenu(d2dMode, 0, 3, down);
          CalculateNextWatering();
        }
      }

      else if (activeScreen == 3) { // SCREEN GEOLOCATION
        bool down = (btn2 == 0);

        if (blinkPos == 1) {
          timezone = StepMenu(timezone, -48, 56, down);
        }
        else if (blinkPos == 2) {
          daySav = StepMenu(daySav, DST_DISABLED, DST_NEW_ZEALAND, down);
        }
        else if (blinkPos == 3) {
          latitude = StepMenu(latitude, -90, 90, down);
        }
        else if (blinkPos == 4) {
          longitude = StepMenu(longitude, -180, 180, down);
        }
      }

      else if (activeScreen == 4) { // SCREEN OPTIONS
        bool down = (btn2 == 0);

        if (blinkPos == 1) {
          sensorType = StepMenu(sensorType, 0, 2, down);
        }
        else if (blinkPos == 2) {
          tempUnit = !tempUnit;
        }
        else if (blinkPos == 3) {
          heatWave = StepMenu(heatWave, 0, 26, down);
        }
        else if (blinkPos == 4) {
          heatMultiplier = StepMenu(heatMultiplier, 1, 9, down);
        }
        else if (blinkPos == 5) {
          skpHumidity = StepMenu(skpHumidity, 0, 99, down);
        }
        else if (blinkPos == 6) {
          clockCalibration = StepMenu(clockCalibration, -240, 240, down);
        }
        else if (blinkPos == 7) {
          screenSaver = StepMenu(screenSaver, 1, 60, down);
        }
      }

      else if (activeScreen == 8) { // MANUAL WATERING
        bool down = (btn2 == 0);

        if (blinkPos == 1) {
          manualWateringDuration = StepMenu(manualWateringDuration, 0, 20, down);
        }
      }
      blinkStatus = true;
    }
  }

  if (btn2 == 0) {
    btn2 = 2;
  }
  else if (btn2 == 2 && !btn2Keep) {
    btn2 = 1;
  }
  if (btn3 == 0) {
    btn3 = 2;
  }
  else if (btn3 == 2 && !btn3Keep) {
    btn3 = 1;
  }
}


void MonitorButtons() {
  bool btnValue;
  byte by;
  // Keep track of the buttons using global variables btn1, btn2, btn3.
  // A button can have one of these statuses: 0=pressed, 1=not pressed, 2=press process completed, turning to 1.
  btnValue = digitalRead(PIN_BTN1);
  if (btnValue != lastBtn1Status) { // every time the button status changes, restarts the debounce process
    lastPin1Debounce = millis();
  }
  if ((millis() - lastPin1Debounce) > DEBOUNCE_DELAY) { // Debounce button
    if (btnValue != btn1Status) {
      btn1Status = btnValue; // track the status of the button 1
    }
  }
  lastBtn1Status = btnValue;
  btnValue = digitalRead(PIN_BTN2);
  if (btnValue != lastBtn2Status) {
    lastPin2Debounce = millis();
  }
  if ((millis() - lastPin2Debounce) > DEBOUNCE_DELAY) {
    if (btnValue != btn2Status) {
      btn2Status = btnValue;
    }
  }
  lastBtn2Status = btnValue;
  btnValue = digitalRead(PIN_BTN3);
  if (btnValue != lastBtn3Status) {
    lastPin3Debounce = millis();
  }
  if ((millis() - lastPin3Debounce) > DEBOUNCE_DELAY) {
    if (btnValue != btn3Status) {
      btn3Status = btnValue;
    }
  }
  lastBtn3Status = btnValue;
  //to enter the edit mode, the blinkstatus must have visible status, and it is not edit mode, and the screen is not read-only, and button 1 is pressed for more than 1.4 seconds
  if ((blinkStatus && !btn1Keep && blinkPosCt[activeScreen - 1] != 0 && pressCt > (1400 / EDIT_FREQ) && btn1Status == 0) || (editMode && (idleCt > (5600 / EDIT_FREQ))) ) {
    if (AlmTrig == 0) { // also enter the edit mode only if no watering is triggered
      editMode = !editMode; // switch mode
      blinkPos = editMode == true ? blinkPosCt[activeScreen - 1] : 0; // when entering the edit mode, set the blinking pointer to the last input position of the screen, so edit can start from the first input position after button is processed.
      // SAVE DATA ////////////////////////////////////////////////////////
      if (!editMode) { // if exiting the edit mode
        bool backupOptions = activeScreen >= 2 && activeScreen <= 4;
        if (activeScreen == 1 || activeScreen == 2) {
          CalculateNextWatering();
        }
        else if (activeScreen == 3) {// Screen Geo Location
          int mm1, mm2;
          float tmz = timezone;
          Dusk2Dawn D2D(latitude, longitude, tmz / 4);
          by = 1;
          maxDaylightMinutes = 0;
          minDaylightMinutes = 1440;
          while ( by < 13 )
          {
            mm1  = D2D.sunrise(dtm.year + 2000, by, 21, false);
            mm2  = D2D.sunset(dtm.year + 2000, by, 21, false);
            if ((mm1 > 1440) || (mm2 > 1440)) {
              mm1 = 0;
              mm2 = 1440;
            }
            if ((mm2 - mm1) < minDaylightMinutes) {
              minDaylightMinutes = (mm2 - mm1);
            }
            if ((mm2 - mm1) > maxDaylightMinutes) {
              maxDaylightMinutes = (mm2 - mm1);
            }
            by++;
          }
          CalculateNextWatering();
        }
        else if (activeScreen == 4) {// Clock calibration
          getTempSensorAddress();
          CreateCalibrationMatrix();
        }
        else if (activeScreen == 8 && idleCt < (5600 / EDIT_FREQ)) {// Watering now
          AlmTrig = 3;
          curAlmDuration = millis(); // this global variable is used to keep track of the seconds elapsed since the last watering was triggered
          AddLogEvent(LOG_MANUAL, WateringDuration(AlmTrig) / 1000); // log manual watering without updating last automatic watering
          blinkPos = 0; // a screen can have zero, one or more edit cells. if the blinking position is zero no cell is blinking
          editMode = false; // start in view mode
          Screen.clear();
          activeScreen = 1; // current screen for view/edit mode
          digitalWrite(PIN_OUT, HIGH);
        }
        SaveSettings(backupOptions);
      } // end of save data
    }
    else {
      if (activeScreen == 1) { // If the watering is triggered and the user (only in the time screen) press the button 1 for more than 2 seconds, switch-off the watering.
        manualStop = true;
      } else { // manual stop of the watering is allowed only in the main screen
        Screen.clear();
        printMsg(20); //TURN OFF
        printMsg(21); //WATERING
        printMsg(22); //FROM TIME SCREEN
        delay(3000);
        Screen.clear();
      }
    }
    btn1Keep = true; // Track the status "button pressed for more than 1.4 seconds"
  }
  if (btn1Keep && btn1Status == 1) { // If the button 1 is not pressed restart the timer for entering the edit mode
    btn1Keep = false;
    pressCt = 0; // restart the timer for entering the edit mode.
  }
  if (editMode == true && btn1Status == 0 && btn1 == 1) { // If in edit mode and the button 1 is pressed (and it was not pressed)
    btn1 = 0; // save the status (pressed) of the button, it will be processed by the method: ProcessButtons
    sleepCt = 0; // restart the timer for entering the sleep mode.
  }
  if (btn1Status == 1 && btn1 == 2) { // If the button 1 is not pressed and the previous button 1 process has been completed
    btn1 = 1; // save the status (not pressed) of the button
    sleepCt = 0;
  }
  if (btn2Keep && pressCt > (800 / EDIT_FREQ) && btn2Status == 0 ) { // when in edit mode the button 2 is kept pressed for more than 0.8 seconds, the value is increased/decreased automatically.
    btn2Keep = false; // this is done mocking up the release of the button
  }
  if (!btn2Keep && btn2Status == 1) { // if the button 2 is not pressed we reset also the keep status to not pressed
    btn2Keep = true;
  }
  if (btn3Keep && pressCt > (800 / EDIT_FREQ) && btn3Status == 0 ) { // when in edit mode the button 3 is kept pressed for more than 0.8 seconds, the value is increased/decreased automatically.
    btn3Keep = false; // this is done mocking up the release of the button
  }
  if (!btn3Keep  && btn3Status == 1) { // if the button 3 is not pressed we reset also the keep status to not pressed
    btn3Keep = true;
  }
  if (!btn1Keep) {
    if (btn2Status == 0 && btn2 == 1) { // copy the button 2 status (pressed) to a global variable
      btn2 = 0;
      sleepCt = 0;
    }
    if (btn2Status == 1 && btn2 == 2) { // copy the button 2 status (released) to a global variable
      btn2 = 1;
      sleepCt = 0;
    }
    if (btn3Status == 0 && btn3 == 1) { // copy the button 3 status (pressed) to a global variable
      btn3 = 0;
      sleepCt = 0;
    }
    if (btn3Status == 1 && btn3 == 2) { // copy the button 3 status (released) to a global variable
      btn3 = 1;
      sleepCt = 0;
    }
  }
}



void PrintScreen(byte scr) {
  unsigned long mss;
  bool isHumPresent = (sensorErr == 0 && sensorType==2 && extSensorIdx != intSensorIdx && soilSensorValueRaw > 0);
  bool blkStatus = (blinkStatus == true && btn2Keep && btn3Keep);
  Screen.setFont(lcd5x7m); 
  Screen.setLetterSpacing(1); 
  Screen.set2X();
  const char * tUnit = tempUnit ? "$" : "@";
  int integerPart, decimalPart, integerPart2, decimalPart2;    
  unsigned long scaled;
  int xtmp;

  if (scr == 1) { // MAIN SCREEN  ///////////////////////////////////////////////////////////////////////////////////////////////////
    if (AlmTrig == 1 || AlmTrig == 2 || AlmTrig == 3) // watering is ON
    {
      PrintShort(0, 0, false, F("&^"), blinkStatus == true  ? ""  : " ]]WATERING");
      mss = WateringDuration(AlmTrig) - (millis() - curAlmDuration) + 1000;
      Screen.set1X();
      Print(0, 6, false, F("  ]TIME LEFT=]&^"), msToChar(cstr1, mss), "" , "", "", "");
      Screen.clear(0, 127, 7, 7);     
    }
    else
    {
      byte dow=dtm_ds.dow;
      if (isSummerTime && dtm_ds.hour==0) //during summertime the dayOfWeek is changed 1 hour later on the DS1307
      {
        dow=dow+1;
        if (dow==8)
        {
          dow=1;
        }
      }
      //date
      Screen.set1X();
      Print( 0, 1, false, dtm_ds.day < 10 ? F("    [&]&]&]20&^") : F("   ]&]&]&]20&^"), (blkStatus == true && blinkPos == 1 ? "   "  : weekDays[dow-1]),
            (blkStatus == true && blinkPos == 2 ? (dtm_ds.day < 10 ? " " : "  ")  : lngToChar(cstr1, dtm_ds.day, false)),
            (blkStatus == true && blinkPos == 3 ? "   "  : months[dtm_ds.month - 1]), (blkStatus == true && blinkPos == 4 ? "  "  : lngToChar(cstr2, dtm_ds.year, true)), "");
      //BATTERY
      byte batteryPercent = ((int)batteryLevel * 100) / 14;
      byte tick = ((int)batteryLevel * 12) / 14;

      cstr5[0] = '"';
      for (byte i = 0; i < 12; i++) {
        cstr5[i + 1] = (i < tick) ? '`' : '_';
      }
      cstr5[13] = ';';
      cstr5[14] = 0;
      Print(0, 6, false, F(" BAT]&%&^"), lngToChar(cstr1, batteryPercent, false), cstr5, "", "", "");
      //last watering
      if (!isDefaultDateTime(wateringLog[0]))
      {
        mss=((wateringLog[0].durationHigh*256)+wateringLog[0].durationLow);
        mss=mss*1000;      
        Print(0, 7, false, wateringLog[0].day < 10 ? F("][*]&[[&]&:&](&)") : F("[[*]&[[&]&:&](&)"), lngToChar(cstr1, wateringLog[0].day, false), months[wateringLog[0].month - 1], lngToChar(cstr2, wateringLog[0].hour, true), lngToChar(cstr3, wateringLog[0].minute, true), msToChar(cstr4, mss));
      }
      
    }


    //temp & moist
    xtmp=Cel2Fah(temperatureSensorValue);
    integerPart = xtmp / 10;
    decimalPart = xtmp % 10;
    scaled = Moist(soilSensorValue);
    integerPart2 = scaled / 10;    
    decimalPart2 = scaled % 10; 
    Print(0, 4, false, F(" [[[TMP]&.&& MST]&.&%^"), sensorErr < 1 ? lngToChar(cstr1, integerPart, false) : "--", sensorErr < 1 ? lngToChar(cstr2, decimalPart, false) : "-", tUnit, isHumPresent ? lngToChar(cstr3, integerPart2, false) : "--", isHumPresent ? lngToChar(cstr4, decimalPart2, false) : "-");

    //low rtc level
    if (batteryLevel < 3){
        PrintShort(0, 5, false, F("    &^"), blinkStatus == true && AlmTrig == 0 ? "RTC BAT LOW!!" : "             ");
    }

    //time
    Screen.setFont(lucida10x14);
    Screen.setLetterSpacing(2);
    Print(12, 2, false, F("&:&:&^"), (blkStatus == true && blinkPos == 5 ? "//"  : lngToChar(cstr1, dtm_ds.hour, true)),
          (blkStatus == true && blinkPos == 6 ? "//"  : lngToChar(cstr2, dtm_ds.minute, true)),
          (blkStatus == true && blinkPos == 7 ? "//"  : lngToChar(cstr3, dtm_ds.second, true)), "", "");
  }


  else if (scr == 2) { // SCHEDULE ///////////////////////////////////////////////////////////////////////////////////////////////////
    printMsg(27); //SCHEDULE^
    PrintUnderline();
    PrintShort(0, 2, false, F("MODE= &^"), blkStatus == true && blinkPos == 1 ? ""  : wateringModes[wateringMode]);
    
    byte h1, m1, h2, m2;
    h1= wateringMode==1 ? exact1h : alm1Time.hour;
    m1= wateringMode==1 ? exact1m : alm1Time.minute;
    h2= wateringMode==1 ? exact2h : alm2Time.hour;
    m2= wateringMode==1 ? exact2m : alm2Time.minute;
    bool isIntervalMode = wateringMode == 4;
    if (wateringMode != 0)
    {
      if (isIntervalMode) 
      {

        Print(0, 3, false, F("EVERY=]&]&^"), 
          (blkStatus == true && blinkPos == 2) ? ((almInterval == 0 || almInterval == 4 || almInterval == 5 || almInterval == 6 || almInterval == 7) ? " " : "  ") : lngToChar(cstr1, wateringIntervals[almInterval] < 60 ? wateringIntervals[almInterval] : wateringIntervals[almInterval]/60, false), 
          (blkStatus == true && blinkPos == 2) ? "   " : (wateringIntervals[almInterval] < 60 ? "MIN" : "HRS"), "", "", "");
        Print(0, 4, false, intervalEndH ==  24 ? ( (blkStatus == true && blinkPos == 6) ? F("BEG=]&:& END=^") : F("BEG=]&:& END=]OFF^")) : F("BEG=]&:& END=]&:&^"), 
          (blkStatus == true && blinkPos == 4) ? "  " :  lngToChar(cstr1, intervalStartH, true), 
          (blkStatus == true && blinkPos == 5) ? "  " :  lngToChar(cstr2, intervalStartM, true), 
          (blkStatus == true && blinkPos == 6) ? "  " :  lngToChar(cstr3, intervalEndH, true), 
          (blkStatus == true && blinkPos == 7) ? "  " :  lngToChar(cstr4, intervalEndM, true), 
          "");
      }
      else
      {
        Print(0, 3, false, ((h1 == 24 && h2 == 24) || wateringMode==0) ? F("WT1= && WT2= &&^") : ((h1 == 24 && h2 != 24) ? F("WT1= && WT2= &:&^") : ((h1 != 24 && h2 == 24) ? F("WT1= &:& WT2= &&^") : F("WT1= &:& WT2= &:&^"))), 
                              ((blkStatus == true && blinkPos == 2) ? "  "  : ((h1 == 24 || wateringMode==0) ? "OF" : lngToChar(cstr1, h1, true))), 
                              ((blkStatus == true && (blinkPos == 3 || (blinkPos == 2 && h1 == 24))) ? "  "  : ((h1 == 24 || wateringMode==0) ? "F " : lngToChar(cstr2, m1, true))), 
                              ((blkStatus == true && blinkPos == 4) ? "  "  : ((h2 == 24 || wateringMode==0) ? "OF" : lngToChar(cstr3, h2, true))), 
                              ((blkStatus == true && (blinkPos == 5 || (blinkPos == 4 && h2 == 24))) ? "  "  : ((h2 == 24 || wateringMode==0) ? "F " : lngToChar(cstr4, m2, true))), 
                              "");
      }

    
      PrintShort(0, 4+isIntervalMode, false, F("DURATION= &^"), (blinkStatus == true && blinkPos == 8 && btn2Keep && btn3Keep ? ""  : getAlmDurOpt(wateringDuration, cstr1)));
      PrintShort(0, 5+isIntervalMode, false, (blkStatus == true && blinkPos == 9) ? F("SEASON COMP= ^") : (wateringMode == 2 ? F("SEASON COMP= AUTO^") : (adjWater == 0 ? F("SEASON COMP= OFF^") : F("SEASON COMP= +&%^"))), lngToChar(cstr1, adjWater*50, false));
    
    
      if (!isIntervalMode)
      {
        PrintShort(0, 6, false, F("SKIP WT DAYS= &^"), (blkStatus == true && blinkPos == 10) ? "" : ((skpDays == 1 || wateringMode == 2) ? "AUTO" : (skpDays == 0 ? "OFF" : lngToChar(cstr1, skpDays - 1, false))));
      }
      
      if (wateringMode == 2 || wateringMode == 3) //D2D
      {
          PrintShort(0, 7, false, F("D2D= &^"), blkStatus == true && blinkPos == 11 ? ""  : (wateringMode == 2 ? "AUTO": d2dModes[d2dMode]));
      }
      else
      {
        Screen.clear(0, 127, 7, 7);     
      }
    }
    else
    {
      Screen.clear(0, 127, 3, 7);      
    }
  }

  else if (scr == 3) { // GEOLOCATION ///////////////////////////////////////////////////////////////////////////////////////////////////
    printMsg(28); //LOCATION^
    PrintUnderline();
    int r = abs(timezone % 4);
    Print(0, 2, false, (blkStatus == true && blinkPos == 1) ? F("TIMEZONE= ^") : F("TIMEZONE= &&&&^"),
          (timezone > -1 ? "+" : (timezone / 4 == 0 ? "-" : "")),
          lngToChar(cstr1, (timezone / 4), false),
          (r == 0 ? "" : (r == 1 ? ":15" : (r == 2 ? ":30" : ":45"))),
          "", "");
    PrintShort(0, 3, false, F("DAYLIGHT SAVING= &^"), (blkStatus == true && blinkPos == 2 ? ""  : dstModes[daySav]));
    Print(0, 4, false, (blkStatus == true && blinkPos == 3) ? F("LATITUDE= ^") : F("LATITUDE= &&^"), (latitude > -1 ? "+" : ""),
          lngToChar(cstr1, latitude, false),
          (latitude > -10 && latitude < 10) ? " " : "" , "", "");
    Print(0, 5, false, (blkStatus == true && blinkPos == 4) ? F("LONGITUDE= ^") : F("LONGITUDE= &&^"), (longitude > -1 ? "+" : ""),
          lngToChar(cstr1, longitude, false), "", "", "");
    //Removing the link below, from all the application screens, violates the license.
	printMsg(36); //www.picapot.com
		  
  }  
  
  else if (scr == 4) { // OPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////
    printMsg(29); //OPTIONS^
    PrintUnderline();
    PrintShort(0, 2, false, F("SENSOR= &^"), (blkStatus == true && blinkPos == 1 ? ""  : sensorTypes[sensorType]));
    PrintShort(0, 3, false, F("TEMP UNIT= &^"), (blkStatus == true && blinkPos == 2 ? ""  : (tempUnit ? "FAHRENHEIT" : "CELSIUS")));
    int hb=Cel2Fah(heatWave*10 + 290)/10;
    Print(0, 4, false, heatWave==0 ? F("HEAT BOOST= &&^") : F("HEAT BOOST= && +&&^"), (blkStatus == true && blinkPos == 3 ? (hb>99 ? "   " : "  ")  : (heatWave == 0 ? "OFF" : lngToChar(cstr1, hb, false))),
          (blkStatus == true && blinkPos == 3 ? " "  : (heatWave == 0 ? " " : tUnit)), 
          (blkStatus == true && blinkPos == 4 ? "  "  : lngToChar(cstr2, heatMultiplier*10, false)), 
           (blkStatus == true && blinkPos == 4 ? "  "  : "%"), "");
    PrintShort(0, 5, false, (blkStatus == true && blinkPos == 5) ? F("MOISTURE LIMIT= ^") : (skpHumidity == 0 ? F("MOISTURE LIMIT= OFF^") : F("MOISTURE LIMIT= &%^")), 
          lngToChar(cstr1, skpHumidity, false));
    Print(0, 6, false, (blkStatus == true && blinkPos == 6) ? F("CLOCK CALIB= ^") : F("CLOCK CALIB= &&]SEC^"), (clockCalibration >= 0 ? "+" : ""),
          lngToChar(cstr1, clockCalibration, false), "", "", "");
    Print(0, 7, false, F("SCREEN TMOUT= &]&^"), (blkStatus == true && blinkPos == 7) ? "" : lngToChar(cstr1, screenSaver, false), (blkStatus == true && blinkPos == 7) ? "" : "MIN", "", "", "");

      
  }

  else if (scr == 5) { // EVENTS ///////////////////////////////////////////////////////////////////////////////////////////////////
    printMsg(30); //EVENTS^
    PrintUnderline();
    if (isDefaultDateTime(wateringLog[1]))
    {
      PrintShort(0, 2, false, F("NO EVENTS YET"), "");
    }
    else
    {
      for (byte i = 1; i <= 6; i++)
      {
        if (!isDefaultDateTime(wateringLog[i]))
        {
          byte eventType = wateringLog[i].hour >> 5;
          bool manualEvent = eventType == LOG_MANUAL;
          unsigned int eventValue = (wateringLog[i].durationHigh*256)+wateringLog[i].durationLow;
          const char *eventDetail;
          if (eventType == LOG_WATERING || manualEvent) {
            mss=(unsigned long)eventValue*1000;
            if (manualEvent) {
              cstr4[0] = 'M';
              cstr4[1] = '-';
              msToChar(cstr4 + 2, mss);
              eventDetail = cstr4;
            }
            else {
              eventDetail = msToChar(cstr4, mss);
            }
          }
          else {
            cstr4[0] = 'S';
            cstr4[1] = 'K';
            cstr4[2] = 'I';
            cstr4[3] = 'P';
            cstr4[4] = '-';
            cstr4[5] = pgm_read_byte(&logEventCodes[eventType]);
            cstr4[6] = 0;
            eventDetail = cstr4;
          }

          Print(0, 1+i, false, F("&]&]&:&](&)^"), lngToChar(cstr1, wateringLog[i].day, false), months[wateringLog[i].month - 1], lngToChar(cstr2, wateringLog[i].hour & 0x1F, true), lngToChar(cstr3, wateringLog[i].minute, true), eventDetail);
        }
      }
    }
  }

#if DIAGNOSTIC == 1
  else if (scr == 6) { // DIAGNOSTIC //////////////////////////////////////////////////////////////////////////////////////////////////
    printMsg(31); //DIAGNOSTIC^
    PrintUnderline();
    Print(0, 2, false, F("V=& B=&^"), lngToChar(cstr1, InternalVoltage, false), lngToChar(cstr2, getBatteryMillivolts(), false), "", "", "");
    int effDaysToSkip;
    bool autoSkipDays = wateringMode == 2 || skpDays == 1;
    bool isMidSeason = adjWaterResult >= SKIPDAYS_EARLY_SPRING_PERC && adjWaterResult < SKIPDAYS_LATE_SPRING_PERC;
    bool validTodayMaxTemp = sensorErr == 0 && dateDiff(maxTempDateTime, dtm_ds) == 0;
    bool skipWt2ForTemperature = isMidSeason && validTodayMaxTemp && maxTempValue < (DOUBLE_WATERING_TEMP * 10);
    bool autoSkipSecondWatering = autoSkipDays && (adjWaterResult < SKIPDAYS_EARLY_SPRING_PERC || skipWt2ForTemperature);
    if (autoSkipDays) {
        effDaysToSkip = (adjWaterResult < SKIPDAYS_WINTER_PERC) ? 3 :
                      (adjWaterResult < SKIPDAYS_EARLY_SPRING_PERC) ? 2 :
                      (adjWaterResult < SKIPDAYS_LATE_SPRING_PERC) ? 0 : 0;
    }
    else
    {
      effDaysToSkip = skpDays;    
    }
    byte wt = 0;
    if (wateringMode == 1)
    {
      wt = (alm1Time.hour != 24) + ((alm2Time.hour != 24) << 1);
      if (wt == 3 && autoSkipSecondWatering) wt = 1;
    }
    else if (wateringMode == 2)
    {
      wt = autoSkipSecondWatering ? 1 : 3;
    }
    else if (wateringMode == 3)
    {
      wt = autoSkipSecondWatering ? 1 : 3;
    }
    Print(0, 3, false, F("SK=& WT=& SE=&%^"), lngToChar(cstr1, effDaysToSkip > 1 ? (effDaysToSkip - 1) : 0, false), wt==0 ? "NONE" : (wt==1 ? "W1" : (wt==2 ? "W2" : "W1+W2")), lngToChar(cstr2, adjWaterResult, false), "", "");
    Print(0, 4, false, F("W1=&:& W2=&:&^"), lngToChar(cstr1, alm1Time.hour, true), lngToChar(cstr2, alm1Time.minute, true), lngToChar(cstr3, alm2Time.hour, true), lngToChar(cstr4, alm2Time.minute, true), "");
    Print(0, 5, false, F("IN=& EX=& ER=& CT=&^"), lngToChar(cstr1, intSensorIdx, false), lngToChar(cstr2, extSensorIdx, false), lngToChar(cstr3, sensorErr, false), lngToChar(cstr4, sensorCount, false), "");     
    Print(0, 6, false, F("I=& 0=& 1=&^"), addrToHex(cstr1, internalSensorAddress), addrToHex(cstr2, temperatureSensorAddress[0]), addrToHex(cstr3, temperatureSensorAddress[1]), "", "");
    
  }
#endif

  else if (scr == 7) { // SENSOR ///////////////////////////////////////////////////////////////////////////////////////////////////
    printMsg(32); //SENSOR^
    PrintUnderline();

    xtmp=Cel2Fah(temperatureSensorValue);
    integerPart = xtmp / 10;
    decimalPart = xtmp % 10;    
    Print(0, 2, false, F("TEMPERATURE:&.&&^"), sensorErr == 0 ? lngToChar(cstr1, integerPart, false) : "--", sensorErr == 0 ? lngToChar(cstr2, decimalPart, false) : "-", tUnit, "", "");
    xtmp=Cel2Fah(maxTempValue);
    integerPart = xtmp / 10;
    decimalPart = xtmp % 10;    
    Print(0, 3, false, F("]']&.&&]&:&^"), lngToChar(cstr1, integerPart, false), lngToChar(cstr2, decimalPart, false), tUnit, lngToChar(cstr3, maxTempDateTime.hour, true), lngToChar(cstr4, maxTempDateTime.minute, true));
    xtmp=Cel2Fah(minTempValue);
    integerPart = xtmp / 10;
    decimalPart = xtmp % 10;    
    Print(0, 4, false, F("],]&.&&]&:&^"), lngToChar(cstr1, integerPart, false), lngToChar(cstr2, decimalPart, false), tUnit, lngToChar(cstr3, minTempDateTime.hour, true), lngToChar(cstr4, minTempDateTime.minute, true));

    scaled = Moist(soilSensorValue); 
    integerPart = scaled / 10;    
    decimalPart = scaled % 10; 
    Print(0, 5, false, F("MOISTURE:&.&%](&)^"), isHumPresent ? lngToChar(cstr1, integerPart, false) : "--", isHumPresent ? lngToChar(cstr2, decimalPart, false) : "-", lngToChar(cstr3, soilSensorValueRaw, false), "", "");
    if (isHumPresent)
    {
      scaled = Moist(maxHumValue); 
      integerPart = scaled / 10;    
      decimalPart = scaled % 10;     
      Print(0, 6, false, F("]']&.&%]&:&^"), lngToChar(cstr1, integerPart, false), lngToChar(cstr2, decimalPart, false), lngToChar(cstr3, maxHumDateTime.hour, true), lngToChar(cstr4, maxHumDateTime.minute, true), "");
      scaled = Moist(minHumValue); 
      integerPart = scaled / 10;    
      decimalPart = scaled % 10; 
      Print(0, 7, false, F("],]&.&%]&:&^"), lngToChar(cstr1, integerPart, false), lngToChar(cstr2, decimalPart, false), lngToChar(cstr3, minHumDateTime.hour, true), lngToChar(cstr4, minHumDateTime.minute, true), "");
    }
    else
    {
      Screen.clear(0, 127, 6, 7);
    }
  }



  else if (scr == 8) { // MANUAL WATERING ///////////////////////////////////////////////////////////////////////////////////////////////////
    printMsg(33); //MANUAL^
    PrintUnderline();
    PrintShort(0, 2, false, F("DURATION= &^"), (blkStatus == true && blinkPos == 1 ? ""  : getAlmDurOpt(manualWateringDuration, cstr1)));
    printMsg(34); //Select]duration^
    printMsg(35); //Hold]LEFT]button^
	
  }
}





//   <!------------------------------------------------->
//   <!--    Clock Calibration by Fabrizio Ranieri    -->
//   <!--    OCL v1.1 + GAtt v1 applies              -->
//   <!------------------------------------------------->
// Clock Calibration is based on a 240 bit array. Each bit represents a period of 6 minutes (6x240 = 1440 = 24 hours) and every 6 minutes the corresponding bit is checked.
// When the bit is high represents 1 second of error to be adjusted accordingly with the sign of the setting [clockCalibration](-240 +240) (-=clock moves behind +=clock moves ahead).
// The array is created from the value contained in the two bytes of the setting [clockCalibration], equally distributing the time corrections throughout the day. 
// The clock can be calibrated for an error of maximum 240 seconds in 24 hours.
// Only [clockCalibration] value (2 bytes) is stored to the battery backed memory, and the array (30 bytes) is created at every reboot or [clockCalibration] setting change.
void CreateCalibrationMatrix() {
  int clockCal, ct = 0;
  clockCal = abs(clockCalibration);
  bool standardProcess = (clockCal <= 120); // if high bit are the majority, invert the logic for a better distribution
  int timeFrame = standardProcess ? (240 / (clockCal == 0 ? 1 : clockCal)) : (240 / ((240 - clockCal) == 0 ? 1 : (240 - clockCal)));
  for (byte i = 0; i < 30; i++)
  {
    calibration[i] = standardProcess ? (byte)0x00 : (byte)0xFF;
  }
  for (byte i = 0; i < 240; i++)
  {
    if (ct < (standardProcess ? clockCal : (240 - clockCal)))
    {
      if ((i % timeFrame) == 0)
      {
        ct = ct + 1;
        WriteCalibrationBit(i, standardProcess);
      }
    }
  }
}

bool ReadCalibrationBit(byte pos) {
  byte ps = pos / 8;
  return (calibration[ps] &  (1 << (7 - (pos % 8)))) == (1 << (7 - (pos % 8)));
}

void WriteCalibrationBit(byte pos, bool val) {
  byte ps = pos / 8;
  calibration[ps] =  val ? calibration[ps] | (1 << (7 - (pos % 8))) : calibration[ps] & (255 - (1 << (7 - (pos % 8)))) ;
}

/*
String Byte2BinString(byte b) { // used to monitor the calibration array
  String s = "";
  for (int i = 7; i >= 0; i-- )
  {
    s = s + String((b >> i) & 0X01); //shift and select first bit
  }
  return s;
}
*/
// end of clock calibration

unsigned int Moist(unsigned int i) {
  unsigned int range = SOIL_SENSOR_MAX - SOIL_SENSOR_MIN;
  if (range == 0) {
    return 0;
  }
  if (i > range) {
    i = range;
  }
  i = (unsigned long)i * 1000 / range;
  if (SOIL_SENSOR_INVERTED) {
    i = 1000 - i;
  }
  return i;
}



int readVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)); //first sample is discarded
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA,ADSC));  
  return 1126400L / ADC;  
}



int getBatteryMillivolts() {
  unsigned long f = analogRead(PIN_BAT_CHECK); //first sample is discarded
  delay(1);
  f = analogRead(PIN_BAT_CHECK);
  f = f * InternalVoltage;
  return int(f / 1024);
}



int getBatteryLevel() {
  long sum = 0;

  for (byte i = 0; i < 10; i++) {
    sum += getBatteryMillivolts();
  }

  int mv = sum / 10;

  if (mv < 2500) return 0;
  else if (mv < 2600) return 1;
  else if (mv < 2700) return 2;
  else if (mv < 2750) return 3;
  else if (mv < 2800) return 4;
  else if (mv < 2840) return 5;
  else if (mv < 2880) return 6;
  else if (mv < 2910) return 7;
  else if (mv < 2940) return 8;
  else if (mv < 2960) return 9;
  else if (mv < 2980) return 10;
  else if (mv < 3000) return 11;
  else if (mv < 3030) return 12;
  else if (mv < 3070) return 13;
  else return 14;
}

char* addrToHex(char *buf, const byte *addr) {
  const char hex[] = "0123456789ABCDEF";

  buf[0] = hex[(addr[6] >> 4) & 0x0F];
  buf[1] = hex[addr[6] & 0x0F];
  buf[2] = hex[(addr[7] >> 4) & 0x0F];
  buf[3] = hex[addr[7] & 0x0F];

  buf[4] = 0;
  return buf;
}

void getTempSensorAddress() {
  sensorCount = 0;
  oneWire.reset_search();
  oneWire.target_search(0x28);
  memset(temperatureSensorAddress, 0, sizeof(temperatureSensorAddress));
  while (sensorCount < 2) {
    if (oneWire.search(temperatureSensorAddress[sensorCount])) {
      sensorCount++;
    } else {
      memset(temperatureSensorAddress[sensorCount], 0, 8);
      break;
    }
  }
  intSensorIdx = 0;
  extSensorIdx = 1;
  for (int s = 0; s < sensorCount; s++) {
    bool match = true;

    for (int i = 0; i < 8; i++) {
      if (temperatureSensorAddress[s][i] != internalSensorAddress[i]) {
        match = false;
        break;
      }
    }

    if (match) {
      intSensorIdx = s;
      if (intSensorIdx == 0)
      {
        extSensorIdx = 1;
      }
      else
      {
        extSensorIdx = 0;
      }
      break;
    }
  }
  if (extSensorIdx==1 && sensorCount<2)
  {
    extSensorIdx=0;
  }

}


void getTemperatureAndHumidity_DS18B20(const byte address[8]) {

  int16_t r = 0;
  long i = 0;
  byte data[12];

  if (oneWire.reset()) {
    oneWire.select(address);
    oneWire.write(0xBE, 0);
    oneWire.read_bytes(data, 9);
    int c = oneWire.crc8(data, 8);
    if (c == data[8]) {
      r = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
      temperatureSensorValueRaw = ((long)r * 10) / 16;
      i = (long)data[2] * 256 + data[3];
      soilSensorValueRaw = sensorType<2 ? 0 : i;
      if ((temperatureSensorValueRaw > -410) && (temperatureSensorValueRaw < 850)) { 
        sensorErr = 0;
      }
      else
      {
        sensorErr++;
      }
    }
    else
    {
      sensorErr++;
    }
  }
  else
  {
    sensorErr++;
  }

  if ((sensorType < 2 || extSensorIdx == intSensorIdx) && oneWire.reset()) {
    oneWire.select(address);
    oneWire.write(0x44, 0);
  }

  sensorErr = sensorErr > 250 ? 1 : sensorErr;
}


// convert local variables to settings array, save it to the DS1307 memory and verify it
void SaveSettings(bool writeToEeprom) {
  byte tmp[SETTINGS_COUNT+1]; 
  memset(tmp, 0, sizeof(tmp));
  byte z = 0;
  while (z < 3) {
    Variables2Settings(tmp);
    UploadSettings(tmp);
    DownloadSettings(tmp);
    if (oneWire.crc8(tmp, SETTINGS_COUNT) == tmp[SETTINGS_COUNT]) {
      z = 4;
    }
    else {
      z++;
      delay(20);
    }
  }
  if (z == 3) { // if the settings cannot be written to the DS1307 memory, after three attempts reset the system
    printMsg(14); //ER1
    while (1) {};
  }

  //create backup of the settings, useful during battery replacement
  if (writeToEeprom)
  {
    UploadSettingsEeprom(tmp);
  }
}

// backup settings to the atmega eeprom memory
void UploadSettingsEeprom(byte* data) {
  for (byte x = 0; x < SETTINGS_COUNT; x++ ) { 
      if (EEPROM.read(100+x) != data[x])
      {
        EEPROM.write(100+x, data[x]);
      }
  }
  uint8_t crc=(uint8_t)oneWire.crc8(data, SETTINGS_COUNT); 
  if (EEPROM.read(100+SETTINGS_COUNT) != crc)
      {
        EEPROM.write(100+SETTINGS_COUNT, crc);
      }
}

// read settings backup from the atmega eeprom memory
void DownloadSettingsEeprom(byte* data) {
  for (byte x = 0; x <= SETTINGS_COUNT; x++ ) { 
    data[x]=EEPROM.read(100+x);
  }
}

// write settings array to the DS1307 memory
void UploadSettings(byte* data) {
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x08);
  for (byte x = 0; x < 31; x++ ) {
    Wire.write((uint8_t)data[x]);
  }
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(14); //ER1
    while (1) {};
  }
  else
  {
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write((uint8_t)0x27);
    for (byte x = 31; x < SETTINGS_COUNT; x++ ) {
      Wire.write((uint8_t)data[x]);
    }
    Wire.write((uint8_t)oneWire.crc8(data, SETTINGS_COUNT));
  }
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(14); //ER1
    while (1) {};
  }
}



// read settings array from the DS1307 memory
void DownloadSettings(byte* data) {
  byte x = 0;
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x08);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(15); //ER2
    while (1) {};
  }
  else
  {
    Wire.requestFrom(CLOCK_ADDRESS, 32);
    while (Wire.available()) {
      data[x] = Wire.read();
      x++;
    }
  }
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x28);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(15); //ER2
    while (1) {};
  }
  else
  {
    Wire.requestFrom(CLOCK_ADDRESS, SETTINGS_COUNT-31);
    while (Wire.available()) {
      data[x] = Wire.read();
      x++;
    }
  }
}


long SecondDiff(const dateTime& t1, const dateTime& t2) {
  return ((long)t2.hour * 3600L + t2.minute * 60L + t2.second) -
         ((long)t1.hour * 3600L + t1.minute * 60L + t1.second);
}


bool isDefaultDateTime(const dateTime& dt)
{
  return dt.year == 0;
}





void Blink(int milliseconds, int del) {
  digitalWrite(PIN_LED, HIGH);   // turn the LED ON
  delay(milliseconds);           // period with the led ON
  digitalWrite(PIN_LED, LOW);    // turn OFF the led
  delay(del);                    // pause after blink
}

uint8_t dec2bcd(uint8_t num) { // Convert Decimal to Binary Coded Decimal (BCD)
  return ((num / 10 * 16) + (num % 10));
}

uint8_t bcd2dec(uint8_t num) { // Convert Binary Coded Decimal (BCD) to Decimal
  return ((num / 16 * 10) + (num % 16));
}


dateTime GetDateTime() { // Read date and time from the clock. The values are stored in the clock registry with a binary encoded decimal format, and they must be converted to decimal
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(24); //ER3
    while (1) {}; // without clock the system cannot proceed and will reset until a working clock will be found
  }
  else
  {
    dateTime res;
    byte rdata = 0x00;
    Wire.requestFrom(CLOCK_ADDRESS, 7);
    if (Wire.available()) rdata = Wire.read();
    res.second = bcd2dec(rdata & 0x7f);
    if (Wire.available()) rdata = Wire.read();
    res.minute = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.hour = bcd2dec(rdata & 0x3f);
    if (Wire.available()) rdata = Wire.read();
    res.dow = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.day = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.month = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.year = bcd2dec(rdata);
    return res;
  }
  delay(SYSTEM_DELAY);
}

void SetDateTime(dateTime val) { // Write date and time to the clock. Values must be saved to the clock with a binary encoded decimal format.
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(25); //ER4
    while (1) {}; // without clock the system cannot work and it will reset until a clock is found
  }
  else {
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x80);
    Wire.write((uint8_t)dec2bcd(val.minute));
    Wire.write((uint8_t)dec2bcd(val.hour & 0x3f));
    Wire.write(val.dow);
    Wire.write((uint8_t)dec2bcd(val.day));
    Wire.write((uint8_t)dec2bcd(val.month));
    Wire.write((uint8_t)dec2bcd(val.year));
    Wire.endTransmission();
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)dec2bcd(val.second));
    Wire.endTransmission();
  }
  delay(SYSTEM_DELAY);
}



void ActivateClockOscillator() { //activate the clock oscillator if not yet active
  bool isActive = true;
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    printMsg(26); //ER5
    while (1) {}; // without clock the system cannot work and it will reset until a clock is found
  }
  else {
    Wire.requestFrom(CLOCK_ADDRESS, 1);
    byte rdata = Wire.read();
    if ((rdata & 0b10000000) == 0b10000000) {
      isActive = false;
    }
    if (!isActive) {
      Wire.beginTransmission(CLOCK_ADDRESS);
      Wire.write((uint8_t)0x00);
      Wire.write((uint8_t)dec2bcd(0));
      Wire.endTransmission();
    }
  }
  delay(SYSTEM_DELAY);
}








//convert milliseconds in the corresponding timespan with format hh:mm:ss
//hh is shown only if greater than zero
char * msToChar(char * str, unsigned long mss)
{
  int i = 0;
  char cpy[3] = {0, 0, 0};
  mss = mss / 1000;
  unsigned long h = mss / 3600;
  if (h > 0)
  {
    lngToChar(cpy, h, true);
    str[i++] = cpy[0];
    str[i++] = cpy[1];
    str[i++] = ':';
  }
  h = (mss % 3600) / 60;
  lngToChar(cpy, h, true);
  str[i++] = cpy[0];
  str[i++] = cpy[1];
  str[i++] = ':';
  h = (mss % 3600) % 60;
  lngToChar(cpy, h, true);
  str[i++] = cpy[0];
  str[i++] = cpy[1];
  str[i] = '\0';
  return str;
}

// https://www.geeksforgeeks.org/implement-itoa/
// modified to accept long, and to add an optional leading zero for the time digits
char * lngToChar(char * str, long num, bool leadingZero)
{
  int i = 0;
  bool isNegative = false;
  if (num < -9 || num > 9) //apply leading zero only to 1 digit numbers
  {
    leadingZero = false;
  }
  // Handle 0 explicitely, otherwise empty string is printed for 0 
  if (num == 0)
  {
    str[i++] = '0';
    if (leadingZero == true)
    {
      str[i++] = '0';
    }
    str[i] = '\0';
    return str;
  }
  // In standard itoa(), negative numbers are handled only with
  // base 10. Otherwise numbers are considered unsigned.
  if (num < 0)
  {
    isNegative = true;
    num = -num;
  }
  // Process individual digits
  while (num != 0)
  {
    int rem = num % 10;
    str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
    num = num / 10;
  }
  if (leadingZero == true)
  {
    str[i++] = '0';
  }
  // If number is negative, append '-'
  if (isNegative)
    str[i++] = '-';
  str[i] = '\0'; // Append string terminator
  // Reverse the string
  reverse(str, i);
  return str;
}


// reverse a string except for the last char
void reverse(char str[], int length)
{
  int start = 0;
  int end = length - 1;
  while (start < end)
  {
    SWAP(*(str + start), *(str + end));
    start++;
    end--;
  }
}

// convert temperature to fahreneit values are x10

int Cel2Fah(int cel)
{
  if (!tempUnit) return cel;

  return (cel * 9) / 5 + 320;
}

char* getAlmDurOpt(byte idx, char* dest) {
  strcpy_P(dest, (PGM_P)pgm_read_word(&almDurOpts[idx]));
  return dest;
}


void printMsg(byte msgId) {
  char buf[30] = {0};

  const char *p = (const char *)pgm_read_word(&(sysmsg[msgId]));
  byte row = pgm_read_byte(p++) - '0';

  byte i = 0;
  char c;
  while (i < 29 && (c = pgm_read_byte(p++)) != 0) {
    buf[i++] = c;
  }
  buf[i] = 0;

  Screen.setFont(lcd5x7m);
  Screen.setLetterSpacing(1);
  Screen.set1X();
  Print(0, row, false, F("&^"), buf, "", "", "", "");
}


void PrintUnderline()
{
  Screen.setLetterSpacing(0);
  PrintShort(0, 1, false, F("#########################^"), "");
  Screen.setLetterSpacing(1);
}

void PrintShort(int x, int y, bool inverted, const __FlashStringHelper * s00, const char * s1) {
  Print(x, y, inverted, s00, s1, "", "", "", "");
}

// Print function with formatting functionality. Every character "&" present in s0 is replaced with the corresponding value s(x). Maximum 5 replacements.
// When a parameter is numeric (integer), it must be converted using the function lngToChar. If more numeric values (max 3) are passed in the same call, each value must be converted using a different array cstr[1->3]
// Reserved characters: &=PLACEHOLDER  [=1x PIXEL SPACE  ]=4x PIXEL SPACE ^=LINEFEED(it can be used only as trailing char)
void Print(int x, int y, bool inverted, const __FlashStringHelper * s00, const char * s1, const char * s2, const char * s3, const char * s4, const char * s5) {
  char buf[33] = {0};
  byte i = 0;
  byte par = 1;

  const char *p = (const char *)s00;
  const char *ref;
  char ch;

  while (i < 32) {
    ch = pgm_read_byte(p++);
    if (ch == 0) break;

    if (ch == '&') {
      if (par == 1) ref = s1;
      else if (par == 2) ref = s2;
      else if (par == 3) ref = s3;
      else if (par == 4) ref = s4;
      else ref = s5;

      if (ref == NULL) {
        buf[i++] = '*';
      } else {
        while (i < 32 && *ref != 0) {
          buf[i++] = *ref++;
        }
      }

      par++;
    }
    else {
      buf[i++] = ch;
    }
  }

  buf[i] = 0;

  Screen.setCursor(x, y);
  Screen.setInvertMode(inverted);
  Screen.write(buf);
}

bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

long daysFromYear0(const dateTime& d) {
  long y = d.year;
  long ly = y - 1;
  long days = y * 365L + ly / 4 - ly / 100 + ly / 400;

  days += dater[d.month - 1] + d.day;

  if (d.month > 2 && isLeapYear(d.year)) days++;

  return days;
}

int dateDiff(const dateTime& d1, const dateTime& d2) {
  return (int)(daysFromYear0(d2) - daysFromYear0(d1));
}

void splitMinutes(int tmm, byte &hh, byte &mm) {
  hh = tmm / 60;
  mm = tmm % 60;
}

/*
int dateDiff(const dateTime& inDtm1, const dateTime& inDtm2) {
  const int dater[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int ref, dd1, dd2;

  ref = inDtm1.year;
  if (inDtm2.year < inDtm1.year)
    ref = inDtm2.year;

  dd1 = dater[inDtm1.month - 1] + inDtm1.day;
  if (isLeapYear(inDtm1.year) && inDtm1.month > 2) {
    dd1 += 1;
  }
  for (int i = ref; i < inDtm1.year; i++) {
    if (isLeapYear(i))
      dd1 += 1;
  }
  dd1 = dd1 + (inDtm1.year - ref) * 365;

  dd2 = dater[inDtm2.month - 1] + inDtm2.day;
  if (isLeapYear(inDtm2.year) && inDtm2.month > 2) {
    dd2 += 1;
  }
  for (int i = ref; i < inDtm2.year; i++) {
    if (isLeapYear(i))
      dd2 += 1;
  }
  dd2 = dd2 + ((inDtm2.year - ref) * 365);

  return dd2 - dd1;
}
*/

dateTime Add1Hour(dateTime inDtm) { // add one hour
  const int monthDays[12] = {31, isLeapYear(inDtm.year) ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (inDtm.hour == 23)
  {
    inDtm.hour = 0;
    if (inDtm.day == monthDays[inDtm.month - 1])
    {
      if (inDtm.month == 12)
      {
        inDtm.day = 1;
        inDtm.month = 1;
        inDtm.year++;
      }
      else
      {
        inDtm.day = 1;
        inDtm.month++;
      }
    }
    else
    {
      inDtm.day++;
    }
  }
  else
  {
    inDtm.hour++;
  }
  return inDtm;
}


dateTime Remove1Hour(dateTime inDtm) { // remove one hour
  const int monthDays[12] = {31, isLeapYear(inDtm.year) ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (inDtm.hour == 0)
  {
    inDtm.hour = 23;
    if (inDtm.day == 1)
    {
      if (inDtm.month == 1)
      {
        inDtm.day = 31;
        inDtm.month = 12;
        inDtm.year--;
      }
      else
      {
        inDtm.day = monthDays[inDtm.month - 2];
        inDtm.month--;
      }
    }
    else
    {
      inDtm.day--;
    }
  }
  else
  {
    inDtm.hour--;
  }
  return inDtm;
}


byte DstSunday(const dateTime& dt, byte sunday) {
  byte firstDow = ((dt.dow - 1) + 7 - ((dt.day - 1) % 7)) % 7;
  byte firstSunday = 1 + ((7 - firstDow) % 7);
  if (sunday > 0) return firstSunday + (sunday - 1) * 7;

  const byte monthDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  byte days = monthDays[dt.month - 1];
  if (dt.month == 2 && isLeapYear(2000 + dt.year)) days++;
  return firstSunday + ((days - firstSunday) / 7) * 7;
}

bool IsDstPeriod(const dateTime& dt, byte startMonth, byte startSunday, byte startHour,
                 byte endMonth, byte endSunday, byte endHour) {
  bool afterStart = dt.month > startMonth;
  if (dt.month == startMonth) {
    byte transitionDay = DstSunday(dt, startSunday);
    afterStart = dt.day > transitionDay || (dt.day == transitionDay && dt.hour >= startHour);
  }

  bool beforeEnd = dt.month < endMonth;
  if (dt.month == endMonth) {
    byte transitionDay = DstSunday(dt, endSunday);
    beforeEnd = dt.day < transitionDay || (dt.day == transitionDay && dt.hour < endHour);
  }

  return startMonth < endMonth ? (afterStart && beforeEnd) : (afterStart || beforeEnd);
}

bool IsSummerTime(dateTime inDtm) {
  switch (daySav) {
    case DST_EUROPE:
      // EU transitions occur at 01:00 UTC; the RTC stores local standard time.
      return IsDstPeriod(inDtm, 3, 0, 1 + timezone / 4, 10, 0, 1 + timezone / 4);
    case DST_USA:
      return IsDstPeriod(inDtm, 3, 2, 2, 11, 1, 1);
    case DST_AUSTRALIA:
      return IsDstPeriod(inDtm, 10, 1, 2, 4, 1, 2);
    case DST_NEW_ZEALAND:
      return IsDstPeriod(inDtm, 9, 0, 2, 4, 1, 2);
    default:
      return false;
  }
}

void SwitchOnScreen()
{
  Screen.ssd1306WriteCmd(SH1106_SET_PUMP_MODE);
  delay(1);
  Screen.ssd1306WriteCmd(SH1106_PUMP_ON);
  delay(1);
  Screen.ssd1306WriteCmd(SSD1306_DISPLAYON);
  screenSleep = false;
  sleepCt = 0;
  refreshNow = true;
  delay(1);
}

void resetWdt()
{
  wdt_reset();  // restart atmega328 wtd counter (first time must be done within 16ms, to avoid a premature reset before entering the registers)
}

