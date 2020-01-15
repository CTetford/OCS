/*
 * Title       Observatory-Control-System
 * by          Howard Dutton
 *
 * Copyright (C) 2012 to 2020 Howard Dutton
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * 
 *
 * Revision History, see GitHub
 *
 *
 * Author: Howard Dutton
 * http://www.stellarjourney.com
 * hjd1964@gmail.com
 *
 * Description
 *
 * General Purpose Observatory Control
 * with LX200 derived command set.
 *
 */

// firmware info
#define ONCUE_OCS
#define FirmwareDate   __DATE__
#define FirmwareNumber "2.0c"
#define FirmwareName   "OnCue OCS"
#define FirmwareTime   __TIME__

#include "Constants.h"

#include <TimeLib.h>  // from here: https://github.com/PaulStoffregen/Time
#include <SPI.h>
#include <Ethernet.h>

#include "Config.h"
#include "Validate.h"

#if NTP == ON
  #include <EthernetUdp.h>
#endif
#if WATCHDOG == ON
  #include <avr/wdt.h>
#endif
#include "string.h"
#include "EEPROM.h"
#include "TimerOne.h"
#include "WebServer.h"
#include "CmdServer.h"
#include "Command.h"

const int timeZone = TIME_ZONE;

WebServer www;
CmdServer Cmd;
CmdServer Cmd1;

#if STAT_TIME_SOURCE == NTP
  EthernetUDP Udp;
  // local port to listen for UDP packets
  unsigned int localPort = 8888;
  time_t startupTime = 0;
  bool fastNTPSync=false;
#elif STAT_TIME_SOURCE == DS3234 || STAT_TIME_SOURCE == DS3234INIT
  #include <SparkFunDS3234RTC.h>  // https://github.com/sparkfun/SparkFun_DS3234_RTC_Arduino_Library/archive/master.zip
  #define DS3234_CS_PIN 53
  time_t startupTime = 0;
#endif

// Pin assignments

typedef struct {
  uint8_t pin;
  uint8_t onState;
} relay_t;
const volatile relay_t relay[] {
  { 0, HIGH},         // not used
  {23, HIGH},         // Relay 1: pin#, ON state
  {25, HIGH},         // Relay 2: pin#, ON state
  {27, HIGH},         // Relay 3: pin#, ON state
  {29, HIGH},         // Relay 4: pin#, ON state
  {31, HIGH},         // Relay 5: pin#, ON state
  {33, HIGH},         // Relay 6: pin#, ON state
// solid state relays (PWM is enabled on these)
  {35, HIGH},         // Relay 7: pin#, ON state
  {37, HIGH},         // Relay 8: pin#, ON state
  {39, HIGH},         // Relay 9: pin#, ON state
  {41, HIGH},         // Relay 10: pin#, ON state
  {43, HIGH},         // Relay 11: pin#, ON state
  {45, HIGH},         // Relay 12: pin#, ON state
  {47, HIGH},         // Relay 13: pin#, ON state
  {49, HIGH}          // Relay 14: pin#, ON state
};
volatile uint8_t relayState[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

typedef struct {
  uint8_t pin;
  uint8_t mode;
  uint8_t onState;
} sense_t;
const volatile sense_t sense[] {
  { 0, INPUT, HIGH},  // not used
  {22, INPUT, HIGH},  // Sense 1: pin#, initialize, ON state 
  {24, INPUT, HIGH},  // Sense 2: pin#, initialize, ON state
  {26, INPUT, HIGH},  // Sense 3: pin#, initialize, ON state
  {28, INPUT, HIGH},  // Sense 4: pin#, initialize, ON state
  {30, INPUT, HIGH},  // Sense 5: pin#, initialize, ON state
  {32, INPUT, HIGH}   // Sense 6: pin#, initialize, ON state
};
volatile uint8_t senseState[] = {0,0,0,0,0,0,0};

                     // Analog 0: A0
                     // Analog 1: A1
                     // Analog 2: A2
                     // Analog 3: A3
                     // Analog 4: A4
                     // Analog 5: A5
volatile int analogState[] = {0,0,0,0,0,0};

// roof
#if ROR_AUTOCLOSE_DAWN_DEFAULT == ON
  boolean roofAutoClose = true;
#else
  boolean roofAutoClose = false;
#endif
boolean roofAutoCloseInitiated = false;

// relay control timer
unsigned long tst;

unsigned long msFiveMinuteCounter;
float insideTemperature;

void setup()   {
#ifdef NexDomeSetup
  // Get dome ready
  NexDomeSetup();
#endif

  // Initialize serial communications
  Serial.begin(9600);
  
#if WATCHDOG == ON
  wdt_enable(WDTO_8S);
#endif

  // Set pins for direct relay control
  for (int i=1; i<=14; i++) { pinMode(relay[i].pin,OUTPUT); setRelayOff(i); }

  // Set pins for input
  for (int i=1; i<=6; i++) pinMode(sense[i].pin,sense[i].mode);

  // Initialize EEPROM if necessary
  if (EEPROM_readLong(EE_key)!=19653291L) {
    EEPROM_writeLong(EE_key,19653291L);
    for (int l=4; l<1024; l++) EEPROM.write(l,0);
    // These are the default for the roof
    EEPROM_writeLong(EE_timeLeftToOpen,120000); // two minutes
    EEPROM_writeLong(EE_timeLeftToClose,0);
  }

  // Restore relay state
  #if POWER_DEVICE1_RELAY != OFF && POWER_DEVICE1_MEMORY != OFF
    if ((bool)EEPROM.read(EE_powerDevice_1)) setRelayOn(POWER_DEVICE1_RELAY);
  #endif
  #if POWER_DEVICE2_RELAY != OFF && POWER_DEVICE2_MEMORY != OFF
    if ((bool)EEPROM.read(EE_powerDevice_2)) setRelayOn(POWER_DEVICE2_RELAY);
  #endif
  #if POWER_DEVICE3_RELAY != OFF && POWER_DEVICE3_MEMORY != OFF
    if ((bool)EEPROM.read(EE_powerDevice_3)) setRelayOn(POWER_DEVICE3_RELAY);
  #endif
  #if POWER_DEVICE4_RELAY != OFF && POWER_DEVICE4_MEMORY != OFF
    if ((bool)EEPROM.read(EE_powerDevice_4)) setRelayOn(POWER_DEVICE4_RELAY);
  #endif
  #if POWER_DEVICE5_RELAY != OFF && POWER_DEVICE5_MEMORY != OFF
    if ((bool)EEPROM.read(EE_powerDevice_5)) setRelayOn(POWER_DEVICE5_RELAY);
  #endif
  #if POWER_DEVICE6_RELAY != OFF && POWER_DEVICE6_MEMORY != OFF
    if ((bool)EEPROM.read(EE_powerDevice_6)) setRelayOn(POWER_DEVICE6_RELAY);
  #endif

  // get ready to go
  weatherInit();
#ifdef ROOF_ON
  roofInit();
#endif
#if THERMOSTAT == ON
  thermostatInit();
#endif

#if ETHERNET_RESET != OFF
  // hold ethernet shield in reset
  pinMode(ETHERNET_RESET,OUTPUT);
  digitalWrite(ETHERNET_RESET,LOW);

  for (int l=0; l<2; l++) {
    delay(1000);
  #if WATCHDOG == ON
    wdt_reset();
  #endif
  }

  // let ethernet shield run
  digitalWrite(ETHERNET_RESET,HIGH);
#endif

  // wait for a bit just to be sure ethernet, etc. is up
  for (int l=0; l<3; l++) {
    delay(1000);
#if WATCHDOG == ON
    wdt_reset();
#endif
  }

  // Initialize the webserver
  www.setResponseHeader(http_defaultHeader);
  www.init();
  www.on("index.htm",index);
#if WEATHER == ON && WEATHER_CHARTS == ON
  www.on("weatherpage.htm",weatherPage);
#if WEATHER_SKY_QUAL == ON || WEATHER_CLOUD_CVR == ON
  www.on("skypage.htm",skyPage);
#endif
#endif
  www.on("relay",relays);
  www.on("setvar",setvar);
  www.on("miscstatus",miscstatus);
#if WEATHER == ON
  www.on("weather",weather);
#endif
#if THERMOSTAT == ON
  www.on("thermostat",thermostat);
  #if THERMOSTAT_HUMIDITY == ON
    www.on("thermostath",thermostath);
  #endif
#endif
#if ROR == ON
  www.on("roofstatus",roofstatus);
#endif
#if WEATHER_CHARTS == ON
  www.on("Chart.js");
#endif
  www.on("/",index);

  // Initialize the cmd server, timeout after 500ms
  Cmd.init(9999,500);
  Cmd1.init(9998,500);

  // Set variables
  msFiveMinuteCounter=millis()+5000UL;
  tst                =millis()+30000UL;
  insideTemperature  =-999.0;

  // for dimming the led's etc, 1 millisecond period
  Timer1.initialize(1000);
  Timer1.attachInterrupt(RelayPwmISR);

#if STAT_TIME_SOURCE == NTP
  delay(3000);
  Udp.begin(localPort);
  #if DEBUG_NPT == ON
    Serial.println("waiting for sync");
  #endif
  setSyncProvider(getNtpTime);
  setSyncInterval(24L*60L*60L); // sync time once a day
  if (now()>365UL*24UL*60UL*60UL) startupTime=now();
#elif STAT_TIME_SOURCE == DS3234 || STAT_TIME_SOURCE == DS3234INIT
  setSyncProvider(getDs3234Time);
  setSyncInterval(24L*60L*60L); // sync time once a day
  startupTime=now();
#endif

}

void loop()                     
{
#ifdef NexDomeLoop
  // keep NexDome alive
  NexDomeLoop();
#endif
  
#if WATCHDOG == ON
  wdt_reset();
#endif

#if STAT_TIME_SOURCE == NTP
  // double check time if it looks like it's not set go back and ask for it again every 10 minutes
  if (now()<365UL*24UL*60UL*60UL) {
    if (!fastNTPSync) { setSyncInterval(10L*60L); fastNTPSync=true; }
  } else {
    if (startupTime==0) startupTime=now();
    if (fastNTPSync) { setSyncInterval(24L*60L*60L); fastNTPSync=false; }
  }
#endif
  // ------------------------------------------------------------------------------------------
  // Handle comms

  // serve web-pages
  www.handleClient();

  // handle cmd-channel
  Cmd.handleClient();
  Cmd1.handleClient();

  // process commands
  processCommands();

#if LIGHT_SW_SENSE == ON && LIGHT_WRW_RELAY != OFF
  // The wall switch controls LED lights
  if (senseChanged(LIGHT_SW_SENSE)) {
    if (senseIsOn(LIGHT_SW_SENSE)) setRelayOn(LIGHT_WRW_RELAY); else setRelayOff(LIGHT_WRW_RELAY);
  }
#endif

  // ------------------------------------------------------------------------------------------
  // Process events

  // update relays
  RelayTimedOff();

  #if THERMOSTAT == ON
    thermostat();
  #endif

#if ROR == ON
  // Auto close the roof at 8am if requested
  if ((roofAutoClose) && (validTime())) {
    if ((hour()==8) && (!roofAutoCloseInitiated)) {
      // if motion is idle
      if (!roofIsMoving()) {
        // and the roof isn't closed, close it
        if (!roofIsClosed()) startRoofClose();
        roofAutoCloseInitiated=true;
      }
    }
    if (hour()==9) roofAutoCloseInitiated=false;
  }

#if ROR_AUTOCLOSE_SAFETY == ON
  // close the roof if safety status calls for it
  if (!isSafe()) {
    // if the roof isn't closed, and motion is idle, close it
    if ((!roofIsClosed()) && (!roofIsMoving()) startRoofClose();
  }
#endif

  if (roofIsMoving()) moveRoof();
#endif

  // Watch clouds
#if WEATHER == ON
  weatherPoll();
#endif
}

bool validTime() {
  return (now()<315360000);
}
