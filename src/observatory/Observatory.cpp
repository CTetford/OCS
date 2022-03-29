// -----------------------------------------------------------------------------------
// Observatory
#include <TimeLib.h>  // from here: https://github.com/PaulStoffregen/Time

#include "Observatory.h"

#include "../lib/tasks/OnTask.h"
#include "../libApp/weatherSensor/WeatherSensor.h"
#include "../libApp/relay/Relay.h"
#include "../lib/sense/Sense.h"
#include "../lib/tls/Tls.h"

#include "power/Power.h"
#include "roof/Roof.h"
#include "safety/Safety.h"
#include "thermostat/Thermostat.h"
#include "weather/Weather.h"

#include "../pages/Pages.h"

#include "../alpaca/Alpaca.h"

WebServer apc;

int timeZone = TIME_ZONE;
time_t startupTime = 0;

#if CONNECTION_CHECK_HOURS != OFF
  EthernetClient connectionCheckClient;
#endif

unsigned long msFiveMinuteCounter;
bool xBusy = false;
bool validKey = true;

void observatoryWrapper() { observatory.poll(); }

void Observatory::init(const char *fwName, int fwMajor, int fwMinor, const char *fwPatch, int fwConfig) {
  strcpy(firmware.name, fwName);
  firmware.version.major = fwMajor;
  firmware.version.minor = fwMinor;
  strcpy(firmware.version.patch, fwPatch);
  firmware.version.config = fwConfig;

  // get nv ready
  if (!nv.isKeyValid(INIT_NV_KEY)) {
    VF("MSG: NV, invalid key wipe "); V(nv.size); VLF(" bytes");
    if (nv.verify()) { VLF("MSG: NV, ready for reset to defaults"); }
  } else { VLF("MSG: NV, correct key found"); }

  // disable SPI devices at start up
  if (SDCARD_CS_PIN != OFF) {
    pinMode(SDCARD_CS_PIN, OUTPUT);
    digitalWrite(SDCARD_CS_PIN, HIGH);
  }

  if (ETHERNET_CS_PIN != OFF) {
    pinMode(ETHERNET_CS_PIN, OUTPUT);
    digitalWrite(ETHERNET_CS_PIN, HIGH);
  }

  if (AUX_CS_PIN != OFF) {
    pinMode(AUX_CS_PIN, OUTPUT);
    digitalWrite(AUX_CS_PIN, HIGH);
  }

  // init is done, write the NV key if necessary
  if (!nv.hasValidKey()) {
    if (!nv.initError) {
      nv.writeKey((uint32_t)INIT_NV_KEY);
      nv.wait();
      if (!nv.isKeyValid(INIT_NV_KEY)) { DLF("ERR: NV, failed to read back key!"); } else { VLF("MSG: NV, reset complete"); }
    }
  }

  relay.init();

  // Restore relay state
  #if POWER_DEVICE1_RELAY != OFF && POWER_DEVICE1_MEMORY != OFF
    if (nv.readC(NV_POWER_DEVICE1)) relay.on(POWER_DEVICE1_RELAY);
  #endif
  #if POWER_DEVICE2_RELAY != OFF && POWER_DEVICE2_MEMORY != OFF
    if (nv.readC(NV_POWER_DEVICE2)) relay.on(POWER_DEVICE2_RELAY);
  #endif
  #if POWER_DEVICE3_RELAY != OFF && POWER_DEVICE3_MEMORY != OFF
    if (nv.readC(NV_POWER_DEVICE3)) relay.on(POWER_DEVICE3_RELAY);
  #endif
  #if POWER_DEVICE4_RELAY != OFF && POWER_DEVICE4_MEMORY != OFF
    if (nv.readC(NV_POWER_DEVICE4)) relay.on(POWER_DEVICE4_RELAY);
  #endif
  #if POWER_DEVICE5_RELAY != OFF && POWER_DEVICE5_MEMORY != OFF
    if (nv.readC(NV_POWER_DEVICE5)) relay.on(POWER_DEVICE5_RELAY);
  #endif
  #if POWER_DEVICE6_RELAY != OFF && POWER_DEVICE6_MEMORY != OFF
    if (nv.readC(NV_POWER_DEVICE6)) relay.on(POWER_DEVICE6_RELAY);
  #endif

  // senses are added in order and "forced" so all are present even if
  // the pin is OFF; this means the sense handle is the sense number
  if (SENSE1_PIN != OFF) sense.add(SENSE1_PIN, SENSE1_INIT_STATE, SENSE1_ON_STATE, true);
  if (SENSE2_PIN != OFF) sense.add(SENSE2_PIN, SENSE2_INIT_STATE, SENSE2_ON_STATE, true);
  if (SENSE3_PIN != OFF) sense.add(SENSE3_PIN, SENSE3_INIT_STATE, SENSE3_ON_STATE, true);
  if (SENSE4_PIN != OFF) sense.add(SENSE4_PIN, SENSE4_INIT_STATE, SENSE4_ON_STATE, true);
  if (SENSE5_PIN != OFF) sense.add(SENSE5_PIN, SENSE5_INIT_STATE, SENSE5_ON_STATE, true);
  if (SENSE6_PIN != OFF) sense.add(SENSE6_PIN, SENSE6_INIT_STATE, SENSE6_ON_STATE, true);
  if (SENSE7_PIN != OFF) sense.add(SENSE7_PIN, SENSE7_INIT_STATE, SENSE7_ON_STATE, true);
  if (SENSE8_PIN != OFF) sense.add(SENSE8_PIN, SENSE8_INIT_STATE, SENSE8_ON_STATE, true);

  #if ROOF == ON
    roof.init();
  #endif

  safety.init();

  #if THERMOSTAT == ON
    thermostat.init();
  #endif

  #if WEATHER == ON
    weather.init();
  #endif

  WDT_ENABLE;

  // ----------------------------------------------------------------------
  // initialize ethernet

  // Note: Ethernet webserver has limitations
  // HANDLER_COUNT_MAX     16
  // PARAMETER_COUNT_MAX   8

  #if OPERATIONAL_MODE == WIFI
    wifiStart();
  #endif

  www.on("index.htm", index);
  #if FAV_ICON == ON
    www.on("favicon.ico");
  #endif
  #if WEATHER == ON && WEATHER_CHARTS == ON
    www.on("weatherpage.htm", weatherPage);
    #if WEATHER_SKY_QUAL == ON || WEATHER_CLOUD_CVR == ON
      www.on("skypage.htm", skyPage);
    #endif
  #endif
  www.on("setvar", indexAjax);
  www.on("relay", relaysAjax);
  www.on("miscstatus", statusContents);
  #if WEATHER == ON
    www.on("weather", weatherContents);
  #endif
  #if THERMOSTAT == ON
    www.on("thermostatt", thermostatTemperatureContents);
    #if THERMOSTAT_HUMIDITY == ON
      www.on("thermostath", thermostatHumidityContents);
    #endif
  #endif
  #if ROOF == ON
    www.on("roofstatus", roofContents);
  #endif
  #if WEATHER_CHARTS == ON
    www.on("Chart.js", NULL);
  #endif
  www.on("/", index);
  www.onNotFound(handleNotFound);

  // ASCOM Alpaca
  apc.on("setup", alpacaSetup);
  apc.on("setup/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/setup", alpacaSetupSafetyMonitor);

  apc.on("management/apiversions", alpacaManagementApiVersions);
  apc.on("management/v1/description", alpacaManagementDescription);
  apc.on("management/v1/configureddevices", alpacaManagementConfiguredDevices);

  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/action", alpacaDefaultAction);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/commandblind", alpacaMethodNotImplemented);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/commandbool", alpacaMethodNotImplemented);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/commandstring", alpacaMethodNotImplemented);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/connected", alpacaSafetyMonitorConnected);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/description", alpacaDescription);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/driverinfo", alpacaDriverInfo);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/driverversion", alpacaDriverVersion);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/interfaceversion", alpacaInterfaceVersion);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/name", alpacaName);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/supportedactions", alpacaDefaultSupportedActions);
  apc.on("api/v1/safetymonitor/" ALPACA_DEVICE_NUMBER "/issafe", alpacaSafetyMonitorIsSafe);

  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/action", alpacaDefaultAction);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/commandblind", alpacaMethodNotImplemented);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/commandbool", alpacaMethodNotImplemented);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/commandstring", alpacaMethodNotImplemented);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/connected", alpacaObservingConditionsConnected);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/description", alpacaDescription);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/driverinfo", alpacaDriverInfo);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/driverversion", alpacaDriverVersion);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/interfaceversion", alpacaInterfaceVersion);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/name", alpacaName);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/supportedactions", alpacaDefaultSupportedActions);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/averageperiod", alpacaObservingConditionsAveragePeriod);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/cloudcover", alpacaObservingConditionsCloudCover);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/dewpoint", alpacaObservingConditionsDewPoint);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/humidity", alpacaObservingConditionsHumidity);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/pressure", alpacaObservingConditionsPressure);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/rainrate", alpacaObservingConditionsRainRate);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/skybrightness", alpacaObservingConditionsSkyBrightness);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/skyquality", alpacaObservingConditionsSkyQuality);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/skytemperature", alpacaObservingConditionsSkyTemperature);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/starfwhm", alpacaObservingConditionsStarFwhm);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/temperature", alpacaObservingConditionsTemperature);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/winddirection", alpacaObservingConditionsWindDirection);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/windgust", alpacaObservingConditionsWindGust);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/windspeed", alpacaObservingConditionsWindSpeed);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/refresh", alpacaObservingConditionsRefresh);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/sensordescription", alpacaObservingConditionsSensorDescription);
  apc.on("api/v1/observingconditions/" ALPACA_DEVICE_NUMBER "/timesincelastupdate", alpacaObservingConditionsTimeSinceLastUpdate);

  apc.onNotFound(alpacaNotFoundError);

  www.begin();
  apc.begin(ALPACA_PORT);

  #if TIME_LOCATION_SOURCE != OFF
    tls.init();
  #endif

  // start observatory monitor task
  VF("MSG: Observatory, start monitor task (rate 1ms priority 4)... ");
  if (tasks.add(1, 0, true, 4, observatoryWrapper, "Obsrvty")) { VLF("success"); } else { VLF("FAILED!"); }
}

void Observatory::connectionCheck() {
  #if CONNECTION_CHECK_HOURS != OFF
    static int connectionCheckTry = 0;
    static unsigned long nextConnectionCheck = 1000UL*3600UL*(CONNECTION_CHECK_HOURS/CHECK_FAST);
    if ((long)(millis() - nextConnectionCheck) > 0) {
      connectionCheckTry++;

      int success = connectionCheckClient.connect(connectCheckIP, 80);

      if (success) {
        VLF("MSG: Connection Check Success");
        connectionCheckClient.stop();
        connectionCheckTry = 0;
        nextConnectionCheck = millis() + (1000UL*3600UL*(CONNECTION_CHECK_HOURS/CHECK_FAST));
      } else {
        VLF("MSG: Connection Check FAILED!");
        nextConnectionCheck = millis() + (1000UL*CONNECT_RECHECK_TIME);
      }

    #if WATCHDOG == ON_CC
      if (!success && !roof.isMoving() && connectionCheckTry >= CONNECT_REBOOT_TRIES) {
        VLF("MSG: Forcing Watchdog Reboot");
        while (true) {};
      }
    #endif

    #if ETHERNET_RESET_PIN != OFF
      if (!success && connectionCheckTry == CONNECT_RESET_TRIES) {
        VLF("MSG: Reset Ethernet shield");
        digitalWrite(ETHERNET_RESET_PIN, LOW);
        tasks.yield(1000);
        digitalWrite(ETHERNET_RESET_PIN, HIGH);
        tasks.yield(1000);

        www.restart();

        #if STANDARD_COMMAND_CHANNEL == ON
          ipSerial.restart();
        #endif

        #if PERSISTENT_COMMAND_CHANNEL == ON
          pipSerial.restart();
        #endif

        #if TIME_LOCATION_SOURCE == NTP
          tls.restart();
        #endif

        #if CONNECTION_FAIL_WATCHDOG == OFF
          connectionCheckTry = 0;
        #endif
      }
    #endif
    }
  #endif
}

void Observatory::poll() {

  connectionCheck();

  #if TIME_LOCATION_SOURCE == NTP
    // double check time if it looks like it's not set go back and ask for it again every 10 minutes
    if (now() < 365L*24L*60L*60L) {
      if (!fastNTPSync) { setSyncInterval(10L*60L); fastNTPSync = true; }
    } else {
      if (startupTime == 0) startupTime = now();
      if (fastNTPSync) { setSyncInterval(24L*60L*60L); fastNTPSync = false; }
    }
  #endif

  // ------------------------------------------------------------------------------------------
  // Handle comms

  // serve web-pages
  www.handleClient();

  // serve ASCOM Alpaca
  apc.handleClient();

  // a wall switch can control lights
  #if LIGHT_SW_SENSE != OFF && LIGHT_WRW_RELAY != OFF
    if (sense.changed(LIGHT_SW_SENSE)) {
      if (sense.isOn(LIGHT_SW_SENSE)) relay.on(LIGHT_WRW_RELAY); else relay.off(LIGHT_WRW_RELAY);
    }
  #endif

  // keep DHCP alive if used
  if (ethernetManager.settings.dhcpEnabled) Ethernet.maintain();
}

Observatory observatory;
