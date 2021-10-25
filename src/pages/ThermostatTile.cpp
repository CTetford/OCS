// thermostat ----------------------------------------------------------------------------------------------------------------
#include "ThermostatTile.h"

#if THERMOSTAT == ON
  #include "htmlHeaders.h"
  #include "htmlScripts.h"
  #include "htmlTabs.h"

  #include "weather.h"
  #include "../libApp/thermostatSensor/ThermostatSensor.h"
  #include "../libApp/relay/Relay.h"
  #include "../observatory/thermostat/Thermostat.h"

  void thermostatTile() {
    char temp[256] = "";

    strcpy_P(temp, htmlThermostatBegin);
    sendHtml(temp);

    strcpy_P(temp, htmlThermostatTemperature1);
    sendHtml(temp);
    thermostatTemperatureContents();
    strcpy_P(temp, htmlThermostatTemperature2);
    sendHtml(temp);

    #if THERMOSTAT_HUMIDITY == ON
      strcpy_P(temp, htmlThermostatHumidity1);
      sendHtml(temp);
      thermostatHumidityContents();
      strcpy_P(temp, htmlThermostatHumidity2);
      sendHtml(temp);
    #endif

    sendHtml(F("<br />"));

    #if HEAT_RELAY != OFF
    {
      strcpy_P(temp, htmlThermostatHeat1);
      sendHtml(temp);

      #if STAT_UNITS == IMPERIAL
        int h = round(getHeatSetpoint()*(9.0/5.0) + 32.0);
      if (getHeatSetpoint() == 0) h = 0;
      #else
        int h = round(thermostat.getHeatSetpoint());
      #endif

      char ws1[20] = "";
      if (h == 0) strcpy(ws1, "selected"); else strcpy(ws1, "");
      sprintf_P(temp, htmlThermostatOptionZero, ws1);
      sendHtml(temp);

      #if STAT_UNITS == IMPERIAL
        char unitHeat = 'F';
        int hs[11] = {40,50,60,65,67,68,69,70,71,72,73};
      #else
        char unitHeat = 'C';
        int hs[11] = {5,10,15,17,18,19,20,21,22,23,24};
      #endif

      for (int i = 0; i < 11; i++) {
        if (h == hs[i]) strcpy(ws1, "selected"); else strcpy(ws1, "");
        sprintf_P(temp, htmlThermostatOption, hs[i], ws1, hs[i], unitHeat);
        sendHtml(temp);
      }
      strcpy_P(temp,htmlThermostatHeat2);
      sendHtml(temp);
    }
    #endif

    #if COOL_RELAY != OFF
    {
      strcpy_P(temp, htmlThermostatCool1);
      sendHtml(temp);

      #if STAT_UNITS == IMPERIAL
        int c = round(getCoolSetpoint()*(9.0/5.0) + 32.0);
        if (getCoolSetpoint() == 0) c = 0;
      #else
        int c = round(thermostat.getCoolSetpoint());
      #endif

      char ws1[20] = "";
      if (c == 0) strcpy(ws1, "selected"); else strcpy(ws1, "");
      sprintf_P(temp, htmlThermostatOptionZero, ws1);
      sendHtml(temp);

      #if STAT_UNITS == IMPERIAL
        char unitCool = 'F';
        int cs[10] = {68,69,70,71,72,73,75,80,90,99};
      #else
        char unitCool = 'C';
        int cs[10] = {20,21,22,23,24,26,28,30,32,37};
      #endif

      for (int i = 0; i < 10; i++) {
        if (c == cs[i]) strcpy(ws1, "selected"); else strcpy(ws1, "");
        sprintf_P(temp, htmlThermostatOption, cs[i], ws1, cs[i], unitCool);
        sendHtml(temp);
      }
      strcpy_P(temp, htmlThermostatCool2);
      sendHtml(temp);
    }
    #endif

    strcpy_P(temp, htmlThermostatEnd);
    sendHtml(temp);
  }

  void thermostatTemperatureContents() {
    char temp[40] = "";
    
    float t = thermostatSensor.temperature();
    if (isnan(t)) {
      strcpy(temp, "Invalid");
    } else {
      #if STAT_UNITS == IMPERIAL
        t = t*(9.0/5.0) + 32.0;
        sprintF(temp, "%6.1f &deg;F ", t);
      #else
        sprintF(temp, "%6.1f &deg;C ", t);
      #endif

      #if HEAT_RELAY != OFF
        if (relay.isOn(HEAT_RELAY)) strcat(temp, "^"); else
      #endif
      #if COOL_RELAY != OFF
        if (relay.isOn(COOL_RELAY)) strcpy(temp, "*"); else 
      #endif
        strcat(temp, "-");
    }
    sendHtml(temp);
  }

  #if THERMOSTAT_HUMIDITY == ON
    void thermostatHumidityContents() {
      char temp[40] = "";
      
      float h = thermostatSensor.humidity();
      if (isnan(h)) {
        strcpy(temp, "Invalid");
      } else {
        sprintF(temp, "%5.1f ", h);
        strcat(temp, "%");
      }
      sendHtml(temp);
    }
  #endif

#endif
