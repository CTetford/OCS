// -----------------------------------------------------------------------------------
// Weather/Safety monitor commands

#include "Weather.h"

#ifdef WEATHER_PRESENT

#include "../../libApp/weatherSensor/WeatherSensor.h"

bool Weather::command(char reply[], char command[], char parameter[], bool *supressFrame, bool *numericReply, CommandError *commandError) {
  UNUSED(*commandError);
  UNUSED(*numericReply);
  UNUSED(*supressFrame);
  UNUSED(*parameter);
  UNUSED(*command);
  UNUSED(*reply);

  if (command[0] == 'G') {
    #if WEATHER_TEMPERATURE == ON
    //  :G1#  Get outside temperature
    //         Returns: nnn.n#
    if (command[1] == '1' && parameter[0] == 0) {
      sprintF(reply, "%3.1f", weatherSensor.temperature());
      *numericReply = false;
    } else
    #endif
  
    #if WEATHER_CLOUD_CVR == ON
    //  :G2#  Get sky IR temperature
    //         Returns: nnn.n#
    if (command[1] == '2' && parameter[0] == 0) {
      sprintF(reply, "%3.1f", weatherSensor.skyTemperature());
      *numericReply = false;
    } else
    //  :G3#  Get differential sky temperature
    //         Returns: nnn.n#
    //         where <= 21 is cloudy
    if (command[1] == '3' && parameter[0] == 0) {
      sprintF(reply, "%3.1f", weather.getAvgSkyDiffTemp());
      *numericReply = false;
    } else
    #endif
  
    #if WEATHER_PRESSURE == ON
    //  :Gb#  Get absolute barometric pressure as Float
    //         Returns: n.nnn#
    //         where n ranges from about 980.0 to 1050.0 (mbar, sea-level compensated)
    if (command[1] == 'b' && parameter[0] == 0) {
      sprintF(reply, "%1.1f", weatherSensor.pressure());
      *numericReply = false;
    } else
    #endif
  
    //  :GC#  Get cloud description
    //         Example: :GC#
    //         Returns: sssss...#
    if (command[1] == 'C' && parameter[0] == 0) {
      #if WEATHER_CLOUD_CVR == ON
        strcpy(reply, weather.cloudCoverDescription());
      #else
        strcpy(reply, "N/A");
      #endif
      *numericReply = false;
    } else
  
    #if WEATHER_HUMIDITY == ON
    //  :Gh#  Get relative humidity reading as Float
    //         Returns: n.n#
    //         where n ranges from 0.0 to 100.0
    if (command[1] == 'h' && parameter[0] == 0) {
      sprintF(reply, "%1.1f", weatherSensor.humidity());
      *numericReply = false;
    } else
    #endif
  
    #if WEATHER_SKY_QUAL == ON
    //  :GQ#  Get sky quality in mag/arc-sec^2
    //         Returns: nnn.n#
    if ((command[1] == 'Q') && (parameter[0] == 0)) {
      sprintF(reply, "%3.1f", weatherSensor.skyQuality());
      *numericReply = false;
    } else
    #endif
  
    //  :GR#  Get rain sensor status
    //         Returns: n#
    //         -1000# is invalid, 0# is N/A, 1# is Rain, 2# is Warn, and 3# is Dry
    if (command[1] == 'R' && parameter[0] == 0) {
      #if WEATHER_RAIN == ON
        sprintf(reply,"%d", (int)lroundf(weatherSensor.rain()));
      #else
        strcpy(reply,"0");
      #endif
      *numericReply = false;
    } else
  
    #if WEATHER_CLOUD_CVR == ON
    //  :GS#  Get averaged sky differential temperature
    //         Returns: nnn.n#
    //         where <=21 is cloudy
    if ((command[1] == 'S') && (parameter[0] == 0)) {
      sprintF(reply, "%3.1f", weather.avgSkyDiffTemp);
      *numericReply = true;
    } else
    #endif
  
    //  :GW#  Get wind status
    //         Returns: OK#, HIGH#, or N/A#
    //         
    if (command[1] == 'W' && parameter[0] == 0) {
      #if WEATHER_WIND_SPD == ON
        if (weatherSensor.windspeed() > WEATHER_WIND_SPD_THRESHOLD) strcpy(reply, "HIGH"); else 
        if (isnan(weatherSensor.windspeed())) strcpy(reply, "Invalid"); else strcpy(reply, "OK");
      #else
        strcpy(reply,"N/A");
      #endif
      *numericReply = false;
    } else return false;
  } else

  #if WEATHER_WIND_SPD != OFF || WEATHER_CLOUD_CVR != OFF
  if (command[0] == 'I') {
    //  :IW#  get Weather threshold #defines
    //         Returns: 20,-14#, WEATHER_WIND_SPD_THRESHOLD,WEATHER_SAFE_THRESHOLD, N/A if sensor == OFF
    if (command[1] == 'W' && parameter[0] == 0) {
      char ws[20];
      if (WEATHER_WIND_SPD != OFF) { sprintf(ws, "%d", WEATHER_WIND_SPD_THRESHOLD);
      } else { sprintf(ws, "%s", "N/A"); }
      char wt[20]      ;
      if (WEATHER_CLOUD_CVR != OFF) { sprintf(wt, "%d", WEATHER_SAFE_THRESHOLD);
      } else { sprintf(wt, "%s", "N/A"); }
      sprintf(reply, "%s,%s", ws, wt);
      *numericReply = false;
    } else return false;
  }
  #endif
    return false;

  return true;
}

#endif
