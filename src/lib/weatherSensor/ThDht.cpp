// -----------------------------------------------------------------------------------------------------------------
// gets outside temperature in deg. C
// gets relative humidity in %
#include "ThDht.h"

#if WEATHER_SENSOR_TH_DHT != OFF

#include "../../tasks/OnTask.h"

extern float _temperature;
extern bool _temperatureAssigned;

extern float _humidity;
extern bool _humidityAssigned;

void dhtWrapper() { dhtw.poll(); }

// setup anemometer
bool Dhtw::init() {
  if (active) return true;

  if (_temperatureAssigned && _humidityAssigned) return false;

  if (!_temperatureAssigned) getT = true;
  if (!_humidityAssigned) getH = true;

  int index = WEATHER_SENSOR_TH_DHT - 1;
  if (index < 0 || index > 7) return false;

  dhtSensor = new DHT(sensePin[index], WEATHER_SENSOR_TH_DHT_TYPE);
  dhtSensor->begin();

  VF("MSG: Dhtw, start monitor task (default rate priority 7)... ");
  if (tasks.add(WEATHER_SENSOR_SAMPLE_PERIOD, 0, true, 7, dhtWrapper)) {
    VL("success");
    _temperatureAssigned = true;
    _humidityAssigned = true;
    active = true;
  } else { VL("FAILED!"); }

  return active;
}

// update
void Dhtw::poll() {
  if (!active) return;

  if (getT) _temperature = dhtSensor->readTemperature();
  tasks.yield(500);
  if (getH) _humidity = dhtSensor->readHumidity();
}

Dhtw dhtw;

#endif
