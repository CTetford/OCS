// -----------------------------------------------------------------------------------------------------------------------------
// Weather related functions
#include "ThSi7021.h"

#if WEATHER_SENSOR_TH_SI7021 != OFF

#include "../../tasks/OnTask.h"

#include <Adafruit_Si7021.h> // https://github.com/adafruit/Adafruit_Si7021
Adafruit_Si7021 siSensor = Adafruit_Si7021();

extern float _temperature;
extern bool _temperatureAssigned;

extern float _humidity;
extern bool _humidityAssigned;

void si7021Wrapper() { si7021w.poll(); }

bool Si7021w::init() {
  if (active) return true;

  if (_temperatureAssigned && _humidityAssigned) return false;

  if (!_temperatureAssigned) getT = true;
  if (!_humidityAssigned) getH = true;

  if (siSensor.begin()) {
    // follow any I2C device in-library init with a reset of the I2C bus speed
    #ifdef HAL_WIRE_RESET_AFTER_CONNECT
      Wire.end();
      Wire.begin();
    #endif
    Wire.setClock(HAL_WIRE_CLOCK);

    VF("MSG: Si7021w, start monitor task (rate 30s priority 7)... ");
    if (tasks.add(30000, 0, true, 7, si7021Wrapper, "weaSi7")) {
      VL("success");
      _temperatureAssigned = true;
      _humidityAssigned = true;
      active = true;
    } else { VL("FAILED!"); }
  } else { DLF("WRN: Si7021w.init(), SI7021 (I2C 0x40) not found"); }

  return active;
}

// update
void Si7021w::poll() {
  if (!active) return;

  if (getT) _temperature = siSensor.readTemperature();
  tasks.yield(500);
  if (getH) _humidity = siSensor.readHumidity();
}

Si7021w si7021w;

#endif
