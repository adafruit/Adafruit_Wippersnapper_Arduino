/*!
 * @file drvLis3mdl.h
 *
 * Driver wrapper for the Adafruit LIS3MDL 3-axis magnetometer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * MIT license, all text here must be included in any redistribution.
 */
#ifndef DRV_LIS3MDL_H
#define DRV_LIS3MDL_H

#include "Wippersnapper_V2.h"
#include "drvBase.h"
#include <Adafruit_LIS3MDL.h>


/**************************************************************************/
/*! 
    @brief  Class that provides a driver interface for a LIS3MDL magnetometer.
*/
/**************************************************************************/
class drvLis3mdl : public drvBase {
public:
  drvLis3mdl(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  ~drvLis3mdl();

  bool begin() override;

  bool getEventRaw(sensors_event_t *rawEvent) override;

  bool getEventBoolean(sensors_event_t *booleanEvent) override;

  bool getEventMagneticField(sensors_event_t *magEvent) override;

protected:
  void ConfigureDefaultSensorTypes() override;

private:
  bool readMagEvent(sensors_event_t *event);
  bool computeMagnitude(const sensors_event_t &event, float &magnitude);

  Adafruit_LIS3MDL *_mag = nullptr;
};

#endif // DRV_LIS3MDL_H
