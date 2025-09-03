/*!
 * @file WipperSnapper_I2C_Driver_QMC5883P.h
 *
 * Driver wrapper for the Adafruit QMC5883P 3-axis magnetometer.
 *
 * Publishes magnetic field magnitude in Gauss as SENSOR_TYPE_MAGNETIC_FIELD.
 */
#ifndef WipperSnapper_I2C_Driver_QMC5883P_H
#define WipperSnapper_I2C_Driver_QMC5883P_H

#include "WipperSnapper_I2C_Driver.h"
#include "Wippersnapper.h"

class Adafruit_QMC5883P; // forward

/**************************************************************************/
/*!
@brief  Class that provides a driver interface for a QMC5883P sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_QMC5883P : public WipperSnapper_I2C_Driver {
public:
  WipperSnapper_I2C_Driver_QMC5883P(TwoWire *i2c, uint16_t sensorAddress);
  ~WipperSnapper_I2C_Driver_QMC5883P();

  bool begin();
  bool getEventRaw(sensors_event_t *magEvent);

private:
  Adafruit_QMC5883P *_qmc = nullptr; ///< Pointer to the QMC5883P sensor object
};

#endif // WipperSnapper_I2C_Driver_QMC5883P_H
