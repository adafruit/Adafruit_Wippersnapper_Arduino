/*!
 * @file drvLsm303dlh.h
 *
 * Driver wrapper for the classic Adafruit LSM303DLH combo sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * MIT license, all text here must be included in any redistribution.
 */
#ifndef DRV_LSM303DLH_H
#define DRV_LSM303DLH_H

#include "Wippersnapper_V2.h"
#include "drvBase.h"

#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>

/**************************************************************************/
/*!
  @brief  Driver interface for the legacy Adafruit LSM303DLH combo sensor.
*/
/**************************************************************************/
class drvLsm303dlh : public drvBase {
public:
  drvLsm303dlh(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
               const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /**************************************************************************/
  /*!
    @brief  Destructor for the LSM303DLH driver.
  */
  /**************************************************************************/
  ~drvLsm303dlh();

  /**************************************************************************/
  /*!
    @brief  Initializes the accelerometer and magnetometer peripherals.
    @returns True if initialization succeeded, False otherwise.
  */
  /**************************************************************************/
  bool begin() override;

  /**************************************************************************/
  /*!
    @brief  Populates a raw magnitude reading into a sensor event.
    @param  rawEvent Pointer to the destination sensor event.
    @returns True if the event was populated successfully.
  */
  /**************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) override;

  /**************************************************************************/
  /*!
    @brief  Retrieves the accelerometer vector event.
    @param  accelEvent Pointer to the destination sensor event.
    @returns True if the event was populated successfully.
  */
  /**************************************************************************/
  bool getEventAccelerometer(sensors_event_t *accelEvent) override;

  /**************************************************************************/
  /*!
    @brief  Retrieves the magnetic field vector event.
    @param  magEvent Pointer to the destination sensor event.
    @returns True if the event was populated successfully.
  */
  /**************************************************************************/
  bool getEventMagneticField(sensors_event_t *magEvent) override;

protected:
  /**************************************************************************/
  /*!
    @brief  Registers the default set of virtual sensor types.
  */
  /**************************************************************************/
  void ConfigureDefaultSensorTypes() override;

private:
  /**************************************************************************/
  /*!
    @brief  Releases any allocated sensor instances.
  */
  /**************************************************************************/
  void teardown();
  /**************************************************************************/
  /*!
    @brief  Computes the linear acceleration magnitude in m/s^2.
    @param  magnitude Reference to store the computed value.
    @returns True if the magnitude was computed successfully.
  */
  /**************************************************************************/
  bool computeAccelMagnitude(float &magnitude);

  Adafruit_LSM303_Accel_Unified *_accel = nullptr;
  Adafruit_LSM303DLH_Mag_Unified *_mag = nullptr;
};

#endif // DRV_LSM303DLH_H
