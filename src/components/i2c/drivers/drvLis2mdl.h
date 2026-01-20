/*!
 * @file drvLis2mdl.h
 *
 * Driver wrapper for the Adafruit LIS2MDL 3-axis magnetometer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * MIT license, all text here must be included in any redistribution.
 */
#ifndef DRV_LIS2MDL_H
#define DRV_LIS2MDL_H

#include "Wippersnapper_V2.h"
#include "drvBase.h"

#include <Adafruit_LIS2MDL.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a LIS2MDL magnetometer.
*/
/**************************************************************************/
class drvLis2mdl : public drvBase {
public:
  drvLis2mdl(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /**************************************************************************/
  /*!
      @brief  Destructor for the LIS2MDL driver wrapper.
  */
  /**************************************************************************/
  ~drvLis2mdl();

  /**************************************************************************/
  /*!
      @brief  Initializes the LIS2MDL sensor and begins I2C.
      @returns True if initialized successfully, False otherwise.
  */
  /**************************************************************************/
  bool begin() override;

  /**************************************************************************/
  /*!
      @brief  Gets a raw magnetometer magnitude event (micro Tesla).
      @param  rawEvent Pointer to the destination sensor event.
      @returns True if the event was obtained successfully.
  */
  /**************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) override;

  /**************************************************************************/
  /*!
      @brief  Placeholder boolean event implementation for compatibility.
      @param  booleanEvent Pointer to the destination sensor event.
      @returns True once the placeholder event has been populated.
  */
  /**************************************************************************/
  bool getEventBoolean(sensors_event_t *booleanEvent) override;

  /**************************************************************************/
  /*!
      @brief  Retrieves the LIS2MDL's magnetometer vector event.
      @param  magEvent Pointer to the destination sensor event.
      @returns True if the event was obtained successfully.
  */
  /**************************************************************************/
  bool getEventMagneticField(sensors_event_t *magEvent) override;

protected:
  /**************************************************************************/
  /*!
      @brief  Registers the default virtual sensors exposed by the driver.
  */
  /**************************************************************************/
  void ConfigureDefaultSensorTypes() override;

private:
  /**************************************************************************/
  /*!
      @brief  Reads a magnetometer event from the LIS2MDL helper.
      @param  event Pointer to the destination sensor event.
      @returns True if the event was obtained successfully.
  */
  /**************************************************************************/
  bool readMagEvent(sensors_event_t *event);

  /**************************************************************************/
  /*!
      @brief  Computes the vector magnitude of a magnetometer reading.
      @param  event Magnetometer event to evaluate.
      @param  magnitude Reference to store the computed magnitude (micro Tesla).
      @returns True if the magnitude was computed successfully.
  */
  /**************************************************************************/
  bool computeMagnitude(const sensors_event_t &event, float &magnitude);

  Adafruit_LIS2MDL *_mag = nullptr;
};

#endif // DRV_LIS2MDL_H
