#ifndef WipperSnapper_I2C_Driver_BH1750_H
#define WipperSnapper_I2C_Driver_BH1750_H

#include "WipperSnapper_I2C_Driver.h"
#include <hp_BH1750.h> //include the library for the BH1750 sensor

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a BH1750 Light sensor.

            This driver uses the H-Resolution Mode and the default measurement 
            time register (MTreg) of 69. According to the datasheet this is 
            the recommended mode for most applications. Typical measurement 
            time in this mode is 120ms

            This driver uses the One Time Measurement feature of the BH1750. The sensor
            returns to Power Down mode after each reading. 
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_BH1750 : public WipperSnapper_I2C_Driver
{
public:
    /*******************************************************************************/
    /*!
        @brief    Constructor for a BH1750 sensor.
        @param    i2c
                  The I2C interface.
        @param    sensorAddress
                  7-bit device address.
    */
    /*******************************************************************************/
    WipperSnapper_I2C_Driver_BH1750(TwoWire *i2c, uint16_t sensorAddress)
        : WipperSnapper_I2C_Driver(i2c, sensorAddress)
    {
        _i2c = i2c;
        _sensorAddress = sensorAddress;
    }

    /*******************************************************************************/
    /*!
        @brief    Destructor for a BH1750 sensor.
    */
    /*******************************************************************************/
    ~WipperSnapper_I2C_Driver_BH1750()
    {
        // Called when a BH1750 component is deleted.
        delete _bh1750;
    }

    /*******************************************************************************/
    /*!
        @brief  Initializes the BH1750 sensor and begins I2C.
                The set the quality to the H-Resolution Mode.
        @returns  True if initialized successfully, False otherwise.
    */
    /*******************************************************************************/
    bool begin()
    {
        _bh1750 = new hp_BH1750();
        // attempt to initialize BH1750
        if (!_bh1750->begin(_sensorAddress, _i2c))
            return false;
        // Set to the recommended quality setting
        _bh1750->setQuality(BH1750_QUALITY_HIGH);
        return true;
    }

    /*******************************************************************************/
  /*!
      @brief    Performs a light sensor read using the One Time Measurement
                feature of the BH1750. The sensor goes to Power Down mode after
                each reading.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    _bh1750->start();
    lightEvent->light = _bh1750->getLux();
    return true;
  }

protected:
    hp_BH1750 *_bh1750; ///< Pointer to BH1750 light sensor object
};

#endif // WipperSnapper_I2C_Driver_BH1750