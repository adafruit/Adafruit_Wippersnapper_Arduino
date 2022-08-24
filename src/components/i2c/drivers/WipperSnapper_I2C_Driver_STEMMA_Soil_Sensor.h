#ifndef WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor_H
#define WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_seesaw.h>

class WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor : public WipperSnapper_I2C_Driver {
    public:
    WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor(TwoWire *i2c, uint16_t sensorAddress)
    : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
        _i2c = i2c;
        _sensorAddress = sensorAddress;
    }

    ~WipperSnapper_I2C_Driver_STEMMA_Soil_Sensor() {
        delete _seesaw;
    }

    /*******************************************************************************/
    /*!
        @brief    Initializes the soil sensor and begins I2C.
        @returns  True if initialized successfully, False otherwise.
    */
    /*******************************************************************************/
    bool begin()
    {
        _seesaw = new Adafruit_seesaw(_i2c);
        return _seesaw->begin(_sensorAddress);
    }

    bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
        tempEvent->temperature = _seesaw->getTemp();
    }

    bool getEventRaw(sensors_event_t *rawEvent) {
        // check if sensor is enabled and data is available
        if (_RawSensorPeriod != 0) {
            return false;
        }
        // TODO: Update this should we add a capacitive moisture type to
        // adafruit_sensor
        rawEvent->data[0] = (float) _seesaw->touchRead(0);
        return true;
    }

    protected:
    Adafruit_seesaw *_seesaw;
};

#endif