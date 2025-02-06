/*!
 * @file drvVl6180x.h
 *
 * Device driver for the VL6180X ToF sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2024 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_VL6180X_H
#define DRV_VL6180X_H

#include "Wippersnapper.h"
#include "drvBase.h"
#include <Adafruit_VL6180X.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL6180X sensor.
*/
/**************************************************************************/
class drvVl6180x : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL6180X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  drvVl6180x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL6180X sensor.
  */
  /*******************************************************************************/
  ~drvVl6180x() {
    // Called when a VL6180X component is deleted.
    delete _vl6180x;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VL6180X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _vl6180x = new Adafruit_VL6180X(_address);
    return _vl6180x->begin(_i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VL6180X's current proximity.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    uint8_t range = _vl6180x->readRange();
    uint8_t status = _vl6180x->readRangeStatus();

    if (status != VL6180X_ERROR_NONE) {
      if ((status >= VL6180X_ERROR_SYSERR_1) &&
          (status <= VL6180X_ERROR_SYSERR_5)) {
        WS_DEBUG_PRINTLN("VL6180X: System error");
      } else if (status == VL6180X_ERROR_ECEFAIL) {
        WS_DEBUG_PRINTLN("VL6180X: ECE failure");
      } else if (status == VL6180X_ERROR_NOCONVERGE) {
        WS_DEBUG_PRINTLN("VL6180X: No convergence");
      } else if (status == VL6180X_ERROR_RANGEIGNORE) {
        WS_DEBUG_PRINTLN("VL6180X: Ignoring range");
      } else if (status == VL6180X_ERROR_SNR) {
        WS_DEBUG_PRINTLN("VL6180X: Signal/Noise error");
      } else if (status == VL6180X_ERROR_RAWUFLOW) {
        WS_DEBUG_PRINTLN("VL6180X: Raw reading underflow");
      } else if (status == VL6180X_ERROR_RAWOFLOW) {
        WS_DEBUG_PRINTLN("VL6180X: Raw reading overflow");
      } else if (status == VL6180X_ERROR_RANGEUFLOW) {
        WS_DEBUG_PRINTLN("VL6180X: Range reading underflow");
      } else if (status == VL6180X_ERROR_RANGEOFLOW) {
        WS_DEBUG_PRINTLN("VL6180X: Range reading overflow");
      }
      proximityEvent->data[0] = NAN;
    } else {
      proximityEvent->data[0] = range;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a light sensor reading.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    // TODO: Update when I2C Sensor Properties allow setting custom Gain, etc.
    // Gain_5 results in max 41.6klux with cover glass - See 2.10.3 in datasheet
    float notRealLux = _vl6180x->readLux(VL6180X_ALS_GAIN_5);
    if (notRealLux < 0 || notRealLux > 41700) {
      lightEvent->light = NAN;
    } else {
      lightEvent->light = notRealLux;
    }
    return true;
  }

protected:
  Adafruit_VL6180X *_vl6180x; ///< Pointer to VL6180X temperature sensor object
};

#endif // drvVl6180x