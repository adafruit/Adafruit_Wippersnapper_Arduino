#ifndef WipperSnapper_I2C_Driver_VL53L4CD_H
#define WipperSnapper_I2C_Driver_VL53L4CD_H
#define SerialPort Serial

#include "WipperSnapper_I2C_Driver.h"
#include "vl53l4cd_class.h"

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VL53L4CD sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_VL53L4CD : public WipperSnapper_I2C_Driver {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VL53L4CD sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_VL53L4CD(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VL53L4CD sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_VL53L4CD() {
    // Called when a VL53L4CD component is deleted.
    delete _vl53l4cd;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VL53L4CD sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    // Initialize serial for output.
    SerialPort.begin(115200);
    SerialPort.println("Starting...");

    // Switch off VL53L4CD satellite component.
    _vl53l4cd->VL53L4CD_Off();

    //Initialize VL53L4CD satellite component.
    _vl53l4cd->InitSensor();

    // Program the highest possible TimingBudget, without enabling the
    // low power mode. This should give the best accuracy
    _vl53l4cd->VL53L4CD_SetRangeTiming(200, 0);

    // Start Measurements
    _vl53l4cd->VL53L4CD_StartRanging();

    return true;
  }

  bool getEventProximity(sensors_event_t *proximityEvent) {
    uint8_t NewDataReady = 0;
    VL53L4CD_Result_t results;
    uint8_t status;
    char report[64];

    do {
      status = _vl53l4cd->VL53L4CD_CheckForDataReady(&NewDataReady);
      return false;
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
      // (Mandatory) Clear HW interrupt to restart measurements
      sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

      // Read measured distance. RangeStatus = 0 means valid data
      sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
      snprintf(report, sizeof(report), "Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
              results.range_status,
              results.distance_mm,
              results.signal_per_spad_kcps);
      SerialPort.print(report);
    }

    proximityEvent->data[0] = results.distance_mm;
    return true;
  }

protected:
  VL53L4CD *_vl53l4cd; ///< Pointer to VL53L4CD ToF sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L4CD