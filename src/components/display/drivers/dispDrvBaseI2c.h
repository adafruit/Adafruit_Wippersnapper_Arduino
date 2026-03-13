/*!
 * @file src/components/display/drivers/dispDrvBaseI2c.h
 *
 * Adapter that wraps I2C output drivers (drvOutputBase) behind the
 * dispDrvBase interface so the display controller can manage them.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISP_DRV_I2C_ADAPTER_H
#define WS_DISP_DRV_I2C_ADAPTER_H

#include "dispDrvBase.h"
#include "../../i2c/drivers/drvOutputBase.h"

/*!
    @brief  Adapter that wraps an I2C output driver (drvOutputBase)
            behind the display driver interface (dispDrvBase).
*/
class dispDrvBaseI2c : public dispDrvBase {
public:
  /*!
      @brief  Constructor that takes ownership of the I2C output driver.
      @param  drv  Pointer to an I2C output driver (will be deleted on
                    destruction).
  */
  dispDrvBaseI2c(drvOutputBase *drv) : dispDrvBase(), _drv(drv) {}

  /*!
      @brief  Destructor.

      Deletes the owned I2C output driver, if one was provided.
  */
  ~dispDrvBaseI2c() {
    if (_drv) {
      delete _drv;
      _drv = nullptr;
    }
  }

  /*!
      @brief  Initializes the wrapped I2C output driver.
      @return True if the wrapped driver exists and initializes successfully.
  */
  bool begin() override {
    if (!_drv)
      return false;
    return _drv->begin();
  }

  /*!
      @brief  Writes a message using the wrapped I2C output driver.
      @param  message      Null-terminated message to send.
      @param  clear_first  Unused for I2C output adapters.
      @param  cursor_x     Unused horizontal cursor position.
      @param  cursor_y     Unused vertical cursor position.

      I2C output drivers currently expose only a simple message write API,
      so cursor placement and clear behavior are ignored.
  */
  void writeMessage(const char *message, bool clear_first = true,
                    int32_t cursor_x = 0, int32_t cursor_y = 0) override {
    // I2C output drivers only support simple WriteMessage for now
    _drv->WriteMessage(message);
  }

private:
  /*! @brief  Owned I2C output driver instance. */
  drvOutputBase *_drv;
};

#endif // WS_DISP_DRV_I2C_ADAPTER_H
