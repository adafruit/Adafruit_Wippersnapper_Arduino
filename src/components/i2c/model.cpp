/*!
 * @file model.cpp
 *
 * Model for the i2c.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "model.h"

/***********************************************************************/
/*!
    @brief  I2C constructor
*/
/***********************************************************************/
I2cModel::I2cModel() {
    _msg_i2c_bus_scan = wippersnapper_i2c_I2cBusScan_init_default;
    _msg_i2c_bus_scanned = wippersnapper_i2c_I2cBusScan_init_default;
    _msg_i2c_device_add_replace = wippersnapper_i2c_I2cDeviceAddOrReplace_init_default;
    _msg_i2c_device_added_replaced = wippersnapper_i2c_I2cDeviceAddedOrReplaced_init_default;
    _msg_i2c_device_remove = wippersnapper_i2c_I2cDeviceRemove_init_default;
    _msg_i2c_device_removed = wippersnapper_i2c_I2cDeviceRemoved_init_default;
    _msg_i2c_device_event = wippersnapper_i2c_I2cDeviceEvent_init_default;
}

bool I2cModel::DecodeI2cDeviceAddReplace(pb_istream_t *stream) {
    // TODO!
return true;
}