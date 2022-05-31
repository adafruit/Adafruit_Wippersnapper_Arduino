/*!
 * @file WipperSnapper_Pixels.cpp
 *
 * This component interfaces with addressable pixel components.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "WipperSnapper_Pixels.h"

/*******************************************************************************/
/*!
    @brief    Constructor for Pixels component.
*/
/*******************************************************************************/
WipperSnapper_Pixels::WipperSnapper_Pixels() {}

/*******************************************************************************/
/*!
    @brief    Destructor for Pixels component.
*/
/*******************************************************************************/
WipperSnapper_Pixels::~WipperSnapper_Pixels() {}

/************************************************************************************/
/*!
    @brief    Initializes a new addressable pixel object, either DotStar or
   NeoPixel.
    @param    msgPixelsCreate
              A PixelsCreate message.
    @return   True if Pixel strip added, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::addPixel(
    wippersnapper_pixels_v1_PixelsCreate msgPixelsCreate) {
  bool is_success = true;

  if (msgPixelsCreate.has_neo_pixel_init) {
    if (WS.statusLEDActive == true &&
        msgPixelsCreate.neo_pixel_init.neo_pixel_pin == STATUS_NEOPIXEL_PIN)
      statusLEDDeinit();
    is_success = _neoDriver.addNeoPixel(msgPixelsCreate.pixel_num,
                                        msgPixelsCreate.pixel_brightness,
                                        msgPixelsCreate.neo_pixel_init);
  } else if (msgPixelsCreate.has_dot_star_init) {
    if (WS.statusLEDActive = true) // we only support one dotStar strand
      statusLEDDeinit();
    is_success = _dotStarDriver.addDotStar(msgPixelsCreate.pixel_num,
                                           msgPixelsCreate.pixel_brightness,
                                           msgPixelsCreate.dot_star_init);
  } else {
    is_success = false;
  }
  return is_success;
}

/************************************************************************************/
/*!
    @brief    Updates the configuration of an addressable pixel object.
    @param    msgPixelsUpdate
              A PixelUpdate message.
    @return   True if Pixel strip updated, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::updatePixel(
    wippersnapper_pixels_v1_PixelsUpdate msgPixelsUpdate) {
  bool is_success = true;
  if (msgPixelsUpdate.has_neo_pixel_config) {
    is_success =
        _neoDriver.updateNeoPixel((uint8_t)msgPixelsUpdate.pixel_brightness,
                                  msgPixelsUpdate.neo_pixel_config);
  } else if (msgPixelsUpdate.has_dot_star_config) {
    is_success =
        _dotStarDriver.updateDotStar((uint8_t)msgPixelsUpdate.pixel_brightness,
                                     msgPixelsUpdate.dot_star_config);
  } else {
    is_success = false;
  }
  return is_success;
}

/***********************************************************************/
/*!
    @brief    Deletes an existing addressable pixel object, if exists.
    @param    msgPixelsDelete
              A PixelsDelete message.
    @return   True if NeoPixel strip deleted, False otherwise.
*/
/**********************************************************************/
bool WipperSnapper_Pixels::deletePixel(
    wippersnapper_pixels_v1_PixelsDelete msgPixelsDelete) {
  bool is_success = true;
  if (msgPixelsDelete.has_neo_pixel_config) {
    is_success = _neoDriver.deleteNeoPixel(msgPixelsDelete.neo_pixel_config);
  } else if (msgPixelsDelete.has_dot_star_config) {
    is_success = _dotStarDriver.deleteDotStar(msgPixelsDelete.dot_star_config);
  } else {
    is_success = false;
  }
  return is_success;
}

/************************************************************************************/
/*!
    @brief    Fills the color of an addressable pixel object.
    @param    msgPixelsFillAll
              A PixelsFillAll message.
    @return   True if Pixel strip filled, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::fillPixel(
    wippersnapper_pixels_v1_PixelsFillAll msgPixelsFillAll) {
  bool is_success = true;
  if (msgPixelsFillAll.has_neo_pixel_config) {
    is_success = _neoDriver.fillNeoPixel(msgPixelsFillAll.color,
                                         msgPixelsFillAll.neo_pixel_config);
  } else if (msgPixelsFillAll.has_dot_star_config) {
    is_success = _dotStarDriver.fillDotStar(msgPixelsFillAll.color,
                                            msgPixelsFillAll.dot_star_config);
  } else {
    is_success = false;
  }
  return is_success;
}

/*********************************************************************/
/*!
    @brief    De-initializes a pixel component, releasing it for use
              as a status pixel.
*/
/********************************************************************/
void WipperSnapper_Pixels::deInitPixel() {
  if (_neoDriver.isInitialized)
    _neoDriver.deinitNeoPixels();
  if (_dotStarDriver.isInitialized)
    _dotStarDriver.deinitDotStars();
}