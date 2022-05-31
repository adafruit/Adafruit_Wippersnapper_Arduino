/*!
 * @file WipperSnapper_Pixels_Drv_DotStar.cpp
 *
 * Component driver for DotStar (APA102) Addressable RGB LEDs.
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
#include "WipperSnapper_Pixels_Drv_DotStar.h"

WipperSnapper_Pixels_Drv_DotStar::WipperSnapper_Pixels_Drv_DotStar() {
  isInitialized = true;
}

WipperSnapper_Pixels_Drv_DotStar::~WipperSnapper_Pixels_Drv_DotStar() {
  isInitialized = false;
}

/************************************************************************************/
/*!
    @brief    Initializes a new DotStar object and adds it to a container of
              ptrs to DotStar objects.
    @param    pixelsNum
              The number of pixels in the DotStar strip.
    @param    pixelsBrightness
              The pixel strip's brightness.
    @param    msgDotStarConfig
              A DotStar initialization message.
    @returns  True if initialized successfully, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels_Drv_DotStar::addDotStar(
    uint32_t pixelsNum, uint32_t pixelsBrightness,
    wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig) {
  // init dotstar driver
  if (!msgDotStarConfig.use_hardware_spi)
    _dotstar = new Adafruit_DotStar(
        (uint16_t)pixelsNum, (uint8_t)msgDotStarConfig.pin_data,
        (uint8_t)msgDotStarConfig.pin_clock); // todo: add ordering param
  else
    _dotstar = new Adafruit_DotStar((uint16_t)pixelsNum); // TODO: Add ordering
  _dotstar->begin();
  _dotstar->clear();
  _dotstar->setBrightness((uint8_t)pixelsBrightness);
  // push ptr to driver into vec.
  _dotstars.push_back(_dotstar);
  return true;
}

/************************************************************************************/
/*!
    @brief    Updates the configuration of a DotStar object.
    @param    pixelBrightness
              The pixel strip's brightness.
    @param    msgDotStarConfig
              A DotStar configuration message.
    @returns  True if DotStar updated succesfully, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels_Drv_DotStar::updateDotStar(
    int8_t pixelBrightness,
    wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig) {
  // Update DotStar, if exists
  for (int i = 0; i < _dotstars.size(); i++) {
    if (_dotstars.at(i)->numPixels() == (int8_t)msgDotStarConfig.pixel_num) {
      _dotstars.at(i)->setBrightness(pixelBrightness);
      _dotstars.at(i)->show();
      return true;
    }
  }
  return false;
}

/************************************************************************************/
/*!
    @brief    Deletes a DotStar object.
    @param    msgDotStarConfig
              A DotStar configuration message.
    @return   True if DotStar deleted successfully, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels_Drv_DotStar::deleteDotStar(
    wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig) {
  // Delete DotStar, if exists
  for (int i = 0; i < _dotstars.size(); i++) {
    if (_dotstars.at(i)->numPixels() == (int8_t)msgDotStarConfig.pixel_num) {
      _dotstars.erase(_dotstars.begin() + i);
      return true;
    }
  }
  return false;
}

/***************************************************************/
/*!
    @brief    Deletes all DotStar objects and releases memory.
*/
/**************************************************************/
void WipperSnapper_Pixels_Drv_DotStar::deinitDotStars() {
  for (int i = 0; i < _dotstars.size(); i++) {
    _dotstars.erase(_dotstars.begin() + i);
  }

  /************************************************************************************/
  /*!
      @brief    Fills the color of a DotStar strip.
      @param    pixelColor
                The desired color to fill a DotStar strip.
      @param    msgDotStarConfig
                A DotStar configuration message.
      @returns  True if DotStar strip was found and filled, False otherwise.
  */
  /***********************************************************************************/
  bool WipperSnapper_Pixels_Drv_DotStar::fillDotStar(
      uint32_t pixelColor,
      wippersnapper_pixels_v1_DotStarConfig msgDotStarConfig) {
    // Fill DotStar strip, if exists
    for (int i = 0; i < _dotstars.size(); i++) {
      if (_dotstars.at(i)->numPixels() == (int8_t)msgDotStarConfig.pixel_num) {
        _dotstars.at(i)->fill(pixelColor);
        _dotstars.at(i)->show();
        return true;
      }
    }
    return false;
  }