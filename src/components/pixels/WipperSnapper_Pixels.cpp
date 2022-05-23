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
    is_success = _neoDriver.addNeoPixel(msgPixelsCreate.pixel_num,
                                        msgPixelsCreate.pixel_brightness,
                                        msgPixelsCreate.neo_pixel_init);
  } else if (msgPixelsCreate.has_dot_star_init) {
    is_success =
        addDotStar(msgPixelsCreate.pixel_num, msgPixelsCreate.pixel_brightness,
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
    is_success = updateDotStar((uint8_t)msgPixelsUpdate.pixel_brightness,
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
    is_success = deleteDotStar(msgPixelsDelete.dot_star_config);
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
    is_success =
        fillDotStar(msgPixelsFillAll.color, msgPixelsFillAll.dot_star_config);
  } else {
    is_success = false;
  }
  return is_success;
}

/************************************************************************************/
/*!
    @brief    Initializes a new DotStar object
    @param    pixelsNum
              The number of pixels in the DotStar strip.
    @param    pixelsBrightness
              The pixel strip's brightness.
    @param    msgDotStarConfig
              A DotStar initialization message.
    @returns  True if initialized successfully, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::addDotStar(
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
bool WipperSnapper_Pixels::updateDotStar(
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
bool WipperSnapper_Pixels::deleteDotStar(
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
bool WipperSnapper_Pixels::fillDotStar(
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