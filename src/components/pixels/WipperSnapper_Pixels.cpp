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
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::addPixel(
    wippersnapper_pixels_v1_PixelsCreate msgPixelsCreate) {
  msgPixelsCreate.pixel_num;
  msgPixelsCreate.pixel_brightness;
  if (msgPixelsCreate.has_neo_pixel_init) {
    addNeoPixel(msgPixelsCreate.pixel_num, msgPixelsCreate.pixel_brightness,
                msgPixelsCreate.neo_pixel_init);
  } else if (msgPixelsCreate.has_dot_star_init) {
    addDotStar(msgPixelsCreate.pixel_num, msgPixelsCreate.pixel_brightness,
               msgPixelsCreate.dot_star_init);
  } else {
    return false;
  }
  return true;
}

/************************************************************************************/
/*!
    @brief    Updates the configuration of an addressable pixel object.
    @param    msgPixelsUpdate
              A PixelUpdate message.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::updatePixel(
    wippersnapper_pixels_v1_PixelsUpdate msgPixelsUpdate) {
  if (msgPixelsUpdate.has_neo_pixel_config) {
    updateNeoPixel((uint8_t)msgPixelsUpdate.pixel_brightness,
                   msgPixelsUpdate.neo_pixel_config);
  } else if (msgPixelsUpdate.has_dot_star_config) {
    updateDotStar((uint8_t)msgPixelsUpdate.pixel_brightness,
                  msgPixelsUpdate.dot_star_config);
  } else {
    return false;
  }
  return true;
}

/***********************************************************************/
/*!
    @brief    Deletes an existing addressable pixel object, if exists.
    @param    msgPixelsDelete
              A PixelsDelete message.
*/
/**********************************************************************/
bool WipperSnapper_Pixels::deletePixel(
    wippersnapper_pixels_v1_PixelsDelete msgPixelsDelete) {
  if (msgPixelsDelete.has_neo_pixel_config) {
    return deleteNeoPixel(msgPixelsDelete.neo_pixel_config);
  } else if (msgPixelsDelete.has_dot_star_config) {
    return deleteDotStar(msgPixelsDelete.dot_star_config);
  } else {
    return false;
  }
  return true;
}

/************************************************************************************/
/*!
    @brief    Fills the color of an addressable pixel object.
    @param    msgPixelsFillAll
              A PixelsFillAll message.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::fillPixel(
    wippersnapper_pixels_v1_PixelsFillAll msgPixelsFillAll) {
  if (msgPixelsFillAll.has_neo_pixel_config) {
    fillNeoPixel(msgPixelsFillAll.color, msgPixelsFillAll.neo_pixel_config);
  } else if (msgPixelsFillAll.has_dot_star_config) {
    fillDotStar(msgPixelsFillAll.color, msgPixelsFillAll.dot_star_config);
  } else {
    return false;
  }
  return true;
}

/************************************************************************************/
/*!
    @brief    Initializes a new NeoPixel object
    @param    pixelsNum
              The number of pixels in the NeoPixel strip.
    @param    pixelsBrightness
              The pixel strip's brightness.
    @param    neoPixelInitMsg
              A NeoPixelInit message.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::addNeoPixel(
    uint32_t pixelsNum, uint32_t pixelsBrightness,
    wippersnapper_pixels_v1_NeoPixelConfig neoPixelInitMsg) {
  // Configure neoPixelType
  uint16_t neoType;
  if (neoPixelInitMsg.neo_pixel_type ==
      wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_RGB) {
    neoType = NEO_RGB + NEO_KHZ800;
  } else if (neoPixelInitMsg.neo_pixel_type ==
             wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_GRB) {
    neoType = NEO_GRB + NEO_KHZ800;
  } else if (neoPixelInitMsg.neo_pixel_type ==
             wippersnapper_pixels_v1_NeoPixelType_NEO_PIXEL_TYPE_RGBW) {
    neoType = NEO_RGBW + NEO_KHZ800;
  } else {
    return false;
  }

  if (neoPixelInitMsg.neo_pixel_speed ==
      wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_KHZ800) {
    neoType += NEO_KHZ800;
  } else if (neoPixelInitMsg.neo_pixel_speed ==
             wippersnapper_pixels_v1_NeoPixelSpeed_NEO_PIXEL_SPEED_KHZ400) {
    neoType += NEO_KHZ400;
  } else {
    neoType += NEO_KHZ800; // default
  }

  // Initialize NeoPixel
  _neopixel = new Adafruit_NeoPixel(
      (int16_t)pixelsNum, (int16_t)neoPixelInitMsg.neo_pixel_pin, neoType);
  _neopixel->begin();
  _neopixel->setBrightness((uint8_t)pixelsBrightness);
  // Add to vector
  _neopixels.push_back(_neopixel);
  return true;
}

/************************************************************************************/
/*!
    @brief    Updates the configuration of NeoPixel object.
    @param    pixelBrightness
              The pixel strip's brightness.
    @param    msgNeoPixelConfig
              A NeoPixel configuration message.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::updateNeoPixel(
    int8_t pixelBrightness,
    wippersnapper_pixels_v1_NeoPixelConfig msgNeoPixelConfig) {
  // update NeoPixel, if exists
  for (int i = 0; i < _neopixels.size(); i++) {
    if (_neopixels.at(i)->getPin() == (int8_t)msgNeoPixelConfig.neo_pixel_pin) {
      _neopixels.at(i)->setBrightness(pixelBrightness);
      _neopixels.at(i)->show();
      return true;
    }
  }
  return false;
}

/************************************************************************************/
/*!
    @brief    Deletes a NeoPixel object, if exists.
    @param    msgNeoPixelConfig
              A NeoPixel configuration message.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::deleteNeoPixel(
    wippersnapper_pixels_v1_NeoPixelConfig msgNeoPixelConfig) {
  // delete NeoPixel, if exists
  for (int i = 0; i < _neopixels.size(); i++) {
    if (_neopixels.at(i)->getPin() == (int8_t)msgNeoPixelConfig.neo_pixel_pin) {
      _neopixels.erase(_neopixels.begin() + i);
      return true;
    }
  }
  return false;
}

/************************************************************************************/
/*!
    @brief    Fills the color of a NeoPixel strip.
    @param    pixelColor
              The desired color to fill a NeoPixel strip.
    @param    msgNeoPixelConfig
              A NeoPixel configuration message.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels::fillNeoPixel(
    uint32_t pixelColor,
    wippersnapper_pixels_v1_NeoPixelConfig msgNeoPixelConfig) {
  // fill NeoPixel with one color, if object exists
  for (int i = 0; i < _neopixels.size(); i++) {
    if (_neopixels.at(i)->getPin() == (int8_t)msgNeoPixelConfig.neo_pixel_pin) {
      _neopixels.at(i)->fill(pixelColor);
      _neopixels.at(i)->show();
      return true;
    }
  }
  return false;
}

// DotStar Driver
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