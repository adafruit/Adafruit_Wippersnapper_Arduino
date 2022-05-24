#include "WipperSnapper_Pixels_Drv_NeoPixel.h"

WipperSnapper_Pixels_Drv_NeoPixel::WipperSnapper_Pixels_Drv_NeoPixel() {}

WipperSnapper_Pixels_Drv_NeoPixel::~WipperSnapper_Pixels_Drv_NeoPixel() {}

bool WipperSnapper_Pixels_Drv_NeoPixel::addNeoPixel(
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
    @return   True if NeoPixel strip found, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels_Drv_NeoPixel::updateNeoPixel(
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
    @return   True if NeoPixel strip found, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels_Drv_NeoPixel::deleteNeoPixel(
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
    @return   True if NeoPixel strip found, False otherwise.
*/
/***********************************************************************************/
bool WipperSnapper_Pixels_Drv_NeoPixel::fillNeoPixel(
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
