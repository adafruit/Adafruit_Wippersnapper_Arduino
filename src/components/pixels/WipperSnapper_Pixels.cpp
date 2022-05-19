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

WipperSnapper_Pixels::WipperSnapper_Pixels() {}

WipperSnapper_Pixels::~WipperSnapper_Pixels() {}

void WipperSnapper_Pixels::addPixel(
    wippersnapper_pixels_v1_PixelsCreate msgPixelsCreate) {
  msgPixelsCreate.pixel_num;
  msgPixelsCreate.pixel_brightness;
  if (msgPixelsCreate.has_neo_pixel_init) {
    addNeoPixel(msgPixelsCreate.pixel_num, msgPixelsCreate.pixel_brightness,
                msgPixelsCreate.neo_pixel_init);
  } else if (msgPixelsCreate.has_dot_star_init) {
    /* code */
  } else {
    // ERROR!
  }
}

void WipperSnapper_Pixels::updatePixel(
    wippersnapper_pixels_v1_PixelsUpdate msgPixelsUpdate) {
  if (msgPixelsUpdate.has_neo_pixel_config) {
    updateNeoPixel((uint8_t)msgPixelsUpdate.pixel_brightness,
                   msgPixelsUpdate.neo_pixel_config);
  }
}

void WipperSnapper_Pixels::deletePixel(
    wippersnapper_pixels_v1_PixelsDelete msgPixelsDelete) {
  if (msgPixelsDelete.has_neo_pixel_config) {
    deleteNeoPixel(msgPixelsDelete.neo_pixel_config);
  }
}

void WipperSnapper_Pixels::fillPixel(
    wippersnapper_pixels_v1_PixelsFillAll msgPixelsFillAll) {
  if (msgPixelsFillAll.has_neo_pixel_config) {
    fillNeoPixel(msgPixelsFillAll.color, msgPixelsFillAll.neo_pixel_config);
  }
}

// NeoPixel Driver
void WipperSnapper_Pixels::addNeoPixel(
    uint32_t pixelsNum, uint32_t pixelsBrightness,
    wippersnapper_pixels_v1_NeoPixelInit neoPixelInitMsg) {
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
}

void WipperSnapper_Pixels::updateNeoPixel(
    int8_t pixelBrightness,
    wippersnapper_pixels_v1_NeoPixelInit msgNeoPixelConfig) {
  // update NeoPixel, if exists
  for (int i = 0; i < _neopixels.size(); i++) {
    if (_neopixels.at(i)->getPin() == (int8_t)msgNeoPixelConfig.neo_pixel_pin) {
      // did we update the brightness on WipperSnapper Web?
      if (_neopixels.at(i)->getBrightness() != pixelBrightness) {
        _neopixels.at(i)->setBrightness(pixelBrightness);
      }
    }
  }
}

void WipperSnapper_Pixels::deleteNeoPixel(
    wippersnapper_pixels_v1_NeoPixelInit msgNeoPixelConfig) {
  // delete NeoPixel, if exists
  for (int i = 0; i < _neopixels.size(); i++) {
    if (_neopixels.at(i)->getPin() == (int8_t)msgNeoPixelConfig.neo_pixel_pin) {
      _neopixels.erase(_neopixels.begin() + i);
    }
  }
}

void WipperSnapper_Pixels::fillNeoPixel(
    uint32_t pixelColor,
    wippersnapper_pixels_v1_NeoPixelInit msgNeoPixelConfig) {
  // fill NeoPixel with one color, if object exists
  for (int i = 0; i < _neopixels.size(); i++) {
    if (_neopixels.at(i)->getPin() == (int8_t)msgNeoPixelConfig.neo_pixel_pin) {
      _neopixels.at(i)->fill(pixelColor);
    }
  }
}

// DotStar Driver
void WipperSnapper_Pixels::addDotStar() {}
void WipperSnapper_Pixels::updateDotStar() {}
void WipperSnapper_Pixels::deleteDotStar() {}
void WipperSnapper_Pixels::fillDotStar() {}