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
  if (msgPixelsCreate.pixel_type ==
      wippersnapper_pixels_v1_PixelType_PIXEL_TYPE_NEOPIXEL) {
    addNeoPixel(msgPixelsCreate.pixel_num, msgPixelsCreate.pixel_brightness,
                msgPixelsCreate.neo_pixel_init);
  } else if (msgPixelsCreate.pixel_type ==
             wippersnapper_pixels_v1_PixelType_PIXEL_TYPE_DOTSTAR) {
    /* code */
  } else {
    // ERROR!
  }
}

void WipperSnapper_Pixels::updatePixel(
    wippersnapper_pixels_v1_PixelsUpdate msgPixelsUpdate) {}

void WipperSnapper_Pixels::deletePixel(
    wippersnapper_pixels_v1_PixelsDelete msgPixelsDelete) {}

void WipperSnapper_Pixels::fillPixel(
    wippersnapper_pixels_v1_PixelsFillAll msgPixelsFillAll) {}

// NeoPixel Driver
void WipperSnapper_Pixels::addNeoPixel(
    uint32_t pixelsNum, uint32_t pixelsBrightness,
    wippersnapper_pixels_v1_NeoPixelInit neoPixelInitMsg) {
  // call NeoPixel constructor
  int16_t pixelNum = pixelsNum;
  // TODO: This is using default neoPixelType constructor, parse out from
  // neopixelinitmsg!
  _neopixel = new Adafruit_NeoPixel((int16_t)pixelsNum,
                                    (int16_t)neoPixelInitMsg.neo_pixel_pin);
  _neopixel->begin();
  _neopixel->setBrightness((uint8_t)pixelsBrightness);

  // push back to vector
  // TODO
}
void WipperSnapper_Pixels::updateNeoPixel() {}
void WipperSnapper_Pixels::deleteNeoPixel() {}
void WipperSnapper_Pixels::fillNeoPixel() {}

// DotStar Driver
void WipperSnapper_Pixels::addDotStar() {}
void WipperSnapper_Pixels::updateDotStar() {}
void WipperSnapper_Pixels::deleteDotStar() {}
void WipperSnapper_Pixels::fillDotStar() {}