/*!
 * @file src/components/display/drivers/dispDrvThinkInk.h
 *
 * Base driver for ThinkInk EPD displays.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DRV_THINKINK
#define WS_DRV_THINKINK

#include "dispDrvBase.h"

/*!
    @brief  Driver for ThinkInk EPD displays.
*/
class drvDispThinkInk : public dispDrvBase {
public:
  using FnCreateAdafruit_EPD = std::function<Adafruit_EPD *(
      int16_t, int16_t, int16_t, int16_t, int16_t, thinkinkmode_t)>;

  /*!
      @brief  ThinkInk-compatible panel identifier -> Adafruit_EPD factory
     table.
      @return A reference to the factory map.
  */
  static const std::map<std::string, FnCreateAdafruit_EPD> &
  getAdafruitEPDFactory() {
    static const std::map<std::string, FnCreateAdafruit_EPD>
        adafruitEPDFactory = {
            // Monochrome
            {"154-mono-D27",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Mono_D27 *d =
                   new ThinkInk_154_Mono_D27(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"154-mono-D67",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Mono_D67 *d =
                   new ThinkInk_154_Mono_D67(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"154-mono-M10",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Mono_M10 *d =
                   new ThinkInk_154_Mono_M10(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-mono-B72",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Mono_B72 *d =
                   new ThinkInk_213_Mono_B72(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-mono-B73",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Mono_B73 *d =
                   new ThinkInk_213_Mono_B73(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-mono-BN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Mono_BN *d =
                   new ThinkInk_213_Mono_BN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-mono-GDEY0213B74",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Mono_GDEY0213B74 *d =
                   new ThinkInk_213_Mono_GDEY0213B74(dc, rst, cs, sram_cs,
                                                     busy);
               d->begin(mode);
               return d;
             }},
            {"213-mono-M21",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Mono_M21 *d =
                   new ThinkInk_213_Mono_M21(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"290-mono-BN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Mono_BN *d =
                   new ThinkInk_290_Mono_BN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"290-mono-M06",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Mono_M06 *d =
                   new ThinkInk_290_Mono_M06(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"370-mono-BAAMFGN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_370_Mono_BAAMFGN *d =
                   new ThinkInk_370_Mono_BAAMFGN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"420-mono-BN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_420_Mono_BN *d =
                   new ThinkInk_420_Mono_BN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"420-mono-M06",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_420_Mono_M06 *d =
                   new ThinkInk_420_Mono_M06(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"583-mono-AAAMFGN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_583_Mono_AAAMFGN *d =
                   new ThinkInk_583_Mono_AAAMFGN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"750-mono-AAAMFGN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_750_Mono_AAAMFGN *d =
                   new ThinkInk_750_Mono_AAAMFGN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            // Grayscale (quad-level)
            {"154-gray-M05",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Grayscale4_M05 *d =
                   new ThinkInk_154_Grayscale4_M05(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"154-gray-T8",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Grayscale4_T8 *d =
                   new ThinkInk_154_Grayscale4_T8(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-gray-MFGN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Grayscale4_MFGN *d =
                   new ThinkInk_213_Grayscale4_MFGN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-gray-T5",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Grayscale4_T5 *d =
                   new ThinkInk_213_Grayscale4_T5(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"266-gray-MFGN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_266_Grayscale4_MFGN *d =
                   new ThinkInk_266_Grayscale4_MFGN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"270-gray-W3",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_270_Grayscale4_W3 *d =
                   new ThinkInk_270_Grayscale4_W3(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"290-gray-EAAMFGN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Grayscale4_EAAMFGN *d =
                   new ThinkInk_290_Grayscale4_EAAMFGN(dc, rst, cs, sram_cs,
                                                       busy);
               d->begin(mode);
               return d;
             }},
            {"290-gray-FPC7519",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Grayscale4_FPC7519 *d =
                   new ThinkInk_290_Grayscale4_FPC7519(dc, rst, cs, sram_cs,
                                                       busy);
               d->begin(mode);
               return d;
             }},
            {"290-gray-T5",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Grayscale4_T5 *d =
                   new ThinkInk_290_Grayscale4_T5(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"420-gray-MFGN",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_420_Grayscale4_MFGN *d =
                   new ThinkInk_420_Grayscale4_MFGN(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"420-gray-T2",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_420_Grayscale4_T2 *d =
                   new ThinkInk_420_Grayscale4_T2(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"426-gray-GDEQ",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_426_Grayscale4_GDEQ *d =
                   new ThinkInk_426_Grayscale4_GDEQ(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            // Tricolor
            {"154-tricolor-RW",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Tricolor_RW *d =
                   new ThinkInk_154_Tricolor_RW(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"154-tricolor-Z17",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Tricolor_Z17 *d =
                   new ThinkInk_154_Tricolor_Z17(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"154-tricolor-Z90",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_154_Tricolor_Z90 *d =
                   new ThinkInk_154_Tricolor_Z90(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-tricolor-MFGNR",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Tricolor_MFGNR *d =
                   new ThinkInk_213_Tricolor_MFGNR(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-tricolor-RW",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Tricolor_RW *d =
                   new ThinkInk_213_Tricolor_RW(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"213-tricolor-Z16",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Tricolor_Z16 *d =
                   new ThinkInk_213_Tricolor_Z16(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"266-tricolor-MFGNR",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_266_Tricolor_MFGNR *d =
                   new ThinkInk_266_Tricolor_MFGNR(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"270-tricolor-C44",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_270_Tricolor_C44 *d =
                   new ThinkInk_270_Tricolor_C44(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"270-tricolor-Z70",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_270_Tricolor_Z70 *d =
                   new ThinkInk_270_Tricolor_Z70(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"290-tricolor-RH",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Tricolor_RH *d =
                   new ThinkInk_290_Tricolor_RH(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"290-tricolor-Z10",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Tricolor_Z10 *d =
                   new ThinkInk_290_Tricolor_Z10(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"290-tricolor-Z13",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Tricolor_Z13 *d =
                   new ThinkInk_290_Tricolor_Z13(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"290-tricolor-Z94",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_290_Tricolor_Z94 *d =
                   new ThinkInk_290_Tricolor_Z94(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"370-tricolor-BABMFGNR",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_370_Tricolor_BABMFGNR *d =
                   new ThinkInk_370_Tricolor_BABMFGNR(dc, rst, cs, sram_cs,
                                                      busy);
               d->begin(mode);
               return d;
             }},
            {"420-tricolor-MFGNR",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_420_Tricolor_MFGNR *d =
                   new ThinkInk_420_Tricolor_MFGNR(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"420-tricolor-RW",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_420_Tricolor_RW *d =
                   new ThinkInk_420_Tricolor_RW(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"420-tricolor-Z21",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_420_Tricolor_Z21 *d =
                   new ThinkInk_420_Tricolor_Z21(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"583-tricolor-AABMFGNR",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_583_Tricolor_AABMFGNR *d =
                   new ThinkInk_583_Tricolor_AABMFGNR(dc, rst, cs, sram_cs,
                                                      busy);
               d->begin(mode);
               return d;
             }},
            {"750-tricolor-AABMFGNR",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_750_Tricolor_AABMFGNR *d =
                   new ThinkInk_750_Tricolor_AABMFGNR(dc, rst, cs, sram_cs,
                                                      busy);
               d->begin(mode);
               return d;
             }},

            // Quadcolor
            {"213-quad-AJHE5",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_213_Quadcolor_AJHE5 *d =
                   new ThinkInk_213_Quadcolor_AJHE5(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
            {"352-quad-AJHE5",
             [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs,
                int16_t busy, thinkinkmode_t mode) -> Adafruit_EPD * {
               ThinkInk_352_Quadcolor_AJHE5 *d =
                   new ThinkInk_352_Quadcolor_AJHE5(dc, rst, cs, sram_cs, busy);
               d->begin(mode);
               return d;
             }},
        };
    return adafruitEPDFactory;
  }

  /*!
      @brief  Constructor for a ThinkInk EPD display.
      @param  epdPanel   The EPD panel name (e.g., "583-mono-Z16").
      @param  epdMode    The EPD mode (e.g.,
     ws_display_EPDMode_EPD_MODE_GRAYSCALE4).
      @param  dc       Data/Command pin.
      @param  rst      Reset pin.
      @param  cs       Chip Select pin.
      @param  sram_cs  Optional SRAM Chip Select pin (-1 for none).
      @param  busy     Optional Busy pin (-1 for none).
  */
  drvDispThinkInk(char *epdPanel, ws_display_EPDMode epdMode, int16_t dc,
                  int16_t rst, int16_t cs, int16_t sram_cs = -1,
                  int16_t busy = -1)
      : dispDrvBase(dc, rst, cs, sram_cs, busy), _display(nullptr) {
    strncpy(_panel, epdPanel, sizeof(_panel) - 1);
    _panel[sizeof(_panel) - 1] = '\0';
    _mode = epdMode;
  }

  /*!
      @brief  Destructor for a ThinkInk EPD display driver.
  */
  ~drvDispThinkInk() {
    if (_display) {
      _display->clearBuffer();
      delete _display;
      _display = nullptr;
    }
    if (_reader) {
      delete _reader;
      _reader = nullptr;
    }
  }

  /*!
      @brief  Finds and initializes the display panel based on the configured
     panel identifier.
      @return True if the panel was found and initialized, False otherwise.
  */
  bool createEPD() {
    // Map the ws_display mode to the THINKINK driver mode
    thinkinkmode_t thinkink_mode = THINKINK_MONO;
    const char *mode_name = "THINKINK_MONO";
    if (_mode == ws_display_EPDMode_EPD_MODE_TRICOLOR) {
      thinkink_mode = THINKINK_TRICOLOR;
      mode_name = "THINKINK_TRICOLOR";
    } else if (_mode == ws_display_EPDMode_EPD_MODE_GRAYSCALE4) {
      thinkink_mode = THINKINK_GRAYSCALE4;
      mode_name = "THINKINK_GRAYSCALE4";
    } else if (_mode == ws_display_EPDMode_EPD_MODE_MONO_PARTIAL) {
      thinkink_mode = THINKINK_MONO_PARTIAL;
      mode_name = "THINKINK_MONO_PARTIAL";
    } else if (_mode == ws_display_EPDMode_EPD_MODE_QUADCOLOR) {
      thinkink_mode = THINKINK_QUADCOLOR;
      mode_name = "THINKINK_QUADCOLOR";
    }

    // Look up the factory entry for the configured panel identifier
    const std::map<std::string, FnCreateAdafruit_EPD> &adafruitEPDFactory =
        getAdafruitEPDFactory();
    std::map<std::string, FnCreateAdafruit_EPD>::const_iterator it =
        adafruitEPDFactory.find(_panel);
    if (it == adafruitEPDFactory.end())
      return false;

    _display = it->second(_pin_dc, _pin_rst, _pin_cs, _pin_sram_cs, _pin_busy,
                          thinkink_mode);

    if (!_display)
      return false;

    return true;
  }

  /*!
      @brief  Initializes the display for the configured panel and mode.
      @return True if initialization succeeded, False otherwise.
  */
  bool begin() override {
    // Attempt to create an EPD object
    if (!createEPD())
      return false;

    _display->setRotation(_rotation);
    _display->setTextSize(_text_sz);
    _display->setTextColor(EPD_BLACK);
    _display->setTextWrap(false);
    _height = _display->height();
    _width = _display->width();
    _display->clearBuffer();
    // Only fully refresh/clear the EPD on a cold-boot, not when it wakes from
    // sleep
    if (!didBootFromSleep())
      _display->display();
    _reader = new Adafruit_ImageReader_EPD();
    if (!_reader) {
      WS_DEBUG_PRINTLN("Failed to create Adafruit_ImageReader_EPD");
      return false;
    }
    return true;
  }

  /*!
      @brief  Returns if the display using the Marquee features.
      @return True if the display is using Marquee features, False otherwise.
  */
  bool isMarqueeEPD() override {
    if (!_reader)
      return false;
    return true;
  }

  /*!
      @brief  Draws the status bar and the Adafruit IO username.
      @param  io_username  Adafruit IO username to display.
  */
  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;
    _display->clearBuffer();
    _display->fillRect(0, 0, _display->width(), EPD_STATUS_BAR_HEIGHT,
                       EPD_BLACK);
    _display->fillRect(EPD_STATUS_BAR_BORDER, EPD_STATUS_BAR_BORDER,
                       _display->width() - (2 * EPD_STATUS_BAR_BORDER),
                       EPD_STATUS_BAR_HEIGHT - (2 * EPD_STATUS_BAR_BORDER),
                       EPD_WHITE);
    _display->setTextSize(1);
    _display->setTextColor(EPD_BLACK);
    _display->setCursor(5, 6);
    _display->print(io_username);
    _statusbar_icons_y = EPD_STATUS_BAR_BORDER +
                         ((EPD_STATUS_BAR_HEIGHT - 2 * EPD_STATUS_BAR_BORDER -
                           EPD_STATUS_BAR_ICON_SZ) /
                          2);
    _statusbar_icon_battery_x =
        _display->width() - EPD_STATUS_BAR_ICON_SZ - EPD_STATUS_BAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x -
                             EPD_STATUS_BAR_ICON_SZ -
                             EPD_STATUS_BAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x - EPD_STATUS_BAR_ICON_SZ -
                              EPD_STATUS_BAR_ICON_SPACING;
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, EPD_STATUS_BAR_ICON_SZ,
                         EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
    _display->display();
  }

  /*!
      @brief  Updates the status bar icons based on current state.
      @param  rssi         Current WiFi RSSI value.
      @param  bat          Current battery level (0-100).
      @param  mqtt_status  True if MQTT is connected.
  */
  void updateStatusBar(int8_t rssi, uint8_t bat, bool mqtt_status) override {
    if (!_display)
      return;
    bool update_rssi = abs(rssi - _statusbar_rssi) >= 5;
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;
    if (!update_rssi && !update_mqtt)
      return;
    if (update_mqtt) {
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ,
                         EPD_WHITE);
      _display->drawBitmap(
          _statusbar_icon_cloud_x, _statusbar_icons_y,
          mqtt_status ? epd_bmp_cloud_online : epd_bmp_cloud_offline,
          EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
      _statusbar_mqtt_connected = mqtt_status;
    }
    if (update_rssi) {
      const unsigned char *wifi_icon = epd_bmp_wifi_no_signal;
      if (rssi >= -50)
        wifi_icon = epd_bmp_wifi_full;
      else if (rssi >= -60)
        wifi_icon = epd_bmp_wifi_fair;
      else if (rssi >= -70)
        wifi_icon = epd_bmp_wifi_weak;
      _display->fillRect(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         EPD_STATUS_BAR_ICON_SZ, EPD_STATUS_BAR_ICON_SZ,
                         EPD_WHITE);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, EPD_STATUS_BAR_ICON_SZ,
                           EPD_STATUS_BAR_ICON_SZ, EPD_BLACK);
      _statusbar_rssi = rssi;
    }
    _display->display();
  }

  void writeMessage(const char *message) override {
    if (!_display)
      return;
    _display->fillRect(0, EPD_STATUS_BAR_HEIGHT, _display->width(),
                       _display->height() - EPD_STATUS_BAR_HEIGHT, EPD_WHITE);
    int16_t y_idx = EPD_STATUS_BAR_HEIGHT + 4;
    _display->setCursor(0, y_idx);
    int16_t line_height = 8 * _text_sz;
    size_t msg_size = strlen(message);
    _display->setTextSize(_text_sz);
    for (size_t i = 0; i < msg_size; i++) {
      if (y_idx + line_height > _height)
        break;
      char parsed_char = 0;
      bool is_newline = false;
      if (parseWriteToken(message, msg_size, i, parsed_char, is_newline,
                          char(247))) {
        if (is_newline) {
          y_idx += line_height;
          if (y_idx + line_height > _height)
            break;
          _display->setCursor(0, y_idx);
        } else {
          _display->write(parsed_char);
        }
        continue;
      }
      _display->print(message[i]);
    }
    _display->display();
  }

  /*!
      @brief  Draws an assembled canvas (raw 1bpp .BMP file bytes) to the
     display.
      @param  bmp  Pointer to the complete BMP file bytes.
      @param  len  Length of the BMP buffer, in bytes.
      @return True if accepted, False otherwise.
  */
  virtual bool drawMarqueeEPD(const uint8_t *bmp, size_t len) override {
    if (!_display || !_reader)
      return false;
    _display->clearBuffer();

    // Draw the BMP to the display
    uint32_t t_decode_start = millis();
    ImageReturnCode rc = _reader->drawBMP(bmp, len, *_display, 0, 0);
    uint32_t t_decode = millis() - t_decode_start;
    if (rc != IMAGE_SUCCESS) {
      WS_DEBUG_PRINT("[display] ERROR: drawBMP rc: ");
      WS_DEBUG_PRINTLNVAR((int)rc);
      return false;
    }

    uint32_t t_refresh_start = millis();
    _display->display();
    WS_DEBUG_PRINT("[display] decode ms: ");
    WS_DEBUG_PRINTVAR(t_decode);
    WS_DEBUG_PRINT(" refresh ms: ");
    WS_DEBUG_PRINTLNVAR(millis() - t_refresh_start);
    return true;
  }

private:
  Adafruit_EPD *_display = nullptr;
  Adafruit_ImageReader_EPD *_reader = nullptr;
  char _panel[32] = {0};
  ws_display_EPDMode _mode = ws_display_EPDMode_EPD_MODE_MONO;
};

#endif // WS_DRV_THINKINK
