/*!
 * @file src/components/display/drivers/dispDrvMipiDsi.h
 *
 * Driver for MIPI DSI video-mode panels on the ESP32-P4 (V2), currently the
 * ILI9881C-based Raspberry Pi Touch Display 2 (7") on the Metro ESP32-P4.
 *
 * The panel's on-board MCU (I2C 0x45) owns panel reset and backlight; the
 * ILI9881C is configured over DSI command mode, then driven in DPI video mode
 * from a single RGB888 (packed B,G,R) frame buffer in PSRAM that an
 * Adafruit_GFX renders to. RGB888 input matches the bring-up proven on the
 * Metro; with RGB565 input the panel only showed flat blue.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DISP_DRV_MIPI_DSI_H
#define WS_DISP_DRV_MIPI_DSI_H

#include "dispDrvBase.h"
#include "dispDrvMipiDsiIli9881cInit.h"
// TEMPORARY HIL comparison: Touch Display 2 init (GPL-2.0-only header)
#include "dispDrvMipiDsiIli9881cInitRpiTd2.h"
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <esp_cache.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>
#include <hal/efuse_hal.h>

extern volatile uint32_t g_ws_dsi_frames; ///< HIL diag: DPI refreshes done

/// HIL diag: log a bring-up step and flush the CDC console before a risky
/// hardware call, so the last step survives a sudden power loss
#define DSI_STEP(msg)                                                          \
  do {                                                                         \
    WS_PRINTER.println("[dsi] " msg);                                          \
    WS_PRINTER.flush();                                                        \
    delay(20);                                                                 \
  } while (0)

/*!
    @brief  HIL diag: counts completed DPI refreshes (ISR context).
    @param  panel  Unused panel handle.
    @param  edata  Unused event data.
    @param  ctx    Unused user context.
    @return false, no higher-priority task woken.
*/
static bool IRAM_ATTR dsiRefreshDone(esp_lcd_panel_handle_t panel,
                                     esp_lcd_dpi_panel_event_data_t *edata,
                                     void *ctx) {
  (void)panel;
  (void)edata;
  (void)ctx;
  g_ws_dsi_frames = g_ws_dsi_frames + 1;
  return false;
}

#define DSI_STATUSBAR_HEIGHT 40      ///< Status bar height in pixels
#define DSI_STATUSBAR_ICON_SZ 16     ///< Status bar icon size in pixels
#define DSI_STATUSBAR_ICON_SPACING 8 ///< Spacing between icons
#define DSI_STATUSBAR_ICON_MARGIN 10 ///< Margin from edge of display

#define DSI_TD2_MCU_ADDR 0x45      ///< Touch Display 2 MCU I2C address
#define DSI_TD2_MCU_REG_ID 0x01    ///< MCU panel ID register
#define DSI_TD2_MCU_REG_RESET 0x02 ///< MCU panel/touch reset register
#define DSI_TD2_MCU_REG_BL 0x03    ///< MCU backlight register
#define DSI_TD2_RESET_ASSERT 0     ///< Hold LCD and touch in reset
#define DSI_TD2_RESET_RELEASE 3    ///< Release LCD and touch resets
#define DSI_TD2_BL_ON 0x84 ///< Backlight enable + brightness 4/31 (USB-safe)

#define DSI_PHY_LDO_CHAN 3      ///< LDO channel feeding VDD_MIPI_DPHY
#define DSI_PHY_LDO_MV 2500     ///< MIPI D-PHY supply voltage
#define DSI_ILI9881C_PAGE 0xFF  ///< ILI9881C command page select register
#define DSI_CMD_TEAR_ON 0x35    ///< DCS tearing effect line on
#define DSI_CMD_SLEEP_OUT 0x11  ///< DCS exit sleep mode
#define DSI_CMD_DISPLAY_ON 0x29 ///< DCS display on

/*!
    @brief  Adafruit_GFX target that renders straight into a DPI frame buffer
            (RGB888 stored as packed B,G,R bytes, native panel orientation).
            GFX colours are RGB565 and are expanded per pixel. Call flush()
            after drawing so the DMA sees the CPU cache contents.
*/
class DsiFramebufferGFX : public Adafruit_GFX {
public:
  /*!
      @brief  Constructor.
      @param  fb  RGB888 (B,G,R) frame buffer of w x h pixels.
      @param  w   Native panel width in pixels.
      @param  h   Native panel height in pixels.
  */
  DsiFramebufferGFX(uint8_t *fb, int16_t w, int16_t h)
      : Adafruit_GFX(w, h), _fb(fb) {}

  /*!
      @brief  Draws one pixel, honouring the GFX rotation.
      @param  x      X coordinate in the rotated space.
      @param  y      Y coordinate in the rotated space.
      @param  color  RGB565 colour.
  */
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || y < 0 || x >= width() || y >= height())
      return;
    int16_t t;
    switch (rotation) {
    case 1:
      t = x;
      x = WIDTH - 1 - y;
      y = t;
      break;
    case 2:
      x = WIDTH - 1 - x;
      y = HEIGHT - 1 - y;
      break;
    case 3:
      t = x;
      x = y;
      y = HEIGHT - 1 - t;
      break;
    }
    putPixel(_fb + ((int32_t)y * WIDTH + x) * 3, color);
  }

  /*!
      @brief  Fills the whole frame buffer with one colour.
      @param  color  RGB565 colour.
  */
  void fillScreen(uint16_t color) override {
    size_t n = (size_t)WIDTH * HEIGHT;
    if (color == 0x0000 || color == 0xFFFF) {
      memset(_fb, color ? 0xFF : 0x00, n * 3);
    } else {
      for (size_t i = 0; i < n; i++)
        putPixel(_fb + i * 3, color);
    }
  }

  /*!
      @brief  Writes the CPU cache back so the DPI DMA scans out the latest
              frame.
  */
  void flush() {
    esp_cache_msync(_fb, (size_t)WIDTH * HEIGHT * 3,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  }

private:
  /*!
      @brief  Stores an RGB565 colour as packed B,G,R bytes.
      @param  p      Destination pixel (3 bytes).
      @param  color  RGB565 colour.
  */
  static void putPixel(uint8_t *p, uint16_t color) {
    uint8_t r = (color >> 11) & 0x1F, g = (color >> 5) & 0x3F, b = color & 0x1F;
    p[0] = (b << 3) | (b >> 2);
    p[1] = (g << 2) | (g >> 4);
    p[2] = (r << 3) | (r >> 2);
  }

  uint8_t *_fb; ///< DPI frame buffer (B,G,R)
};

/*!
    @brief  Driver for MIPI DSI panels on the ESP32-P4 (Raspberry Pi Touch
            Display 2: ILI9881C, 720x1280, 2 data lanes).
*/
class dispDrvMipiDsi : public dispDrvBase {
public:
  /*!
      @brief  Constructor.
      @param  bus  DSI bus index (0 on the ESP32-P4).
  */
  dispDrvMipiDsi(uint32_t bus = 0) : dispDrvBase(), _bus(bus) {}

  /*!
      @brief  Destructor. Blanks the backlight and releases the DSI panel,
              bus and PHY supply.
  */
  ~dispDrvMipiDsi() {
    if (_display) {
      delete _display;
      _display = nullptr;
    }
    if (_mcu_found)
      writeMcu(DSI_TD2_MCU_REG_BL, 0);
    if (_panel)
      esp_lcd_panel_del(_panel);
    if (_io)
      esp_lcd_panel_io_del(_io);
    if (_dsi_bus)
      esp_lcd_del_dsi_bus(_dsi_bus);
    if (_phy_ldo)
      esp_ldo_release_channel(_phy_ldo);
  }

  /*!
      @brief  Brings up the Touch Display 2 MCU, the DSI link, the ILI9881C
              and DPI video, then clears the screen and enables the
              backlight.
      @return True if the panel is running, False otherwise.
  */
  bool begin() override {
    if (_bus != 0) {
      WS_DEBUG_PRINTLN("[display] ERROR: Only DSI bus 0 is supported!");
      return false;
    }
    // The panel MCU shares the main I2C bus (STEMMA QT); begin() is a no-op
    // when the bus is already running on these pins.
    DSI_STEP("I2C + panel MCU id");
    Wire.begin();
    if (!identifyMcu())
      return false;
    DSI_STEP("panel MCU: backlight off, reset assert");
    if (!writeMcu(DSI_TD2_MCU_REG_BL, 0) ||
        !writeMcu(DSI_TD2_MCU_REG_RESET, DSI_TD2_RESET_ASSERT)) {
      WS_DEBUG_PRINTLN("[display] ERROR: Panel MCU reset failed!");
      return false;
    }
    delay(20);

    esp_ldo_channel_config_t ldo_cfg = {};
    ldo_cfg.chan_id = DSI_PHY_LDO_CHAN;
    ldo_cfg.voltage_mv = DSI_PHY_LDO_MV;
    DSI_STEP("LDO: MIPI PHY supply on");
    if (!ok(esp_ldo_acquire_channel(&ldo_cfg, &_phy_ldo), "MIPI PHY power"))
      return false;

    esp_lcd_dsi_bus_config_t bus_cfg = {};
    bus_cfg.bus_id = (int)_bus;
    bus_cfg.num_data_lanes = 2;
    // Pre-v3 silicon (Metro rev A, v0.x/v1.x) needs the 20 MHz PHY PLL
    // reference; efuse revision is major * 100 + minor
    bus_cfg.phy_clk_src = efuse_hal_chip_revision() < 300
                              ? MIPI_DSI_PHY_PLLREF_CLK_SRC_DEFAULT_LEGACY
                              : MIPI_DSI_PHY_PLLREF_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = 1000;
    DSI_STEP("DSI bus + PHY PLL");
    if (!ok(esp_lcd_new_dsi_bus(&bus_cfg, &_dsi_bus), "DSI bus"))
      return false;

    esp_lcd_dbi_io_config_t io_cfg = {};
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    DSI_STEP("DBI command IO");
    if (!ok(esp_lcd_new_panel_io_dbi(_dsi_bus, &io_cfg, &_io),
            "DSI command interface"))
      return false;

    DSI_STEP("panel MCU: reset release");
    if (!writeMcu(DSI_TD2_MCU_REG_RESET, DSI_TD2_RESET_RELEASE)) {
      WS_DEBUG_PRINTLN("[display] ERROR: Panel MCU reset release failed!");
      return false;
    }
    delay(20);
    DSI_STEP("ILI9881C init table");
    if (!sendInit())
      return false;

    esp_lcd_dpi_panel_config_t dpi_cfg = {};
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.dpi_clock_freq_mhz = 69.444f;
    dpi_cfg.in_color_format = LCD_COLOR_FMT_RGB888;
    dpi_cfg.out_color_format = LCD_COLOR_FMT_RGB888;
    dpi_cfg.num_fbs = 1;
    dpi_cfg.video_timing.h_size = DSI_TD2_NATIVE_W;
    dpi_cfg.video_timing.v_size = DSI_TD2_NATIVE_H;
    dpi_cfg.video_timing.hsync_pulse_width = 33;
    dpi_cfg.video_timing.hsync_back_porch = 50;
    dpi_cfg.video_timing.hsync_front_porch = 239;
    dpi_cfg.video_timing.vsync_pulse_width = 2;
    dpi_cfg.video_timing.vsync_back_porch = 30;
    dpi_cfg.video_timing.vsync_front_porch = 20;
    DSI_STEP("DPI panel + frame buffer alloc");
    if (!ok(esp_lcd_new_panel_dpi(_dsi_bus, &dpi_cfg, &_panel), "DPI panel"))
      return false;
    // HIL diag: count refreshes for the heartbeat's frame rate
    esp_lcd_dpi_panel_event_callbacks_t cbs = {};
    cbs.on_refresh_done = dsiRefreshDone;
    ok(esp_lcd_dpi_panel_register_event_callbacks(_panel, &cbs, nullptr),
       "DPI refresh callback");

    void *fb = nullptr;
    if (!ok(esp_lcd_dpi_panel_get_frame_buffer(_panel, 1, &fb),
            "frame buffer") ||
        !fb)
      return false;
    _display = new DsiFramebufferGFX((uint8_t *)fb, DSI_TD2_NATIVE_W,
                                     DSI_TD2_NATIVE_H);
    if (!_display)
      return false;
    _display->setRotation(_rotation);
    _display->fillScreen(0x0000);
    _display->flush();
    DSI_STEP("DPI video start");
    if (!ok(esp_lcd_panel_init(_panel), "DPI video start"))
      return false;
    // Route the bridge's DPI stream from the frame buffer rather than the DSI
    // host's pattern generator (the panel otherwise shows a flat colour)
    DSI_STEP("DPI pattern off (frame buffer)");
    if (!ok(esp_lcd_dpi_panel_set_pattern(_panel, MIPI_DSI_PATTERN_NONE),
            "frame buffer mode"))
      return false;

    _display->setTextColor(0xFFFF);
    _display->setTextWrap(false);
    _display->setTextSize(_text_sz);
    DSI_STEP("panel MCU: backlight on");
    if (!writeMcu(DSI_TD2_MCU_REG_BL, DSI_TD2_BL_ON)) {
      WS_DEBUG_PRINTLN("[display] ERROR: Backlight enable failed!");
      return false;
    }
    DSI_STEP("streaming");
    WS_DEBUG_PRINTLN("[display] MIPI DSI panel initialized");
    return true;
  }

  /*!
      @brief  Draws the status bar and the Adafruit IO username.
      @param  io_username  Adafruit IO username to display.
  */
  void drawStatusBar(const char *io_username) override {
    if (!_display)
      return;
    _display->fillScreen(0x0000);
    _display->fillRect(0, 0, _display->width(), DSI_STATUSBAR_HEIGHT, 0xFFFF);
    _display->setTextSize(2);
    _display->setTextColor(0x0000);
    _display->setCursor(DSI_STATUSBAR_ICON_MARGIN, 12);
    _display->print(io_username);

    _statusbar_icons_y = (DSI_STATUSBAR_HEIGHT - DSI_STATUSBAR_ICON_SZ) / 2;
    _statusbar_icon_battery_x =
        _display->width() - DSI_STATUSBAR_ICON_SZ - DSI_STATUSBAR_ICON_MARGIN;
    _statusbar_icon_wifi_x = _statusbar_icon_battery_x - DSI_STATUSBAR_ICON_SZ -
                             DSI_STATUSBAR_ICON_SPACING;
    _statusbar_icon_cloud_x = _statusbar_icon_wifi_x - DSI_STATUSBAR_ICON_SZ -
                              DSI_STATUSBAR_ICON_SPACING;
    _display->drawBitmap(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         epd_bmp_cloud_online, DSI_STATUSBAR_ICON_SZ,
                         DSI_STATUSBAR_ICON_SZ, 0x0000);
    _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         epd_bmp_wifi_full, DSI_STATUSBAR_ICON_SZ,
                         DSI_STATUSBAR_ICON_SZ, 0x0000);
    _display->drawBitmap(_statusbar_icon_battery_x, _statusbar_icons_y,
                         epd_bmp_bat_full, DSI_STATUSBAR_ICON_SZ,
                         DSI_STATUSBAR_ICON_SZ, 0x0000);
    _display->setTextColor(0xFFFF);
    _display->setTextSize(_text_sz);
    _display->flush();
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
    bool update_rssi = abs(rssi - _statusbar_rssi) >= 3;
    bool update_mqtt = mqtt_status != _statusbar_mqtt_connected;
    if (!update_rssi && !update_mqtt)
      return;
    if (update_mqtt) {
      _display->fillRect(_statusbar_icon_cloud_x, _statusbar_icons_y,
                         DSI_STATUSBAR_ICON_SZ, DSI_STATUSBAR_ICON_SZ, 0xFFFF);
      _display->drawBitmap(
          _statusbar_icon_cloud_x, _statusbar_icons_y,
          mqtt_status ? epd_bmp_cloud_online : epd_bmp_cloud_offline,
          DSI_STATUSBAR_ICON_SZ, DSI_STATUSBAR_ICON_SZ, 0x0000);
      _statusbar_mqtt_connected = mqtt_status;
    }
    if (update_rssi) {
      const unsigned char *wifi_icon = epd_bmp_wifi_no_signal;
      if (rssi >= -50) {
        wifi_icon = epd_bmp_wifi_full;
      } else if (rssi >= -60) {
        wifi_icon = epd_bmp_wifi_fair;
      } else if (rssi >= -70) {
        wifi_icon = epd_bmp_wifi_weak;
      }
      _display->fillRect(_statusbar_icon_wifi_x, _statusbar_icons_y,
                         DSI_STATUSBAR_ICON_SZ, DSI_STATUSBAR_ICON_SZ, 0xFFFF);
      _display->drawBitmap(_statusbar_icon_wifi_x, _statusbar_icons_y,
                           wifi_icon, DSI_STATUSBAR_ICON_SZ,
                           DSI_STATUSBAR_ICON_SZ, 0x0000);
      _statusbar_rssi = rssi;
    }
    DSI_STEP("status bar: cache sync");
    _display->flush();
  }

  /*!
      @brief  Writes a message below the status bar.
      @param  message  Message text to render.
  */
  void writeMessage(const char *message) override {
    if (!_display)
      return;
    _display->fillRect(0, DSI_STATUSBAR_HEIGHT, _display->width(),
                       _display->height() - DSI_STATUSBAR_HEIGHT, 0x0000);
    int16_t y_idx = DSI_STATUSBAR_HEIGHT + 10;
    int16_t line_height = 8 * _text_sz;
    _display->setTextSize(_text_sz);
    _display->setCursor(0, y_idx);
    size_t msg_len = strlen(message);
    for (size_t i = 0; i < msg_len; i++) {
      if (y_idx + line_height > _display->height())
        break;
      char parsed_char = 0;
      bool is_newline = false;
      if (parseWriteToken(message, msg_len, i, parsed_char, is_newline,
                          char(247))) {
        if (is_newline) {
          y_idx += line_height;
          if (y_idx + line_height > _display->height())
            break;
          _display->setCursor(0, y_idx);
        } else {
          _display->write(parsed_char);
        }
        continue;
      }
      _display->print(message[i]);
    }
    DSI_STEP("message: cache sync");
    _display->flush();
  }

private:
  static const int16_t DSI_TD2_NATIVE_W = 720;  ///< Native panel width
  static const int16_t DSI_TD2_NATIVE_H = 1280; ///< Native panel height

  /*!
      @brief  Logs a failed ESP-IDF call.
      @param  err   Return code.
      @param  what  Step that was attempted.
      @return True if err is ESP_OK.
  */
  bool ok(esp_err_t err, const char *what) {
    if (err == ESP_OK)
      return true;
    WS_DEBUG_PRINT("[display] ERROR: DSI ");
    WS_DEBUG_PRINTVAR(what);
    WS_DEBUG_PRINT(" failed: ");
    WS_DEBUG_PRINTLNVAR(esp_err_to_name(err));
    WS_PRINTER.flush(); // HIL diag: get the error out before anything else
    delay(20);
    return false;
  }

  /*!
      @brief  Writes one register of the Touch Display 2 MCU.
      @param  reg    MCU register.
      @param  value  Value to write.
      @return True if the MCU acknowledged the write.
  */
  bool writeMcu(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(DSI_TD2_MCU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  /*!
      @brief  Reads the Touch Display 2 MCU ID and checks it is a 7" panel.
      @return True if a supported panel MCU answered.
  */
  bool identifyMcu() {
    // The MCU needs a STOP and ~5 ms before the register read
    Wire.beginTransmission(DSI_TD2_MCU_ADDR);
    Wire.write(DSI_TD2_MCU_REG_ID);
    if (Wire.endTransmission() != 0) {
      WS_DEBUG_PRINTLN("[display] ERROR: No Touch Display 2 MCU at 0x45, "
                       "check the DSI ribbon and 5V power!");
      return false;
    }
    delay(5);
    if (Wire.requestFrom((uint8_t)DSI_TD2_MCU_ADDR, (uint8_t)1) != 1) {
      WS_DEBUG_PRINTLN("[display] ERROR: Panel MCU ID read failed!");
      return false;
    }
    uint8_t id = Wire.read();
    WS_DEBUG_PRINT("[display] Panel MCU ID: 0x");
    WS_DEBUG_PRINTLNVAR(String(id, HEX));
    // Low nibble is the panel model: 1 and 4 are the 7" Touch Display 2
    uint8_t model = id & 0x0F;
    if (model != 1 && model != 4) {
      WS_DEBUG_PRINTLN("[display] ERROR: Unsupported panel, only the 7\" "
                       "Touch Display 2 is supported!");
      return false;
    }
    _mcu_found = true;
    return true;
  }

  /*!
      @brief  Selects an ILI9881C command page.
      @param  page  Page number.
      @return True on success.
  */
  bool selectPage(uint8_t page) {
    const uint8_t sel[3] = {0x98, 0x81, page};
    return ok(esp_lcd_panel_io_tx_param(_io, DSI_ILI9881C_PAGE, sel, 3),
              "page select");
  }

  /*!
      @brief  Sends the ILI9881C vendor init, then tear-on, sleep-out and
              display-on.
      @return True on success.
  */
  bool sendInit() {
    for (size_t i = 0;
         i < sizeof(ili9881c_init_rpi_td2) / sizeof(ili9881c_init_rpi_td2[0]);
         i++) {
      const ili9881c_init_reg_t *e = &ili9881c_init_rpi_td2[i];
      if (e->reg == DSI_ILI9881C_PAGE) {
        if (!selectPage(e->value))
          return false;
      } else if (!ok(esp_lcd_panel_io_tx_param(_io, e->reg, &e->value, 1),
                     "panel init write")) {
        return false;
      }
    }
    if (!selectPage(0))
      return false;
    const uint8_t tear_mode = 0;
    if (!ok(esp_lcd_panel_io_tx_param(_io, DSI_CMD_TEAR_ON, &tear_mode, 1),
            "tear on") ||
        !ok(esp_lcd_panel_io_tx_param(_io, DSI_CMD_SLEEP_OUT, nullptr, 0),
            "sleep out"))
      return false;
    delay(120);
    return ok(esp_lcd_panel_io_tx_param(_io, DSI_CMD_DISPLAY_ON, nullptr, 0),
              "display on");
  }

  uint32_t _bus;                               ///< DSI bus index
  bool _mcu_found = false;                     ///< Panel MCU answered
  esp_ldo_channel_handle_t _phy_ldo = nullptr; ///< MIPI PHY supply
  esp_lcd_dsi_bus_handle_t _dsi_bus = nullptr; ///< DSI bus
  esp_lcd_panel_io_handle_t _io = nullptr;     ///< DSI command (DBI) IO
  esp_lcd_panel_handle_t _panel = nullptr;     ///< DPI video panel
  DsiFramebufferGFX *_display = nullptr;       ///< Frame buffer renderer
};

#endif // WS_DISP_DRV_MIPI_DSI_H
