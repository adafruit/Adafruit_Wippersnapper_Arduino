/*!
 * @file Wippersnapper_FS.cpp
 *
 * Wippersnapper TinyUSB Filesystem
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#if defined(ARDUINO_MAGTAG29_ESP32S2) || defined(ARDUINO_METRO_ESP32S2) ||     \
    defined(ARDUINO_METRO_ESP32S3) || defined(ARDUINO_FUNHOUSE_ESP32S2) ||     \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) ||                                    \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) ||                               \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) ||                                  \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT) ||                           \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM) ||                       \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) ||                          \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) ||                               \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT) ||                           \
    defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ESP32S3_DEV) ||            \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT) ||                        \
    defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2) ||                             \
    defined(ARDUINO_XIAO_ESP32S3) || defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350)

#include "Wippersnapper_FS.h"
#include "Wippersnapper_FS_format_policy.h"
#include "print_dependencies.h"
// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                           EXTERNAL_FLASH_USE_SPI);
#elif defined(ARDUINO_ARCH_ESP32)
// ESP32-S2 uses same flash device that stores code.
// Therefore there is no need to specify the SPI and SS
Adafruit_FlashTransport_ESP32 flashTransport;
#elif defined(ARDUINO_ARCH_RP2040)
// RP2040 use same flash device that store code.
// Therefore there is no need to specify the SPI and SS
// Use default (no-args) constructor to be compatible with CircuitPython
// partition scheme
Adafruit_FlashTransport_RP2040 flashTransport;
#else
#error No QSPI/SPI flash are defined on your board variant.h!
#endif

Adafruit_SPIFlash flash(&flashTransport); ///< SPIFlash object
FatVolume wipperFatFs; ///< File system object from Adafruit SDFat

Adafruit_USBD_MSC usb_msc; /*!< USB mass storage object */

FATFS elmchamFatfs;    ///< Elm Cham's fatfs object
uint8_t workbuf[4096]; ///< Working buffer for f_fdisk function.

bool makeFilesystem() {
  FRESULT r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
    return false;
  }
  return true;
}

bool setVolumeLabel() {
  // mount to set disk label
  FRESULT r = f_mount(&elmchamFatfs, "0:", 1);
  if (r != FR_OK) {
    return false;
  }
  // set label
  r = f_setlabel("WIPPER");
  if (r != FR_OK) {
    return false;
  }
  // unmount the fs
  f_unmount("0:");
  return true;
}

#ifdef ARDUINO_ARCH_ESP32
// NVS namespace + key for the "this board has had a working filesystem"
// marker. NVS lives in its own flash partition (untouched by f_mkfs) and is
// engineered to survive power loss, so it is the one place a "was provisioned"
// fact reliably outlives both a brownout and a FAT format.
static const char *kWSFsNvsNamespace = "ws_fs";
static const char *kWSFsProvisionedKey = "fs_ok";
// Boot-reason diagnostics, persisted in the same NVS namespace (survives a
// brownout and a FAT format). last_rst is updated every boot; fmt_rst/fmt_cnt
// capture the reset reason in effect when a (re)format actually ran, so a
// format that wiped data can be correlated with the reset that preceded it.
static const char *kWSFsLastResetKey = "last_rst"; ///< reset code this boot
static const char *kWSFsFormatResetKey = "fmt_rst"; ///< reset code at last format
static const char *kWSFsFormatCountKey = "fmt_cnt"; ///< number of (re)formats
#endif

/**************************************************************************/
/*!
    @brief    Returns whether this board has ever had a working filesystem
              mounted (i.e. it was previously provisioned). On the ESP32 this
              is persisted in NVS, which survives both a brownout and a FAT
              format; other platforms have no equivalent off-filesystem store,
              so this always returns false there and the blank-volume check in
              isFlashSafeToFormat() is relied on instead.
    @returns  True if a working filesystem has previously been seen on this
              board, False otherwise (or if the marker can't be read).
*/
/**************************************************************************/
static bool wasFilesystemProvisioned() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences prefs;
  if (!prefs.begin(kWSFsNvsNamespace, /*readOnly=*/true))
    return false; // namespace doesn't exist yet -> never provisioned
  bool provisioned = prefs.getBool(kWSFsProvisionedKey, false);
  prefs.end();
  return provisioned;
#else
  return false;
#endif
}

/**************************************************************************/
/*!
    @brief    Records (once) that this board has a working filesystem, so a
              later mount failure caused by a brownout is never mistaken for a
              blank factory chip and silently reformatted. No-op on platforms
              without NVS.
*/
/**************************************************************************/
static void markFilesystemProvisioned() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences prefs;
  if (!prefs.begin(kWSFsNvsNamespace, /*readOnly=*/false))
    return;
  if (!prefs.getBool(kWSFsProvisionedKey, false))
    prefs.putBool(kWSFsProvisionedKey, true);
  prefs.end();
#endif
}

/**************************************************************************/
/*!
    @brief    Records the reset reason seen on this boot into NVS (last_rst),
              so the actual boot reason is available for diagnosis even when the
              board halts before the usual printDeviceInfo() reset-reason dump.
              No-op on platforms without NVS / reset-reason support.
*/
/**************************************************************************/
static void recordBootResetReason() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences prefs;
  if (!prefs.begin(kWSFsNvsNamespace, /*readOnly=*/false))
    return;
  prefs.putUChar(kWSFsLastResetKey, (uint8_t)getResetReasonCode(0));
  prefs.end();
#endif
}

/**************************************************************************/
/*!
    @brief    Records, into NVS, that a (re)format happened this boot and the
              reset reason in effect when it ran (fmt_rst), plus a running count
              (fmt_cnt). This is the "write the boot reason only after a format"
              diagnostic: a format that erased data can later be correlated with
              the reset that preceded it. No-op without NVS.
*/
/**************************************************************************/
static void recordFormatEvent() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences prefs;
  if (!prefs.begin(kWSFsNvsNamespace, /*readOnly=*/false))
    return;
  prefs.putUChar(kWSFsFormatResetKey, (uint8_t)getResetReasonCode(0));
  prefs.putUInt(kWSFsFormatCountKey, prefs.getUInt(kWSFsFormatCountKey, 0) + 1);
  prefs.end();
#endif
}

/**************************************************************************/
/*!
    @brief    Prints the persisted boot/format diagnostics (this boot's reset
              reason, and - if any format has ever run - the count and the reset
              reason at the last format) to the serial console. No-op without
              NVS.
*/
/**************************************************************************/
static void printFsDiagnostics() {
#ifdef ARDUINO_ARCH_ESP32
  Preferences prefs;
  if (!prefs.begin(kWSFsNvsNamespace, /*readOnly=*/true))
    return;
  uint8_t lastRst = prefs.getUChar(kWSFsLastResetKey, 0);
  uint8_t fmtRst = prefs.getUChar(kWSFsFormatResetKey, 0);
  uint32_t fmtCnt = prefs.getUInt(kWSFsFormatCountKey, 0);
  prefs.end();

  WS_DEBUG_PRINT("[fs] boot reset reason: ");
  WS_DEBUG_PRINTVAR((int)lastRst);
  WS_DEBUG_PRINT(" (");
  WS_DEBUG_PRINTVAR(getResetReasonStr(lastRst));
  WS_DEBUG_PRINTLN(")");
  if (fmtCnt > 0) {
    WS_DEBUG_PRINT("[fs] filesystem (re)formats so far: ");
    WS_DEBUG_PRINTLNVAR(fmtCnt);
    WS_DEBUG_PRINT("[fs] reset reason at last format: ");
    WS_DEBUG_PRINTVAR((int)fmtRst);
    WS_DEBUG_PRINT(" (");
    WS_DEBUG_PRINTVAR(getResetReasonStr(fmtRst));
    WS_DEBUG_PRINTLN(")");
  }
#endif
}

/**************************************************************************/
/*!
    @brief    Decides whether it is safe to (re)create the filesystem after a
              mount failure, i.e. whether the failure looks like a genuinely
              blank FAT user partition (safe to format) rather than a flash
              chip whose reads are momentarily unstable due to a voltage sag /
              brownout, or a volume holding data that simply won't mount (must
              NOT be formatted - that would erase the user's data on recovery).
              A brownout is frequently NOT reported as a brownout reset reason,
              so we cannot rely on the reset reason alone before doing
              something destructive.

              This is the secondary gate, only consulted on a genuine first
              boot (no NVS "provisioned" marker - see wasFilesystemProvisioned).
              It (a) confirms the flash chip is actually responding, (b) on the
              ESP32 confirms a known-good off-filesystem structure (NVS, located
              via the partition API rather than a hardcoded offset) reads back
              sanely, and (c) on every platform requires the FAT volume's own
              boot sector to read back genuinely erased (all 0xFF) before
              allowing a format.
    @returns  True if a (re)format is safe, False if the flash is unreadable /
              unstable / not genuinely blank, in which case it must NOT be
              formatted.
*/
/**************************************************************************/
bool isFlashSafeToFormat() {
  // (a) Confirm a real flash chip is responding - a dead/disconnected or
  // browning-out SPI bus often floats high and reads back as all 0xFF, which
  // would otherwise be mistaken for a genuinely-erased (blank) chip.
  uint32_t jedecID = flash.getJEDECID();
  if (jedecID == 0 || jedecID == 0xFFFFFF)
    return false;

#ifdef ARDUINO_ARCH_ESP32
  // (b) Locate the NVS partition via the partition API (no hardcoded offset)
  // and confirm its first page header is a valid NVS page state: UNINITIALIZED
  // 0xFFFFFFFF / ACTIVE 0xFFFFFFFE / FULL 0xFFFFFFFC / FREEING 0xFFFFFFF8. The
  // high byte is always 0xFF; the low byte distinguishes the state. Garbage
  // from a browning-out SPI bus is very unlikely to match this.
  const esp_partition_t *nvs = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
  if (nvs == NULL)
    return false; // can't locate NVS -> don't trust the flash -> don't format
  uint8_t hdr[2];
  if (esp_partition_read(nvs, 0, hdr, sizeof(hdr)) != ESP_OK)
    return false;
  if (hdr[1] != 0xFF ||
      (hdr[0] != 0xFF && hdr[0] != 0xFE && hdr[0] != 0xFC && hdr[0] != 0xF8))
    return false;
#endif

  // (c) A (re)format is only appropriate when the FAT volume's boot sector is
  // genuinely erased (all 0xFF). Any other unmountable content is treated as
  // possible voltage/brownout corruption and must NOT be erased. SPIFlash
  // reads are relative to the FAT partition, so offset 0 is the boot sector.
  uint8_t sector[256];
  for (uint32_t addr = 0; addr < 512; addr += sizeof(sector)) {
    if (flash.readBuffer(addr, sector, sizeof(sector)) != sizeof(sector))
      return false; // could not read the volume -> unsafe to format
    for (uint32_t i = 0; i < sizeof(sector); i++) {
      if (sector[i] != 0xFF)
        return false; // non-erased content -> not blank -> refuse to format
    }
  }
  return true; // chip healthy + boot sector fully erased -> safe to format
}

/**************************************************************************/
/*!
    @brief    Initializes USB-MSC and the QSPI flash filesystem.
*/
/**************************************************************************/
Wippersnapper_FS::Wippersnapper_FS() {
#if PRINT_DEPENDENCIES
  WS_DEBUG_PRINTLN("Build Dependencies:");
  WS_DEBUG_PRINTLN("*********************");
  WS_DEBUG_PRINTLN(project_dependencies);
  WS_DEBUG_PRINTLN("*********************");
  WS_PRINTER.flush();
  delay(50); // give host a chance to finish reading serial buffer
#endif
  // Detach USB device during init.
  TinyUSBDevice.detach();
  // Wait for detach
  delay(500);

  // Persist this boot's reset reason early, before any mount/format decision,
  // so the actual boot reason is recoverable for diagnosis even if we halt here
  // (the usual printDeviceInfo() reset-reason dump runs later, in connect()).
  recordBootResetReason();

  // Attempt to mount the existing filesystem WITHOUT formatting. initFilesystem
  // (with force_format=false) no longer creates a filesystem on a mount
  // failure - it returns false and lets the gates below decide - so a
  // brownout-corrupted FAT is never silently wiped.
  if (!initFilesystem()) {
    // Mount failed. A battery sag / brownout can momentarily corrupt SPI flash
    // reads - often WITHOUT being reported as a brownout reset - so give the
    // power rail a moment to settle and retry before doing anything
    // destructive.
    delay(50); // let power stabilise after failure
    if (!initFilesystem()) {
      // Still unmountable. Decide - very carefully - whether to (re)format.
      // The gating order lives in decideFsAction() (pure + unit-tested):
      //   - previously provisioned  -> NEVER format (corruption/brownout)
      //   - genuine first boot      -> format only if the flash is healthy and
      //                                the volume is genuinely blank.
      bool provisioned = wasFilesystemProvisioned();
      // Only probe the flash when a format is actually on the table - on a
      // provisioned board we already know we must not format, so we skip the
      // reads to conserve a possibly-dying battery.
      bool safeToFormat = provisioned ? false : isFlashSafeToFormat();

      switch (decideFsAction(/*mounted=*/false, provisioned, safeToFormat)) {
      case WsFsAction::HaltProvisionedBrownout:
        // Re-attach USB so the halt reason is visible over serial (CDC) and the
        // board is not a silent black box. We deliberately do NOT bring up USB
        // mass storage here: the FAT did not mount, so exposing it to the host
        // could trigger a "repair/initialize" prompt that wipes the very
        // secrets.json we are protecting. fsHalt() leaves the status LED solid
        // as the on-device indicator.
        TinyUSBDevice.attach();
        fsHalt("Filesystem unreadable but this board was previously "
               "provisioned - likely a brownout or power-loss recovery. "
               "Recharge and reset; refusing to reformat to protect your "
               "secrets.json.");
        break;
      case WsFsAction::HaltUnsafeToFormat:
        // Never-provisioned board whose flash looks unstable / not-blank.
        // Surface the reason over serial (CDC) and halt without formatting.
        TinyUSBDevice.attach();
        fsHalt("Flash reads unstable (possible brownout) - refusing to "
               "format. Recharge/repower the board and reset.");
        break;
      case WsFsAction::Format:
        // Flash is healthy and genuinely blank: safe to create a new
        // filesystem. [DESTRUCTIVE]
        if (!initFilesystem(true)) {
          TinyUSBDevice.attach();
          setStatusLEDColor(RED);
          fsHalt("ERROR Initializing Filesystem");
        }
        // Re-validate the flash chip responds after the (destructive) format -
        // a format that "succeeded" against a flaky chip would otherwise go
        // unnoticed.
        {
          uint32_t postFormatJedecID = flash.getJEDECID();
          if (postFormatJedecID == 0 || postFormatJedecID == 0xFFFFFF) {
            fsHalt("Flash unreadable immediately after format - aborting. "
                   "Recharge/repower the board and reset.");
          }
        }
        // Persist that a (re)format ran this boot and the reset reason behind
        // it, so a format that wiped data can be correlated with its cause.
        recordFormatEvent();
        break;
      case WsFsAction::Mounted:
        break; // unreachable: mount already failed above
      }
    }
  }

  // Report the persisted boot/format diagnostics (this boot's reset reason,
  // and any prior format + the reset reason that preceded it).
  printFsDiagnostics();

  // Initialize USB-MSD
  initUSBMSC();

  // If we created a new filesystem, halt until user RESETs device.
  if (_freshFS) {
    WS_DEBUG_PRINTLN("New filesystem created! Resetting the board shortly...");
    WS.enableWDT(500);
    while (1) {
      delay(1000);
    }
  }
}

/************************************************************/
/*!
    @brief    Filesystem destructor
*/
/************************************************************/
Wippersnapper_FS::~Wippersnapper_FS() {}

/**************************************************************************/
/*!
    @brief    Initializes the flash filesystem.
    @param    force_format
                If true, forces a new filesystem to be created. [DESTRUCTIVE]
    @return   True if filesystem initialized correctly, false otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::initFilesystem(bool force_format) {
  // Init. flash library
  if (!flash.begin())
    return false;

  // Try to mount the existing filesystem.
  bool mounted = wipperFatFs.begin(&flash);

  if (!mounted && !force_format) {
    // Mount failed and the caller did NOT request a format. Do NOT create a
    // filesystem here - that would erase a brownout-corrupted FAT and the
    // user's secrets.json with it. Report the failure and let the caller (the
    // constructor's guarded format path) decide whether a format is safe.
    return false;
  }

  if (force_format || !mounted) {
    // Create a new filesystem.
    // NOTE: THIS WILL ERASE ALL DATA ON THE FLASH
    if (!makeFilesystem())
      return false;

    // set volume label
    if (!setVolumeLabel())
      return false;

    // sync all data to flash
    flash.syncBlocks();

    _freshFS = true;
  }

  // Check new FS
  if (!wipperFatFs.begin(&flash))
    return false;

  // TODO: Don't do this unless we need the space and createSecrets fails
  //  If CircuitPython was previously installed - erase CPY FS
  eraseCPFS();
  // Also, should probably relabel drive to WIPPER if CIRCUITPY was there
  // using setVolumeLabel(), but note the FS must be unmounted first

  // No file indexing on macOS
  if (!wipperFatFs.exists("/.fseventsd/no_log")) {
    wipperFatFs.mkdir("/.fseventsd/");
    File32 writeFile = wipperFatFs.open("/.fseventsd/no_log", FILE_WRITE);
    if (!writeFile)
      return false;
    writeFile.close();
  }

  if (!wipperFatFs.exists("/.metadata_never_index")) {
    File32 writeFile = wipperFatFs.open("/.metadata_never_index", FILE_WRITE);
    if (!writeFile)
      return false;
    writeFile.close();
  }
  if (!wipperFatFs.exists("/.Trashes")) {
    File32 writeFile = wipperFatFs.open("/.Trashes", FILE_WRITE);
    if (!writeFile)
      return false;
    writeFile.close();
  }

  // Create wippersnapper_boot_out.txt file
  if (!createBootFile() && !WS.brownOutCausedReset)
    return false;

  // Check if secrets.json file already exists
  if (!configFileExists()) {
    // Create new secrets.json file and halt
    createSecretsFile();
  }

  // We have a working, mountable filesystem that we did NOT just create this
  // boot. Persist that fact so a future mount failure (e.g. a brownout
  // corrupting the FAT) is recognised as corruption and never silently
  // reformatted. On a fresh format this point is not reached: createSecretsFile
  // above halts the board until the user edits secrets.json and resets, after
  // which the next boot mounts the now-real FS and records the marker here.
  if (!_freshFS) {
    markFilesystemProvisioned();
  }

  return true;
}

/**************************************************************************/
/*!
    @brief    Initializes the USB MSC device.
*/
/**************************************************************************/
void Wippersnapper_FS::initUSBMSC() {
  // Set disk vendor id, product id and revision with string up to 8, 16, 4
  // characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  // Set callback
  usb_msc.setReadWriteCallback(qspi_msc_read_cb, qspi_msc_write_cb,
                               qspi_msc_flush_cb);

  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.pageSize() * flash.numPages() / 512, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  // init MSC
  usb_msc.begin();

  // Attach MSC and wait for enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
  }
  TinyUSBDevice.attach();
  delay(500);
}

/**************************************************************************/
/*!
    @brief    Checks if secrets.json file exists on the flash filesystem.
    @returns  True if secrets.json file exists, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_FS::configFileExists() {
  // Does secrets.json file exist?
  if (!wipperFatFs.exists("/secrets.json"))
    return false;
  File32 file = wipperFatFs.open("/secrets.json", FILE_READ);
  if (!file)
    return false;
  int firstChar = file.peek();
  file.close();
  if (firstChar <= 0 || firstChar == 255)
    return false;
  return true;
}

/**************************************************************************/
/*!
    @brief    Erases the default CircuitPython filesystem if it exists.
*/
/**************************************************************************/
void Wippersnapper_FS::eraseCPFS() {
  if (WS.brownOutCausedReset) {
    return; // Can't serial print here, in next PR check we're not out of space
  }
  if (wipperFatFs.exists("/boot_out.txt")) {
    wipperFatFs.remove("/boot_out.txt");
    wipperFatFs.remove("/code.py");
    File32 libDir = wipperFatFs.open("/lib");
    libDir.rmRfStar();
  }
}

/**************************************************************************/
/*!
    @brief    Erases the existing "wipper_boot_out.txt" file from the FS.
*/
/**************************************************************************/
void Wippersnapper_FS::eraseBootFile() {
  // overwrite previous boot_out file on each boot
  if (wipperFatFs.exists("/wipper_boot_out.txt")) {
    wipperFatFs.remove("/wipper_boot_out.txt");
  }
}

/**************************************************************************/
/*!
    @brief    Creates or overwrites `wipper_boot_out.txt` file to FS.
*/
/**************************************************************************/
bool Wippersnapper_FS::createBootFile() {
  bool is_success = false;
  char sMAC[18] = {0};
  String newContent;

  // Generate new content
  newContent += "Adafruit.io WipperSnapper\n";
  newContent += "WipperSnapper Firmware Version: " + String(WS_VERSION) + "\n";
  newContent += "Board ID: " + String(BOARD_ID) + "\n";

#if defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(USE_AIRLIFT) ||            \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350)
  newContent +=
      "AirLift Coprocessor Firmware Version: " + String(WS._airlift_version) +
      "\n";
#endif

  sprintf(sMAC, "%02X:%02X:%02X:%02X:%02X:%02X", WS._macAddr[0], WS._macAddr[1],
          WS._macAddr[2], WS._macAddr[3], WS._macAddr[4], WS._macAddr[5]);
  newContent += "MAC Address: " + String(sMAC) + "\n";

#if PRINT_DEPENDENCIES
  newContent += ("Build dependencies:\n");
  newContent += (project_dependencies);
  newContent += ("\n");
#endif

// Print ESP-specific info to boot file
#ifdef ARDUINO_ARCH_ESP32
  newContent += "ESP-IDF Version: " + String(ESP.getSdkVersion()) + "\n";
  newContent += "ESP32 Core Version: " + String(ESP.getCoreVersion()) + "\n";
#endif

  // Check if the file exists and read its content
  File32 bootFile = wipperFatFs.open("/wipper_boot_out.txt", FILE_READ);
  if (bootFile) {
    String existingContent;
    while (bootFile.available()) {
      existingContent += char(bootFile.read());
    }
    bootFile.close();

    // Compare existing content with new content
    if (existingContent == newContent) {
      WS_DEBUG_PRINTLN("INFO: wipper_boot_out.txt already exists with the "
                       "same content.");
      return true; // No need to overwrite
    } else {
      WS_DEBUG_PRINTLN("INFO: wipper_boot_out.txt exists but with different "
                       "content. Overwriting...");
    }
  } else {
    WS_DEBUG_PRINTLN("INFO: could not open wipper_boot_out.txt for reading. "
                     "Creating new file...");
  }

  if (WS.brownOutCausedReset) {
    return false;
  }

  // Overwrite the file with new content
  bootFile = wipperFatFs.open("/wipper_boot_out.txt", FILE_WRITE);
  if (bootFile) {
    bootFile.print(newContent);
    bootFile.flush();
    bootFile.close();
    is_success = true;
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to open wipper_boot_out.txt for writing!");
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief    Creates a default secrets.json file on the filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::createSecretsFile() {
  // Open file for writing
  File32 secretsFile = wipperFatFs.open("/secrets.json", FILE_WRITE);
  secretsFile.truncate(0);
  // Create a default secretsConfig structure
  secretsConfig secretsConfig;
  strcpy(secretsConfig.aio_user, "YOUR_IO_USERNAME_HERE");
  strcpy(secretsConfig.aio_key, "YOUR_IO_KEY_HERE");
  strcpy(secretsConfig.network.ssid, "YOUR_WIFI_SSID_HERE");
  strcpy(secretsConfig.network.pass, "YOUR_WIFI_PASS_HERE");
  secretsConfig.status_pixel_brightness = STATUS_PIXEL_BRIGHTNESS_DEFAULT;

  // Serialize the struct to a JSON document
  JsonDocument doc;
  doc.set(secretsConfig);
  serializeJsonPretty(doc, secretsFile);
  secretsFile.flush();
  secretsFile.close();

  // Signal to user that action must be taken (edit secrets.json)
  writeToBootOut(
      "ERROR: Please edit the secrets.json file. Then, reset your board.\n");
  delay(500);   // previously 2500
  initUSBMSC(); // re-init USB MSC to show new file to user for editing
  fsHalt("ERROR: Please edit the secrets.json file. Then, reset your board.");
}

/**************************************************************************/
/*!
    @brief    Parses a secrets.json file on the flash filesystem.
*/
/**************************************************************************/
void Wippersnapper_FS::parseSecrets() {
  // Attempt to open the secrets.json file for reading
  File32 secretsFile = wipperFatFs.open("/secrets.json");
  if (!secretsFile) {
    fsHalt("ERROR: Could not open secrets.json file for reading!");
  }

  // Attempt to deserialize the file's JSON document
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, secretsFile);
  if (error == DeserializationError::EmptyInput) {
    // An empty/blank read of secrets.json on a previously-provisioned board is
    // almost never a genuinely empty file - it is the same failed-flash-read a
    // brownout produces (the SPI rail sags during power-loss recovery and bytes
    // come back as 0x00/0xFF, so a perfectly intact file deserializes as
    // EmptyInput). Recreating it from the template here REMOVES the user's real
    // credentials - observed on hardware: a drained MagTag came up, read its
    // intact secrets.json back empty, and this path replaced it with a
    // placeholder, leaving the real file orphaned on flash.
    //
    // Gate the destructive recreate on the SAME persistent NVS provisioned
    // marker #937 uses for the format decision - NOT on WS.brownOutCausedReset.
    // The reset reason is unreliable as a safety gate: a brownout-corrupted
    // read routinely surfaces on a boot that reports POWERON_RESET, not
    // brownout (15), so the brownout flag is false exactly when we most need to
    // protect. A provisioned board that suddenly reads empty must
    // halt-and-protect, never recreate; only a board that has never had a
    // working FS may bootstrap one.
    if (wasFilesystemProvisioned()) {
      writeToBootOut("ERROR: secrets.json read back empty on a provisioned "
                     "board - refusing to recreate (likely a "
                     "brownout/power-loss misread). Recharge and reset.\n");
      fsHalt("ERROR: secrets.json read back empty on a previously-provisioned "
             "board - refusing to recreate it and erase your credentials. This "
             "is almost always a brownout/power-loss misread of an intact "
             "file. Recharge and reset; edit secrets.json by hand only if it "
             "persists.");
    } else if (WS.brownOutCausedReset) {
      fsHalt("ERROR: Empty secrets.json file, can't recreate due to brownout - "
             "recharge or must be fixed manually.");
    } else {
      // Never-provisioned board: genuinely bootstrapping a template is correct.
      // TODO: Can't serial print here, in next PR check we're not out of space
      WS_DEBUG_PRINTLN("ERROR: Empty secrets.json file, recreating...");
      secretsFile.close();
      wipperFatFs.remove("/secrets.json");
      createSecretsFile(); // calls fsHalt
    }
  } else if (error) {
    fsHalt(String("ERROR: Unable to parse secrets.json file - "
                  "deserializeJson() failed with code") +
           error.c_str());
  }

  if (doc.containsKey("network_type_wifi")) {
    // set default network config
    convertFromJson(doc["network_type_wifi"], WS._config.network);

    if (!doc["network_type_wifi"].containsKey("alternative_networks")) {
      // do nothing extra, we already have the only network
      WS_DEBUG_PRINTLN("Found single wifi network in secrets.json");

    } else if (doc["network_type_wifi"]["alternative_networks"]
                   .is<JsonArray>()) {

      WS_DEBUG_PRINTLN("Found multiple wifi networks in secrets.json");
      // Parse network credentials from array in secrets
      JsonArray altnetworks = doc["network_type_wifi"]["alternative_networks"];
      int8_t altNetworkCount = (int8_t)altnetworks.size();
      WS_DEBUG_PRINT("Network count: ");
      WS_DEBUG_PRINTLN(altNetworkCount);
      if (altNetworkCount == 0) {
        fsHalt("ERROR: No alternative network entries found under "
               "network_type_wifi.alternative_networks in secrets.json!");
      }
      // check if over 3, warn user and take first three
      for (int i = 0; i < altNetworkCount; i++) {
        if (i >= 3) {
          WS_DEBUG_PRINT("WARNING: More than 3 networks in secrets.json, "
                         "only the first 3 will be used. Not using ");
          WS_DEBUG_PRINTLN(altnetworks[i]["network_ssid"].as<const char *>());
          break;
        }
        convertFromJson(altnetworks[i], WS._multiNetworks[i]);
        WS_DEBUG_PRINT("Added SSID: ");
        WS_DEBUG_PRINTLN(WS._multiNetworks[i].ssid);
        WS_DEBUG_PRINT("PASS: ");
        WS_DEBUG_PRINTLN(WS._multiNetworks[i].pass);
      }
      WS._isWiFiMulti = true;
    } else {
      fsHalt("ERROR: Unrecognised value type for "
             "network_type_wifi.alternative_networks in secrets.json!");
    }
  } else {
    fsHalt("ERROR: Could not find network_type_wifi in secrets.json!");
  }

  // Extract a config struct from the JSON document
  WS._config = doc.as<secretsConfig>();

  // Validate the config struct is not filled with default values
  if (strcmp(WS._config.aio_user, "YOUR_IO_USERNAME_HERE") == 0 ||
      strcmp(WS._config.aio_key, "YOUR_IO_KEY_HERE") == 0) {
    writeToBootOut(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!\n");

    fsHalt(
        "ERROR: Invalid IO credentials in secrets.json! TO FIX: Please change "
        "io_username and io_key to match your Adafruit IO credentials!");
  }

  if (strcmp(WS._config.network.ssid, "YOUR_WIFI_SSID_HERE") == 0 ||
      strcmp(WS._config.network.pass, "YOUR_WIFI_PASS_HERE") == 0) {
    writeToBootOut("ERROR: Invalid network credentials in secrets.json! TO "
                   "FIX: Please change network_ssid and network_password to "
                   "match your Adafruit IO credentials!\n");

    fsHalt("ERROR: Invalid network credentials in secrets.json! TO FIX: Please "
           "change network_ssid and network_password to match your Adafruit IO "
           "credentials!");
  }

  // specify type of value for json key, by using the |operator to include
  // a typed default value equivalent of with .as<float> w/ default value
  // https://arduinojson.org/v7/api/jsonvariant/or/
  WS._config.status_pixel_brightness =
      doc["status_pixel_brightness"] | (float)STATUS_PIXEL_BRIGHTNESS_DEFAULT;

  // Close secrets.json file
  secretsFile.close();
}

/**************************************************************************/
/*!
    @brief    Appends message string to wipper_boot_out.txt file.
    @param    str
                PROGMEM string.
*/
/**************************************************************************/
void Wippersnapper_FS::writeToBootOut(PGM_P str) {
  // Append error output to FS
  File32 bootFile = wipperFatFs.open("/wipper_boot_out.txt", FILE_WRITE);
  if (bootFile) {
    bootFile.print(str);
    bootFile.flush();
    bootFile.close();
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to open wipper_boot_out.txt for logging!");
    // feels like we should check why, if good use-case ok, otherwise fsHalt
    // as indicates fs corruption or disc access issue (maybe latter is okay)
  }
}

/**************************************************************************/
/*!
    @brief    Halts execution and blinks the status LEDs yellow.
    @param    msg
                Error message to print to serial console.
*/
/**************************************************************************/
void Wippersnapper_FS::fsHalt(String msg) {
  statusLEDSolid(WS_LED_STATUS_FS_WRITE);
#ifdef ARDUINO_ARCH_ESP32
  // Capture the actual reset reason once, so the halt reports the observed boot
  // reason (e.g. "12 (SW_CPU_RESET)") rather than only inferring "likely a
  // brownout" - a brownout is frequently not reported as a brownout reset.
  int rstCode = getResetReasonCode(0);
  const char *rstStr = getResetReasonStr(rstCode);
#endif
  while (1) {
    WS_DEBUG_PRINTLN("Fatal Error: Halted execution!");
    WS_DEBUG_PRINTLN(msg.c_str());
#ifdef ARDUINO_ARCH_ESP32
    WS_DEBUG_PRINT("Boot/reset reason: ");
    WS_DEBUG_PRINTVAR(rstCode);
    WS_DEBUG_PRINT(" (");
    WS_DEBUG_PRINTVAR(rstStr);
    WS_DEBUG_PRINTLN(")");
#endif
    delay(1000);
    yield();
  }
}

/**************************************************************************/
/*!
    @brief    Callback invoked when received READ10 command. Copies disk's
              data up to buffer.
    @param    lba
                Logical address of first block to read.
    @param    buffer
                Desired buffer to read from.
    @param    bufsize
                Desired size of buffer to read.
    @returns  Number of written bytes (must be multiple of block size)
*/
/**************************************************************************/
int32_t qspi_msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

/**************************************************************************/
/*!
    @brief    Callback invoked when received WRITE command. Process data
              in buffer to disk's storage.
    @param    lba
                Logical address of first block to write.
    @param    buffer
                Desired buffer to write to.
    @param    bufsize
                Desired size of buffer to write.
    @returns  Number of written bytes (must be multiple of block size)
*/
/**************************************************************************/
int32_t qspi_msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

/***************************************************************************/
/*!
    @brief  Callback invoked when WRITE10 command is completed (status
            received and accepted by host). Used to flush any pending cache.
*/
/***************************************************************************/
void qspi_msc_flush_cb(void) {
  // sync w/flash
  flash.syncBlocks();
  // clear file system's cache to force refresh
  wipperFatFs.cacheClear();
}

//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C" {

DSTATUS disk_status(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

DSTATUS disk_initialize(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

DRESULT disk_read(BYTE pdrv,  /* Physical drive nmuber to identify the drive */
                  BYTE *buff, /* Data buffer to store read data */
                  DWORD sector, /* Start sector in LBA */
                  UINT count    /* Number of sectors to read */
) {
  (void)pdrv;
  return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, /* Physical drive nmuber to identify the drive */
                   const BYTE *buff, /* Data to be written */
                   DWORD sector,     /* Start sector in LBA */
                   UINT count        /* Number of sectors to write */
) {
  (void)pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, /* Physical drive nmuber (0..) */
                   BYTE cmd,  /* Control code */
                   void *buff /* Buffer to send/receive control data */
) {
  (void)pdrv;

  switch (cmd) {
  case CTRL_SYNC:
    flash.syncBlocks();
    return RES_OK;

  case GET_SECTOR_COUNT:
    *((DWORD *)buff) = flash.size() / 512;
    return RES_OK;

  case GET_SECTOR_SIZE:
    *((WORD *)buff) = 512;
    return RES_OK;

  case GET_BLOCK_SIZE:
    *((DWORD *)buff) = 8; // erase block size in units of sector size
    return RES_OK;

  default:
    return RES_PARERR;
  }
}
}

#endif // USE_TINYUSB