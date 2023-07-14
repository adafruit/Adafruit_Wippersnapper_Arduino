# Renames tinyusb's tusb_config.h to tusb_config.h_backup for PlatformIO build environments using TinyUSB
# https://github.com/platformio/platform-espressif32/issues/809#issuecomment-1132079120
import os

Import("env")

adafruit_usb_config = os.path.join(
    env.subst("$PROJECT_LIBDEPS_DIR"),
    env.subst("$PIOENV"),
    "Adafruit TinyUSB Library",
    "src",
    "tusb_config.h",
)

if os.path.isfile(adafruit_usb_config):
    os.rename(adafruit_usb_config, adafruit_usb_config + "_backup")