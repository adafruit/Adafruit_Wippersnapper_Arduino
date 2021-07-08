# Rules to build Wippersnapper UF2 files
#
# SPDX-License-Identifier: MIT

PROJECT_NAME          := adafruit-wippersnapper
PROJECT_VER_MAJOR     := 1
PROJECT_VER_MINOR     := 0
PROJECT_VER_PATCH     := 0
PROJECT_VER_BUILD     := BETA
PROJECT_VER_BUILD_NUM := 1

BOARD_PYPORTAL 		:= samd51-pyportal
BOARD_METRO_AIRLIFT := samd51-metro-airlift
BOARD_FUNHOUSE 		:= esp32s2-funhouse
BOARD_METRO_S2		:= esp32s2-metro
BOARD_MAGTAG_S2		:= esp32s2-magtag

# NOTE: path to "ci-arduino/build_platform.py" must be set on system prior to running

# Path to uf2conv utility, be sure to run `init submodule` prior to running this script!
UF2CONV = tools/uf2/utils/uf2conv.py

# SAMD51 - base address and familyID
UF2_BASE_SAMD51   := 0x4000
UF2_FAMILY_SAMD51 := 0x55114460

# ESP32-S2 - base address and familyID
UF2_BASE_ESP32S2   := 0x0000
UF2_FAMILY_ESP32S2 := 0xbfdd4eee

all: esp32s2 samd51
clean-all: clean-esp32s2 clean-samd51

## ESP32-S2 ##
esp32s2: esp32s2-funhouse esp32s2-metro esp32s2-magtag
clean-esp32s2: clean-esp32s2-funhouse clean-esp32s2-metro clean-esp32s2-magtag

esp32s2-magtag:
			mkdir -p build/$(BOARD_MAGTAG_S2)/
			arduino-cli compile --fqbn esp32:esp32:adafruit_magtag29_esp32s2:SerialMode=cdc,PSRAM=enabled,PartitionScheme=default,CPUFreq=240,FlashMode=qio,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,DebugLevel=none,USBStack=tinyusb -e  examples/Wippersnapper_demo/Wippersnapper_demo.ino
			python3 $(UF2CONV) examples/Wippersnapper_demo/build/esp32.esp32.adafruit_magtag29_esp32s2/Wippersnapper_demo.ino.bin -f $(UF2_FAMILY_ESP32S2) -b $(UF2_BASE_ESP32S2) -o build/esp32s2-magtag/$(PROJECT_NAME)-$(BOARD_MAGTAG_S2)-$(PROJECT_VER_MAJOR)-$(PROJECT_VER_MINOR)-$(PROJECT_VER_PATCH)-$(PROJECT_VER_BUILD)-$(PROJECT_VER_BUILD_NUM).uf2

clean-esp32s2-magtag:
			rm -r build/$(BOARD_MAGTAG_S2)/
			rm -r examples/Wippersnapper_demo/build/esp32.esp32.adafruit_magtag29_esp32s2

esp32s2-metro:
			mkdir -p build/$(BOARD_METRO_S2)/
			arduino-cli compile --fqbn esp32:esp32:adafruit_metro_esp32s2:SerialMode=cdc,PSRAM=enabled,PartitionScheme=default,CPUFreq=240,FlashMode=qio,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,DebugLevel=none,USBStack=tinyusb -e  examples/Wippersnapper_demo/Wippersnapper_demo.ino
			python3 $(UF2CONV) examples/Wippersnapper_demo/build/esp32.esp32.adafruit_metro_esp32s2/Wippersnapper_demo.ino.bin -f $(UF2_FAMILY_ESP32S2) -b $(UF2_BASE_ESP32S2) -o build/esp32s2-metro/$(PROJECT_NAME)-$(BOARD_METRO_S2)-$(PROJECT_VER_MAJOR)-$(PROJECT_VER_MINOR)-$(PROJECT_VER_PATCH)-$(PROJECT_VER_BUILD)-$(PROJECT_VER_BUILD_NUM).uf2

clean-esp32s2-metro:
			rm -r build/$(BOARD_METRO_S2)/
			rm -r examples/Wippersnapper_demo/build/esp32.esp32.adafruit_metro_esp32s2

esp32s2-funhouse:
			mkdir -p build/$(BOARD_FUNHOUSE)/
			arduino-cli compile --fqbn esp32:esp32:adafruit_funhouse_esp32s2:SerialMode=cdc,PSRAM=enabled,PartitionScheme=default,CPUFreq=240,FlashMode=qio,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,DebugLevel=none,USBStack=tinyusb -e  examples/Wippersnapper_demo/Wippersnapper_demo.ino
			python3 $(UF2CONV) examples/Wippersnapper_demo/build/esp32.esp32.adafruit_funhouse_esp32s2/Wippersnapper_demo.ino.bin -f $(UF2_FAMILY_ESP32S2) -b $(UF2_BASE_ESP32S2) -o build/esp32s2-funhouse/$(PROJECT_NAME)-$(BOARD_FUNHOUSE)-$(PROJECT_VER_MAJOR)-$(PROJECT_VER_MINOR)-$(PROJECT_VER_PATCH)-$(PROJECT_VER_BUILD)-$(PROJECT_VER_BUILD_NUM).uf2


esp32s2-funhouse:
			mkdir -p build/$(BOARD_FUNHOUSE)/
			arduino-cli compile --fqbn esp32:esp32:adafruit_funhouse_esp32s2:SerialMode=cdc,PSRAM=enabled,PartitionScheme=default,CPUFreq=240,FlashMode=qio,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,DebugLevel=none,USBStack=tinyusb -e  examples/Wippersnapper_demo/Wippersnapper_demo.ino
			python3 $(UF2CONV) examples/Wippersnapper_demo/build/esp32.esp32.adafruit_funhouse_esp32s2/Wippersnapper_demo.ino.bin -f $(UF2_FAMILY_ESP32S2) -b $(UF2_BASE_ESP32S2) -o build/esp32s2-funhouse/$(PROJECT_NAME)-$(BOARD_FUNHOUSE)-$(PROJECT_VER_MAJOR)-$(PROJECT_VER_MINOR)-$(PROJECT_VER_PATCH)-$(PROJECT_VER_BUILD)-$(PROJECT_VER_BUILD_NUM).uf2

clean-esp32s2-funhouse:
			rm -r build/$(BOARD_FUNHOUSE)/
			rm -r examples/Wippersnapper_demo/build/esp32.esp32.adafruit_funhouse_esp32s2

## SAMD51 ##
samd51: samd51-metro-airlift samd51-pyportal
clean-samd51: clean-samd51-metro-airlift clean-samd51-pyportal

samd51-metro-airlift:
			mkdir -p build/$(BOARD_METRO_AIRLIFT)/
			python3 $(BUILD_PLATFORM) metro_m4_airliftlite_tinyusb --export-binaries
			python3 $(UF2CONV) examples/Wippersnapper_demo/build/adafruit.samd.adafruit_metro_m4_airliftlite/Wippersnapper_demo.ino.bin --base $(UF2_BASE_SAMD51) --family $(UF2_FAMILY_SAMD51) -o build/samd51-metro-airlift/$(PROJECT_NAME)-$(BOARD_METRO_AIRLIFT)-$(PROJECT_VER_MAJOR)-$(PROJECT_VER_MINOR)-$(PROJECT_VER_PATCH)-$(PROJECT_VER_BUILD)-$(PROJECT_VER_BUILD_NUM).uf2

samd51-pyportal:
			mkdir -p build/$(BOARD_PYPORTAL)/
			python3 $(BUILD_PLATFORM) pyportal_tinyusb --export-binaries
			python3 $(UF2CONV) examples/Wippersnapper_demo/build/adafruit.samd.adafruit_pyportal_m4/Wippersnapper_demo.ino.bin --base $(UF2_BASE_SAMD51) --family $(UF2_FAMILY_SAMD51) -o build/samd51-pyportal/$(PROJECT_NAME)-$(BOARD_PYPORTAL)-$(PROJECT_VER_MAJOR)-$(PROJECT_VER_MINOR)-$(PROJECT_VER_PATCH)-$(PROJECT_VER_BUILD)-$(PROJECT_VER_BUILD_NUM).uf2

clean-samd51-metro-airlift:
			rm -r build/$(BOARD_METRO_AIRLIFT)/
			rm -r examples/Wippersnapper_demo/build/adafruit.samd.adafruit_metro_m4_airliftlite

clean-samd51-pyportal:
			rm -r build/$(BOARD_PYPORTAL)/
			rm -r examples/Wippersnapper_demo/build/adafruit.samd.adafruit_pyportal_m4

.PHONY: clean
clean:
			rm -r build/*
			rm -r examples/Wippersnapper_demo/build/*
			rm -r examples/wippersnapper-simpletest-esp8266/build/*
