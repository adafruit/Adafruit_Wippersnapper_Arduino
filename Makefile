# Rules to build Wippersnapper UF2 files
#
# SPDX-License-Identifier: MIT

PROJECT_NAME := WIPPERSNAPPER
PROJECT_VER_MAJOR := 0
PROJECT_VER_MINOR := 1
PROJECT_VER_PATCH := 0

# Path to https://github.com/adafruit/ci-arduino/blob/master/build_platform.py
TEST_PLATFORMS = ~/Desktop/github_brentru/ci-arduino/build_platform.py

# Path to uf2conv, be sure to init submodule prior to running!
UF2CONV = tools/uf2/utils/uf2conv.py

# SAMD51 - base address and familyID
UF2_BASE_SAMD51   := 0x4000
UF2_FAMILY_SAMD51 := 0x55114460

samd51-metro-airlift:
			mkdir -p build/samd51-metro-airlift/
			python3 $(TEST_PLATFORMS) metro_m4_airliftlite_tinyusb --export-binaries
			python3 $(UF2CONV) examples/Wippersnapper_demo/build/adafruit.samd.adafruit_metro_m4_airliftlite/Wippersnapper_demo.ino.bin --base $(UF2_BASE_SAMD51) --family $(UF2_FAMILY_SAMD51) -o build/samd51-metro-airlift/$(PROJECT_NAME)-$(PROJECT_VER_MAJOR)-$(PROJECT_VER_MINOR)-$(PROJECT_VER_PATCH).uf2


clean-samd51-metro-airlift:
			rm -r build/samd51-metro-airlift/

.PHONY: clean
clean:
			rm -r build/*
			rm -r examples/Wippersnapper_demo/build/*
			rm -r examples/wippersnapper-simpletest-esp8266/build/*
