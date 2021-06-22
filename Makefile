PROJECT_NAME := WIPPERSNAPPER
PYTHON := python3

# Path to https://github.com/adafruit/ci-arduino/blob/master/build_platform.py
TEST_PLATFORMS = ~/Desktop/github_brentru/ci-arduino/build_platform.py

# SAMD51 - base address adn familyID
UF2_BASE_SAMD51   := 0x4000
UF2_FAMILY_SAMD51 := 0x55114460

samd51-metro-airlift:
			mkdir build-m4-metro-airlift
			python3 $(TEST_PLATFORMS) metro_m4_airliftlite_tinyusb --export-binaries

clean-samd51-metro-airlift:
			rm -r build-m4-metro-airlift


.PHONY: clean
clean:
			rm -r examples/Wippersnapper_demo/build/*/*
			rm -r uf2