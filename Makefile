PROJECT_NAME := WIPPERSNAPPER

TEST_PLATFORMS = ~/Desktop/github_brentru/ci-arduino/build_platform.py

UF2_FAMILY_SAMD51 := 0x55114460
UF2_BASE_SAMD51   := 0x4000

build-m4-metro-airlift:
			python3 $(TEST_PLATFORMS) metro_m4_airliftlite_tinyusb --export-binaries


.PHONY: clean
clean:
			rm -r examples/Wippersnapper_demo/build/*/*
			rm -r uf2