# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build/Test Commands
- Build with PlatformIO: `pio run`
- Upload firmware: `python upload_no_build.py`
- Run tests: `cd tests && python test_offline.py`
- Run single test: `wokwi-cli --elf tests/bin/offline/firmware.elf --timeout 120000 --scenario tests/scenarios/offline/[test-file].scenario.yaml --diagram tests/diagrams/offline.json`

## Code Style Guidelines
- Follow Adafruit library conventions
- Header guards: UPPERCASE_WITH_UNDERSCORES
- Classes: Model-View-Controller pattern (see components directory)
- Naming: camelCase for methods, snake_case for variables, UPPER_CASE for constants
- Imports: Group Arduino core, Adafruit libraries, then project-specific headers
- Documentation: Use Doxygen-style comments for classes and functions
- Error handling: Return error codes, use status LEDs for user feedback
- Follow existing code patterns for new components

## Contributing
- Follow Code of Conduct in CODE_OF_CONDUCT.md
- For new components: https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper
- For new boards: https://learn.adafruit.com/how-to-add-a-new-board-to-wippersnapper