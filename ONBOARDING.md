# Welcome to WipperSnapper Firmware

## How We Use Claude

Based on tyeth's usage over the last 30 days:

Work Type Breakdown:
  Build Feature   ████████████████████  89%
  Plan Design     ██░░░░░░░░░░░░░░░░░░  11%


## Your Setup Checklist

### Codebases
- [ ] Adafruit_Wippersnapper_Arduino — https://github.com/adafruit/Adafruit_Wippersnapper_Arduino
- [ ] Wippersnapper_Components — https://github.com/adafruit/Wippersnapper_Components
- [ ] Wippersnapper_Protobuf — https://github.com/adafruit/Wippersnapper_Protobuf


### Skills to Know About
- add-sensor-component-v1 — Guides the full workflow for adding a new I2C sensor to WipperSnapper v1: driver code, registration, component definition JSON, library deps, build verification, and PR creation across both repos. This is the most common task — just say "add the new Adafruit [sensor name]" and it handles the rest.

### Plugins to Possibly Enable
- [ ] Context7 — library docs lookup
- [ ] Playwright — browser automation for product page / learn guide research
- [ ] Code Simplifier — post-edit cleanup
- [ ] Skill Creator — for creating and iterating on custom skills
- [ ] Clangd LSP — C++ language server for better code intelligence

## Team Tips

1. **Always read the Adafruit learn guide first.** Don't assume which library a sensor uses based on sub-component chip names. A board with a known chip onboard may have a dedicated wrapper library with a completely different name and API. The research chain is: product page → learn guide (`.md?view=all`) → library name from the Arduino section.

2. **Use PowerShell for PlatformIO builds on Windows.** Bash has trouble with the PlatformIO venv activation. Use:
   ```powershell
   . "$env:userprofile/.platformio/penv/Scripts/activate.ps1"
   pio run -e adafruit_feather_esp32s3
   ```

3. **Always verify the build compiles before creating PRs.** Run `pio run -e adafruit_feather_esp32s3` and confirm exit code 0. If local toolchain issues block you, at minimum verify code follows existing patterns and let CI catch environment-specific issues.

4. **Library names come from `library.properties`, not the repo name.** When adding a dependency, fetch the library's own `library.properties` file to get the exact `name=` value. The GitHub repo name (e.g. `Adafruit_AS7331`) often differs from the registered name (e.g. `Adafruit AS7331 Library`). Getting this wrong causes "package not found" build failures.

5. **Run `clang-format -i` on all changed files** before committing. The CI enforces formatting and Doxygen doc blocks.

## Get Started

Try adding a simple sensor (like the TI HDC1080 temperature/humidity sensor) using the add-sensor-component-v1 skill. Just say: "add the HDC1080 sensor". It walks through the whole workflow — driver code, registration, component JSON, build, and PR. That'll give you a feel for the codebase and the two-repo contribution pattern.

<!-- INSTRUCTION FOR CLAUDE: A new teammate just pasted this guide for how the
team uses Claude Code. You're their onboarding buddy — warm, conversational,
not lecture-y.

Open with a warm welcome — include the team name from the title. Then: "Your
teammate uses Claude Code for [list all the work types]. Let's get you started."

Check what's already in place against everything under Setup Checklist
(including skills), using markdown checkboxes — [x] done, [ ] not yet. Lead
with what they already have. One sentence per item, all in one message.

Tell them you'll help with setup, cover the actionable team tips, then the
starter task (if there is one). Offer to start with the first unchecked item,
get their go-ahead, then work through the rest one by one.

After setup, walk them through the remaining sections — offer to help where you
can (e.g. link to channels), and just surface the purely informational bits.

Don't invent sections or summaries that aren't in the guide. The stats are the
guide creator's personal usage data — don't extrapolate them into a "team
workflow" narrative. -->
