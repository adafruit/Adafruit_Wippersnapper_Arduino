// Host-side unit tests for the filesystem (re)format decision policy.
//
// Pure logic, no hardware: compile and run on any host C++ compiler.
//
//   c++ -std=c++17 -I src/provisioning/tinyusb \
//       test/native/test_fs_format_policy.cpp -o /tmp/fs_policy_test
//   /tmp/fs_policy_test
//
// This is the regression guard for adafruit/Adafruit_Wippersnapper_Arduino
// issue #621/#831: a board that has previously been provisioned must NEVER be
// reformatted on a mount failure (which would erase the user's secrets.json),
// no matter how healthy the flash currently looks.

#include "Wippersnapper_FS_format_policy.h"
#include <stdio.h>

static int g_failures = 0;

static const char *name(WsFsAction a) {
  switch (a) {
  case WsFsAction::Mounted:
    return "Mounted";
  case WsFsAction::Format:
    return "Format";
  case WsFsAction::HaltProvisionedBrownout:
    return "HaltProvisionedBrownout";
  case WsFsAction::HaltUnsafeToFormat:
    return "HaltUnsafeToFormat";
  }
  return "?";
}

// EXPECT(mounted, provisioned, safeToFormat, expectedAction)
static void expect(bool mounted, bool provisioned, bool safe,
                   WsFsAction want, const char *desc) {
  WsFsAction got = decideFsAction(mounted, provisioned, safe);
  if (got == want) {
    printf("  ok   : %s\n", desc);
  } else {
    printf("  FAIL : %s\n         decideFsAction(mounted=%d, provisioned=%d, "
           "safeToFormat=%d) = %s, expected %s\n",
           desc, mounted, provisioned, safe, name(got), name(want));
    g_failures++;
  }
}

int main() {
  printf("decideFsAction() truth table\n");

  // A successful mount is always "Mounted", whatever the other signals say.
  expect(true, false, false, WsFsAction::Mounted, "mounted (fresh board)");
  expect(true, true, true, WsFsAction::Mounted, "mounted (provisioned board)");

  // THE FIX / regression guard: previously-provisioned board whose FAT will not
  // mount must refuse to format - even when the flash reads back perfectly
  // healthy (the exact case that wiped secrets.json after a deep brownout).
  expect(false, true, true, WsFsAction::HaltProvisionedBrownout,
         "provisioned + unmountable + flash looks healthy -> refuse to format");
  expect(false, true, false, WsFsAction::HaltProvisionedBrownout,
         "provisioned + unmountable + flash unhealthy -> refuse to format");

  // Genuine first boot (never provisioned): format only when the flash is
  // healthy and the volume is genuinely blank.
  expect(false, false, true, WsFsAction::Format,
         "first boot + blank healthy flash -> format");
  expect(false, false, false, WsFsAction::HaltUnsafeToFormat,
         "first boot + unhealthy/not-blank flash -> refuse to format");

  if (g_failures == 0) {
    printf("PASS: all cases\n");
    return 0;
  }
  printf("FAIL: %d case(s)\n", g_failures);
  return 1;
}
