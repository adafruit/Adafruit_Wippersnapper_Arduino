/*!
 * @file Wippersnapper_FS_format_policy.h
 *
 * Pure (hardware-free) decision policy for whether the filesystem may be
 * (re)formatted after a mount attempt. Kept in its own header, free of any
 * Arduino/flash/NVS includes, so the gating logic can be unit-tested on a host
 * compiler (see test/native/test_fs_format_policy.cpp).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WIPPERSNAPPER_FS_FORMAT_POLICY_H
#define WIPPERSNAPPER_FS_FORMAT_POLICY_H

/*!
    @brief  What the filesystem constructor should do after attempting to mount
            the existing filesystem.
*/
enum class WsFsAction {
  Mounted, ///< Filesystem mounted - nothing to (re)create.
  Format,  ///< Genuine first boot on healthy, blank flash - safe to create FS.
           ///< [DESTRUCTIVE]
  HaltProvisionedBrownout, ///< Board was previously provisioned but the FAT
                           ///< won't mount: corruption (likely brownout). Must
                           ///< NOT format - that would erase secrets.json.
  HaltUnsafeToFormat,      ///< Never provisioned, but the flash is unhealthy or
                           ///< the volume is not genuinely blank. Refuse to
                           ///< format until power is stable.
};

/**************************************************************************/
/*!
    @brief    Pure decision for what to do after a filesystem mount attempt.
              Contains no hardware access so it can be unit-tested on a host.

              The ordering is the whole point of the fix: a board that has ever
              had a working filesystem (wasProvisioned) must NEVER be
              reformatted on a mount failure, regardless of how healthy the
              flash currently looks - once a brownout-drained battery recovers
              enough to boot, the flash reads fine again, so flashSafeToFormat
              alone cannot tell "blank from the factory" apart from "corrupted
              by a brownout".
    @param    mounted
                True if the FAT volume mounted successfully.
    @param    wasProvisioned
                True if a working filesystem has previously been seen on this
                board (persisted in NVS across brownouts and FAT formats).
    @param    flashSafeToFormat
                True if the flash chip is responding AND the volume reads back
                genuinely blank/erased (only meaningful when !wasProvisioned).
    @returns  The action the constructor should take.
*/
/**************************************************************************/
inline WsFsAction decideFsAction(bool mounted, bool wasProvisioned,
                                 bool flashSafeToFormat) {
  if (mounted)
    return WsFsAction::Mounted;
  if (wasProvisioned)
    return WsFsAction::HaltProvisionedBrownout; // protect secrets.json
  if (!flashSafeToFormat)
    return WsFsAction::HaltUnsafeToFormat;
  return WsFsAction::Format; // genuine first boot on healthy, blank flash
}

#endif // WIPPERSNAPPER_FS_FORMAT_POLICY_H
