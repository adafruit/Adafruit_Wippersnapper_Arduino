# Brownout / power-loss filesystem hardening — open items

Status: **draft / request-for-comment.** This is a running checklist of data-loss
and corruption hazards found while validating #937 (don't auto-format a
provisioned board) and #942 (auto-repair a power-loss-erased boot region) on real
hardware. Several of these are **not yet fixed** — the goal of this file is to get
the WipperSnapper maintainers' eyes on the full scope before either PR leaves
draft. Corrections and "no, that's fine because…" very welcome.

## How we got here (one paragraph)

A provisioned MagTag was drained flat repeatedly. Two distinct outcomes were
observed and dumped off-chip:

1. **Boot region erased** (first 4 KB of `ffat` = boot sector + FAT). The #942
   rescue restored it from the NVS backup and the board recovered itself —
   `secrets.json` intact, no reformat. ✅
2. **`secrets.json` "lost."** The FAT mounted cleanly, but the firmware halted on
   *Invalid IO credentials*. An off-chip dump showed the **real `secrets.json`
   was byte-intact in an orphaned cluster**, while the live directory entry now
   pointed at a freshly-written placeholder template. Nothing was corrupted — a
   transient brownout *read* (rail sag → `0x00`/`0xFF` bytes) was misinterpreted
   as "empty file → recreate," which deleted the good file. This is exactly the
   "misinterpretation of a failed flash operation" @tyeth called out, and it
   confirms his warning that the reset reason "varies between those reboots (not
   always brownout)."

The takeaway driving this list: **brownout failures are a *family* of bugs, not
one.** Format-on-mount-fail (#937) and boot-region-erase (#942) are two members;
the *recreate-on-bad-read* paths and a *guard-timing* bug are still open.

---

## Must-fix before #937 leaves draft

### [ ] 1. CRITICAL — `WS.brownOutCausedReset` is set too late; every FS-path check of it is dead code
`brownOutCausedReset` is only set in `printDeviceInfo()` → `print_reset_reason()`
(`Wippersnapper.cpp`), which runs inside **`connect()`**. But the sketch calls
**`provision()` first**, and the entire `Wippersnapper_FS` constructor +
`parseSecrets()` run inside `provision()`. So for the whole filesystem path the
flag is still its zero-init value (`false`).

Consequence — every `if (WS.brownOutCausedReset)` guard in the FS path never
fires:
- `parseSecrets()` "can't recreate due to brownout" — never protected anything.
- `eraseCPFS()` `if (brownOutCausedReset) return;` — dead; it always runs.
- `createBootFile()` brownout skips — the "skip write under brownout" branch
  never triggers.

Same class as a guard wired to a value that isn't live yet.

**Empirical backing — the brownout reset code never even fires on this board.**
`WS.brownOutCausedReset` is set only on reset code `15` (brownout). Across **~2,400
boots over every drain cycle logged**, the observed reset-reason distribution is:

| code | reason | count |
|------|--------|-------|
| 1  | POWERON_RESET | 2290 |
| 3  | SW_RESET      | 101  |
| 12 | SW_CPU_RESET  | 11   |
| 15 | BROWNOUT_RST  | **0** |

Code `15` was observed **zero times** — every single power-loss recovery comes
back as `POWERON_RESET`. (The word "brownout" in the logs is exclusively our own
halt-message text, not a reset code.) So the guard is doubly dead: set too late to
matter *and* keyed on a code that empirically never occurs here. Even with the
timing fixed, gating on the brownout reset code would protect ~0% of real drains.
This is the measurement behind @tyeth's "the boot reason varies (not always
brownout)" — on this hardware it is *never* brownout.

**Fix:** establish the boot reason *before* constructing the FS
(`getResetReasonCode()` is a ROM call valid immediately at boot) if a boot-reason
is wanted for diagnostics, but **stop gating FS-destructive ops on the reset
reason at all** — use the persistent NVS provisioned-marker, which is both live
early and reliable. This is the keystone; several items below collapse once it's
done.

### [x] 2. HIGH — `configFileExists()` → `createSecretsFile()` was an un-fixed twin of the bug already patched in `parseSecrets()` — **now fixed**
`configFileExists()` returns false when the **first byte** of `secrets.json`
reads back `0x00` or `0xFF` (`file.peek()`), which is precisely what a brownout
misread produces. `initFilesystem()` then calls `createSecretsFile()` →
placeholder template + halt. It was **not guarded by `wasFilesystemProvisioned()`**,
and it runs *inside the constructor, before* `parseSecrets()` — so it's actually the
earlier / more likely trigger of the template overwrite we saw. The `parseSecrets`
fix did **not** cover this door.

**Confirmed live on hardware, then fixed.** A subsequent battery drain walked
straight through this exact door: the MagTag mounted cleanly, `configFileExists()`
peeked its intact `secrets.json` back blank (boot reason `1 (POWERON_RESET)`, **not**
brownout), `createSecretsFile()` overwrote the live directory entry with the
template, and the board halted on *"Please edit the secrets.json file."* An
off-chip `ffat` dump showed both blobs present — the real credentials byte-intact
in an orphaned cluster and the fresh template in the live entry — the same orphaned
-good-file signature as the `parseSecrets` case, just through the earlier door.

**Fixed** by gating the recreate on the same persistent NVS provisioned-marker
`parseSecrets()` uses: a provisioned board that suddenly cannot read `secrets.json`
now halts-and-protects instead of recreating; only a never-provisioned board may
bootstrap a template.

### [ ] 3. Single destructive-op rule across the FS
Audit every destructive operation — `f_mkfs`/format, `wipperFatFs.remove(...)`,
`rmRfStar()`, `createSecretsFile()` — against one invariant:
**never destroy data on a previously-provisioned board without a deliberate user
gesture.** Today only the *format* decision (#937) and *one* recreate path (the
new `parseSecrets` fix) honor it.

## Same-PR hardening (strongly recommended)

### [ ] 4. MEDIUM — `eraseCPFS()` runs every boot behind the dead guard (#1)
Removes `boot_out.txt`, `code.py`, and `rmRfStar()`s `/lib`. No-op for a normal
WipperSnapper board (no CircuitPython files), so it hasn't bitten us — but it's a
latent `rm -rf` gated only on the dead `brownOutCausedReset` and a single
`exists()` a brownout could misread. Gate on the provisioned-marker / a real
boot-reason read.

### [ ] 5. MEDIUM — minimize flash writes while the rail is marginal
During a drain the constructor *writes* repeatedly as Vcc collapses — each an
opportunity to interrupt a program/erase and corrupt the FAT (the very thing #942
recovers): `createBootFile()` rewrites `wipper_boot_out.txt`; `.fseventsd/`,
`.metadata_never_index`, `.Trashes` are (re)created on `exists()` misreads;
`writeToBootOut()` appends on every halt/error (which defeats `createBootFile`'s
content-compare on the next boot). This is @tyeth's "diff before write" point —
fewer writes on a marginal rail is strictly safer.

### [ ] 6. MEDIUM — a cosmetic dot-file glitch can spuriously halt a provisioned board
If any `.fseventsd`/`.metadata_never_index`/`.Trashes` open fails,
`initFilesystem()` returns false → the constructor's format-decision path →
`HaltProvisionedBrownout`. No data loss (good), but a macOS-marker hiccup taking
a provisioned board offline is an availability surprise worth a guard.

## #942 / follow-up scope

### [ ] 7. MEDIUM — boot-region backup staleness → real FAT cross-linking
A drain dump showed `wipper_boot_out.txt` and the template `secrets.json`
**cross-linked** (overlapping clusters) — concrete fallout of restoring a stale
boot region (old FAT) over a filesystem whose files had since moved. The
"one-write-stale" limitation documented on #942 can produce an *inconsistent*
FAT, not merely an old-but-valid one. (See also the silent backup-write-failure
limitation already noted in the #942 description.)

### [ ] 8. DESIGN — the rescue backup window is fixed, but the erasure location is random (aka stochastic)
#942 backs up only the **first 4 KB** (boot sector + FAT). A power-loss erase hits
a 4 KB-aligned block, but **which** block varies boot-to-boot — so a fixed backup
window cannot cover it. Demonstrated across real drains of the *same* board:

- **Drain #1** — the erase hit `0x310000` (boot sector + FAT). Rescue's window
  matched → it restored and the board self-healed. ✅
- **Drain #4** — the erase hit `0x311000`–`0x314000` (**the root directory**), plus
  `0x32d000`–`0x32f000` (data). The first 4 KB was **intact** (4089 non-`0xFF`,
  valid `55AA`), so the rescue correctly recorded *live-region-structured →
  hands-off* and did nothing; #937 then correctly halted (unmountable + provisioned
  → refuse to reformat). Both behaved as designed — but the damage was entirely
  **outside** the rescue's 4 KB window, so the board could not self-heal. An
  off-chip dump confirmed `secrets.json`'s bytes were still byte-complete at
  `0x316600`, but **orphaned**: the root-directory entry pointing to them was
  erased, so the file is unreachable and the volume won't mount. Data physically
  present, functionally lost.
- **Drain #7** — a near-exact replay of #4 on a *different* root-dir block: the
  erase hit `0x311000`–`0x312000` (the **first 4 KB sector of the root
  directory**), zeroing-out to `0xFF` the very entry block where `secrets.json`'s
  8.3 directory record lived. After the erase, an off-chip dump showed **no
  `SECRETS JSON` directory entry anywhere** on the volume, yet the real
  credentials were still byte-complete in their data cluster (cluster 10, FAT
  still `EOF`-allocated) — orphaned with no name pointing at them. The boot region
  (first 4 KB) was again intact, so the rescue correctly stood down and #937
  halted (*"refusing to reformat to protect your secrets.json"*). The serial log
  caught the whole arc in one sitting: the board ran healthy on real creds for
  ~2.4 h, then a single power-loss event erased the root-dir sector and it was
  unmountable from that boot on. Notably the **fixed-window rescue *did* fire once
  earlier in the same recovery saga** (`rescues so far: 1`) when an erase happened
  to land on its boot-region window — concrete proof the cover is real but
  *location-dependent*: same board, same firmware, three different erase blocks,
  three different outcomes.
- **Drain #9** — the finest-grained metadata erase yet: the casualty was the
  **single 32-byte `secrets.json` directory record** alone. Unlike #4/#7 the
  volume stayed **fully mountable** — boot sector valid (`55AA`), FAT internally
  consistent (70 clusters, no out-of-range pointers / loops / cross-links),
  `wipper_boot_out.txt` chaining cleanly — yet a full root-dir scan found **no
  `secrets.json` entry**, while the real credentials sat byte-complete and
  `EOF`-allocated at cluster 13, orphaned. So "metadata erase" isn't only the
  coarse *root-dir-sector* case (#7); a power-loss can also surgically drop **one
  live directory entry** and leave an otherwise-healthy volume that simply no
  longer knows `secrets.json` exists. A region/boot-sector backup cannot cover
  this (the boot region was pristine); only #8(b) — backing up `secrets.json`
  *content* — would. Footnote: this boot stayed **dark (no USB-CDC serial)** and
  the dump could *not* pin that on FS corruption (the FAT was clean), so the
  protective halt may not always be reached — a separate worry that the
  halt-and-protect floor only helps if the firmware survives init to print it.
- **Drain #10** — the same erase *location* as #7 (the first 4 KB root-dir sector,
  ffat `0x311000`, all `0xFF`) but the **opposite mount outcome**: here the boot
  sector stayed valid (`55AA`) and the FAT stayed consistent (45 clusters, no
  OOB), so the volume was **mountable** and only `secrets.json` went missing
  (its entry was in the erased sector; creds again orphaned byte-complete at
  cluster 13) — the #9 *outcome* via the #7 *mechanism*. The lesson: **a given
  erase location does not map to a fixed outcome.** #7's first-sector erase
  presented as *unmountable* (→ `#937` reformat-refusal halt); #10's presented as
  *mountable with secrets.json orphaned*. Whether SdFat mounts or chokes on an
  all-`0xFF` directory sector is itself variable, so the failure is a spectrum in
  *both* axes — erase location **and** how the mount layer reacts to it. Same
  conclusion: only #8(b) (content backup of `secrets.json`) auto-covers every
  point on that spectrum.

So this is broader than "data clusters aren't backed up" — the casualty in drains
#4, #7, #9 and #10 was **metadata** (the root directory, down to a single entry).
Two directions, not mutually exclusive:

- **(a)** widen the metadata backup to boot sector + FAT + **root directory**
  (first ~`0x5000`). Still loses if a `secrets.json` *data* cluster is the block
  that dies.
- **(b)** *(most robust — recommended)* back up **`secrets.json` content** to NVS
  and re-materialize it on a failed mount, regardless of which metadata/data block
  was erased. This covers the actual goal — *never lose the credentials* — where a
  region-based backup only covers *some* erase locations.

Reframe for #937/#942 reviewers: the boot-region rescue is a best-effort cover for
*one* erase location; on its own it cannot make "survive any single-block
power-loss erase" a guarantee. #937's halt-and-protect is the floor that catches
every location the rescue misses (no data destroyed), but the board still needs a
human to recover unless (b) is added.

### [ ] 12. HIGH (#942) — the rescue only restores a *blank* boot region, not a *garbage-corrupted* one — so it stands down in-window when it could have healed
Distinct from #8 (damage *outside* the window). Here the damage was squarely
**inside** the rescue's 4 KB window, a valid backup was present, the rest of the
filesystem was intact — and the rescue still did nothing, because its restore
trigger only recognises a *clean* erase, not corruption.

Observed live (drain #8, off-USB drain, recovered-on-its-own **failed**):
- The **boot region** (`ffat 0x310000`, boot sector + FAT, file `0x0`–`0x1000`)
  came back as **high-entropy garbage** — 3767 of 4096 bytes non-`0xFF`, no valid
  `0x55AA` boot signature (offset `0x1FE` read `f7 fb`), boot sector starting
  `fb fe 97 cf …` instead of a FAT `EB ..`. Not a clean erase — a *partial /
  aborted-write corruption*.
- **Everything else was intact.** The root directory parsed cleanly and the live
  `secrets.json` entry (short name `SECRE~11JSO`, cluster 13) pointed at a
  **byte-valid `secrets.json` with the real credentials** — confirmed in the dump
  (`{ "io_username": "flaviof" … }`, 207 bytes). `wipper_boot_out.txt` intact too.
- A **valid boot-region backup existed in NVS** (`ws_boot_bk`/`boot_bk` keys
  present; a `0x55AA`-bearing boot sector found in the NVS dump). The damage was
  exactly what the backup covers.
- Yet the FS was unmountable (garbage boot sector + FAT) → #937 correctly halted.

Why the rescue stood down: `restoreBootRegionIfBlank()` restores only when the
live region is **"blank enough"** — no `0x55AA` **and** at most ~`len/64` (≈64)
non-`0xFF` stragglers. A brownout that *corrupts* rather than *erases* leaves the
region with **thousands** of non-`0xFF` bytes, so `3767 ≫ 64` fails the test and
the rescue hands off — precisely when restoring its known-good backup would have
made the volume mountable again. The heuristic was tuned for the clean-erase case
(drain #1) and is blind to in-window corruption.

**Fix:** broaden the restore trigger from "blank enough" to **"structurally
invalid / unmountable"** — i.e. restore the CRC-verified backup whenever the live
boot sector lacks a valid `0x55AA` (or won't parse as FAT), whether the region is
blank *or* garbage. An unmountable boot region has nothing worth preserving, so a
known-good restore can only help. Caveat (see #7): the backup is one-write-stale,
so a restored FAT may need an fsck pass — but a stale-yet-valid FAT is strictly
better than garbage, and it keeps the auto-heal promise for the *in-window* case
instead of silently giving up. This is the cheapest high-value #942 hardening:
the backup and the window were both right; only the trigger was too narrow.

### [ ] 9. LOW — writable USB-MSC after mount
Once mounted, the volume is exposed writable; a host OS that flags it dirty can
write "repairs." Already mitigated for the unmounted case (MSC deliberately not
exposed there). Consider `usb_msc.setWritable(false)` until a deliberate edit
gesture.

## Process

### [ ] 10. A real test matrix — one green drain proves nothing
The outcome depends on *which* 4 KB block the collapsing rail disturbs and *when*
in the boot a read lands. We need fault injection (forced empty reads / single-
block erases at each write/read point) plus many battery drains, with a
byte-level survivor check (off-chip `ffat` dump) after each. Battery cycling alone
is too coarse and too slow to trust.

### [ ] 11. HIGH (recoverability) — the protective halt has no escape hatch; a board damaged beyond the rescue is permanently bricked for a normal user
Direct consequence of #937's halt, observed live after drain #4 **and again
after drain #7** (root-dir erase → unmountable → 204 consecutive halt prints
across a night of power-cycles, no in-firmware way out). Once the NVS
provisioned-marker is set and the FS is damaged in a way the #942 rescue can't
repair (e.g. the erased block was the **root directory**, item #8), the board is
deadlocked:

- mount fails every boot → #937 (correctly) refuses to reformat to protect
  `secrets.json` → `HaltProvisionedBrownout`, forever;
- the rescue can't help (damage outside its 4 KB window);
- re-flashing the **app** does nothing — the damage is in `ffat` (`0x310000`) and
  the marker is in `nvs` (`0x9000`), neither of which an app upload touches.

There is **no in-firmware path out**. Recovery required a host with `esptool` to
`erase_region` both `ffat` *and* `nvs` (erasing only one isn't enough: blank `ffat`
+ marker still set → still `HaltProvisionedBrownout`; cleared marker + non-blank
`ffat` → `HaltUnsafeToFormat`). A normal user with only the USB drive cannot do
this — for them the board is bricked.

So "refuse to reformat to protect secrets.json" needs a deliberate, user-reachable
**override**, e.g.:
- hold a button (D0/BOOT) during boot to authorize a one-time reformat, or
- after N consecutive provisioned-but-unmountable boots, expose a read-only MSC
  drive with a `RECOVER.txt`/flag the user can act on, or
- surface a clear "hold X to reset filesystem" prompt on the halt screen/serial.
The protective halt is the right floor; it just needs an exit that doesn't require
a soldering-iron-adjacent toolchain. Pairs naturally with item #9 (read-only MSC).

---

_All hardware logs and flash dumps referenced here have SSID/credentials redacted;
recovered private values were inspected locally only and never posted._
