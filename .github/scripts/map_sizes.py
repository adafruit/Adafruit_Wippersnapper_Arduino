#!/usr/bin/env python3
"""Summarise flash use per library / object from a GNU ld .map file.

TEMPORARY diagnostic for adafruit/Adafruit_Wippersnapper_Arduino#1175 (SAMD
flash overflow). Works even when the link failed on a region overflow, as
long as ld still wrote the map.

usage: map_sizes.py <file.map> [top_n]
"""
import collections
import re
import sys

FLASH_SECTIONS = (".text", ".rodata", ".data", ".ARM.extab", ".ARM.exidx")
# " .text.foo   0x00001234   0x56 /path/obj.o"  (name may wrap onto its own line)
LINE = re.compile(r"^\s+(\.\S+)?\s*0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+(\S.*\.(?:o|a\(.+\)))\s*$")


def group_of(path):
    p = path.replace("\\", "/")
    m = re.search(r"/libraries/([^/]+)/", p)
    if m:
        return "lib:" + m.group(1)
    if "/sketch/" in p or "Wippersnapper_demo" in p:
        return "sketch"
    if "/core/" in p or "core.a" in p:
        return "core"
    m = re.search(r"/(lib[^/]+\.a)\(", p)
    if m:
        return "toolchain:" + m.group(1)
    return "other"


def main():
    path = sys.argv[1]
    top_n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    text = open(path, encoding="utf-8", errors="replace").read()
    start = text.find("Linker script and memory map")
    body = text[start:] if start >= 0 else text
    by_group = collections.Counter()
    by_obj = collections.Counter()
    pending = None
    for raw in body.splitlines():
        m = LINE.match(raw)
        if not m:
            s = raw.strip()
            pending = s if s.startswith(".") and len(s.split()) == 1 else None
            continue
        sec = m.group(1) or pending or ""
        pending = None
        if not sec.startswith(FLASH_SECTIONS):
            continue
        size = int(m.group(2), 16)
        obj = m.group(3)
        by_group[group_of(obj)] += size
        by_obj[obj.replace("\\", "/").split("/libraries/")[-1]] += size
    total = sum(by_group.values())
    print(f"Flash (.text/.rodata/.data) attributed: {total} bytes")
    print("\n== by library / group ==")
    for g, s in by_group.most_common():
        print(f"{s:9d}  {g}")
    print(f"\n== top {top_n} objects ==")
    for o, s in by_obj.most_common(top_n):
        print(f"{s:9d}  {o}")


if __name__ == "__main__":
    main()
