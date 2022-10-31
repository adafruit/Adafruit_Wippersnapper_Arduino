#!/usr/bin/python
import subprocess

def fetch_library_property_depends():
    with open('library.properties', 'r') as fh:
        for line in fh.readlines():
            if line.startswith('depends='):
                return line.replace('\n', '').split('=')[1].split(', ')
    return None

lib_list = fetch_library_property_depends()
print("-------------------------------------------")
print("Adding ", len(lib_list), " libraries")
print("-------------------------------------------")
for lib in lib_list:
    print("_-_ STARTED adding " + lib)
    subprocess.call(['pio','lib', 'install', lib ])
    print("_-_ DONE adding " + lib)
    print("-------------------------------------------")
        