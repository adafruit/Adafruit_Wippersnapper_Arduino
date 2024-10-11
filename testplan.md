copy and analyse the hex difference for a corrupted file system versus freshly saved secrets + hunky dorey

retest first secrets.json acces on a new device on tinyUF2 0.18.2 and WS 88, versus tinyUF2 0.20.1 and WS88, then with no-ds-format-nor-file-recreation instead of WS88

then add the metro-s3
Update step images in ws_boards for metro-s3

then add FT usb-to-gpio to ws_python


Test ws_boards c6-aVref branch (esp32 aref=1.1 instead of 3.3, and C6 from 1.1 to 2.5v)

Fix pin numbers + names for Metro S2 (all digital etc, tx/rx and D1 [etc] seem wrong, analog5 is wrong etc)