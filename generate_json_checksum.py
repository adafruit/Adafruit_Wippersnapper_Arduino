# generates the 8-bit checksum from a JSON file, provided its path is given as an argument
import sys
import json

# load json file from provided path
try:
    with open(sys.argv[1], 'r') as f:
        json_data = json.load(f)
except FileNotFoundError as e:
    print("Error: JSON file not found, exiting...")
    exit(1)

if not json_data:
    print("Error: JSON file did not contain any data or path didn't match exactly, exiting...")
    exit(1)

if 'checksum' in json_data:
    del json_data['checksum']

# NOTE: This is important to re-serialize json data to match ArduinoJSON's compact serialization format
str_json_data = json.dumps(json_data, separators=(',', ':'), sort_keys=True)

# calculate checksum
calculated_checksum = sum(bytearray(str_json_data.encode())) & 0xFF
print('Checksum: ', calculated_checksum)
print('Checksum (hex): ', hex(calculated_checksum))

# store and write checksum to new file
json_data['checksum'] = calculated_checksum
with open(sys.argv[1], 'w') as f:
    json.dump(json_data, f)
print('SUCCESS: Checksum written to JSON file!')