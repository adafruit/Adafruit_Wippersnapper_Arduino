# generates the 8-bit checksum from a JSON file, provided its path is given as an argument
import sys
import json

# load json file from provided path
with open(sys.argv[1], 'r') as f:
    json_data = json.load(f)
if not json_data:
    print("Error: JSON file did not contain any data or path didn't match exactly, exiting...")
    exit(1)

# convert json dict->string and sum
str_json_data = json.dumps(json_data)
sum_json_data = sum(bytearray(str_json_data.encode()))
print('Sum (hex): ', hex(sum_json_data))

# calculate checksum
calculated_checksum = sum(bytearray(str_json_data.encode())) & 0xFF
print('Checksum: ', calculated_checksum)
print('Checksum (hex): ', hex(calculated_checksum))

# store and write checksum to new file
json_data['checksum'] = calculated_checksum
with open(sys.argv[1], 'w') as f:
    json.dump(json_data, f)
print('SUCCESS: Checksum written to JSON file!')
