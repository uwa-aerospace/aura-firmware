import struct
import csv
import sys
import os

header_val = 0x69694269
record_format = "<I f f fff fff fff fff ffff"
record_size = struct.calcsize(record_format)

input_path = sys.argv[1]
output_path = os.path.splitext(input_path)[0] + ".csv"

with open(input_path, "rb") as f, open(output_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(
        [
            "logTime",
            "accelVertVel",
            "accelCorrected_x",
            "accelCorrected_y",
            "accelCorrected_z",
            "accelMs_x",
            "accelMs_y",
            "accelMs_z",
            "gyroCorrected_x",
            "gyroCorrected_y",
            "gyroCorrected_z",
            "gyroDps_x",
            "gyroDps_y",
            "gyroDps_z",
            "attitudeQuatn_w",
            "attitudeQuatn_v_x",
            "attitudeQuatn_v_y",
            "attitudeQuatn_v_z",
        ]
    )
    data = f.read()
    offset = 0
    while offset <= len(data) - 4:
        if struct.unpack_from("<I", data, offset)[0] == header_val:
            if offset + record_size <= len(data):
                record = struct.unpack_from(record_format, data, offset)
                writer.writerow(record[1:])
                offset += record_size
            else:
                break
        else:
            offset += 1
