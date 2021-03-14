import astropy
# from astropy import subpackage  
from astropy import units as u
from astropy import coordinates as coord
import csv
import numpy as np

file1 = open('test_python.txt', 'r')
with open('test_output.csv', mode='w') as test_file:
    test_writer = csv.writer(test_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    temp = []
    count = 0
    while True:
        count += 1
        line1 = file1.readline()
        if not line1:
            break
        tempString = line1.strip()
        temp = tempString.split()

        range_val = float(temp[0])
        azimuth_val = float(temp[1])
        altitude_val = float(temp[2])
        encoder_val = float(temp[3])
        n = float(temp[4])
        range_val -= n
        x = (range_val * np.cos(azimuth_val) * np.cos(altitude_val)) + (n * np.cos(encoder_val))
        y = (range_val * np.sin(azimuth_val) * np.cos(altitude_val)) + (n * np.sin(encoder_val))
        z = (range_val * np.sin(altitude_val))
        row = [x, y, z]
        test_writer.writerow(row)

file1.close()
test_file.close()
print(count)

    

