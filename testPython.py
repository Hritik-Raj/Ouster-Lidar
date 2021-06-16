import astropy
# from astropy import subpackage  
from astropy import units as u
from astropy import coordinates as coord
import csv
import numpy as np
import math

def runScript(filename, writeFile, gen1_altitude_angles, gen1_azimuth_angles):

	piVal = 3.14159265358979323846264338327950288
	frameVal = ""
	timeVal = ""
	file1 = open(filename, 'r')
	w = 1024
	azimuthRads = piVal * 2.0/ w
	azimuthArr = [0] * 32768
	altitudeArr = [0] * 32768
	with open(writeFile, mode='w') as test_file:
		test_writer = csv.writer(test_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		temp = []
		count = 0
		uCount = 0
		vCount = 0
		while True:
			line1 = file1.readline()
			if not line1:
				break
			tempString = line1.strip()
			temp = tempString.split()

			if (temp[0]) == "Frame:":
				# continue
				frameVal = (temp[1])
			
			elif (temp[0]) == "Time:":
				# continue
				timeVal = (temp[1])
			
			else:
				count += 1
				vCount += 1
				if vCount == 1024:
					vCount = 0
					uCount += 1
					if uCount == 32:
						uCount = 0
				range_val = float(temp[0]) # corrected range
				if math.isclose(range_val, 0, abs_tol= 0.001):
					continue
				encoder_val = 2 * piVal - (vCount * azimuthRads)
				# i = uVal * 1024 + v
				i = uCount * 1024 + vCount
				azimuthArr[i] = -gen1_azimuth_angles[uCount] * piVal / 180.0
				altitudeArr[i] = gen1_altitude_angles[uCount] * piVal / 180.0
				correctedAzimuth = azimuthArr[i] + encoder_val
				n = 15.8060
				range_val -= n
				x = (range_val * np.cos(correctedAzimuth) * np.cos(altitudeArr[i])) + (n * np.cos(encoder_val))
				y = (range_val * np.sin(correctedAzimuth) * np.cos(altitudeArr[i])) + (n * np.sin(encoder_val))
				z = (range_val * np.sin(altitudeArr[i]))
				row = [x, y, z]
				test_writer.writerow(row)
		file1.close()
		test_file.close()
		print(count)

if __name__ == "__main__":
	gen1_altitude_angles = [
	12.56,
	9.84,
	7.08,
	4.29,
	3.25,
	2.2,
	1.85,
	1.48,
	1.12,
	0.78,
	0.44,
	0.09,
	-0.28,
	-0.62,
	-0.96,
	-1.32,
	-1.68,
	-2.02,
	-2.37,
	-2.72,
	-3.1,
	-3.77,
	-4.49,
	-5.15,
	-5.9,
	-7.28,
	-8.66,
	-10.03,
	-11.4,
	-12.74,
	-14.07,
	-15.39
	]


	gen1_azimuth_angles =[
	-4.19,
	-4.2,
	-4.23,
	-4.24,
	-1.41,
	1.41,
	-1.41,
	-4.26,
	4.21,
	1.4,
	-1.43,
	-4.24,
	4.21,
	1.4,
	-1.42,
	-4.24,
	4.21,
	1.41,
	-1.43,
	-4.25,
	4.2,
	-1.43,
	4.21,
	-1.42,
	4.2,
	4.21,
	4.2,
	4.21,
	4.2,
	4.2,
	4.21,
	4.21
	]
	runScript("testing.txt", 'test_output.csv', gen1_altitude_angles, gen1_azimuth_angles)
