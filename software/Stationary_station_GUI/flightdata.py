import matplotlib.pyplot as plt
import numpy as np
import math
import struct

with open("flightdatafri-final", mode='rb') as file: # b is important -> binary
    fileContent = file.read()

datacnt = len(fileContent)
framecnt = int(datacnt/36)

gyrox_d = []
gyroy_d = []
gyroz_d = []

accelx_d = []
accely_d = []
accelz_d = []

gravity = []

magx_d = []
magy_d = []
magz_d = []

for i in range(framecnt):
	gyrox_i = fileContent[i*36+0:i*36+4]
	gyroy_i = fileContent[i*36+4:i*36+8]
	gyroz_i = fileContent[i*36+8:i*36+12]

	accelx_i = fileContent[i*36+12:i*36+16]
	accely_i = fileContent[i*36+16:i*36+20]
	accelz_i = fileContent[i*36+20:i*36+24]

	magx_i = fileContent[i*36+24:i*36+28]
	magy_i = fileContent[i*36+28:i*36+32]
	magz_i = fileContent[i*36+32:i*36+36]

	gyrox_d.append(struct.unpack('f', gyrox_i)[0])
	gyroy_d.append(struct.unpack('f', gyroy_i)[0])
	gyroz_d.append(struct.unpack('f', gyroz_i)[0])

	accelx = struct.unpack('f', accelx_i)[0]
	accely = struct.unpack('f', accely_i)[0]
	accelz = struct.unpack('f', accelz_i)[0]

	accelx_d.append(accelx)
	accely_d.append(accely)
	accelz_d.append(accelz)

	gravity.append(math.sqrt(accelx**2 + accely**2 + accelz**2))

	magx_d.append(struct.unpack('f', magx_i)[0])
	magy_d.append(struct.unpack('f', magy_i)[0])
	magz_d.append(struct.unpack('f', magz_i)[0])

# print(len(gyrox_d))

# fig = plt.figure()
# ax = plt.axes()

# x = np.linspace(0, framecnt)
plt.figure(1)
plt.subplot(711)
plt.plot(gyrox_d)

plt.subplot(712)
plt.plot(gyroy_d, 'k')

plt.subplot(713)
plt.plot(gyroz_d, 'r')

# plt.subplot(914)
# plt.plot(accelx_d)

# plt.subplot(915)
# plt.plot(accely_d, 'k')

# plt.subplot(916)
# plt.plot(accelz_d, 'r')

plt.subplot(714)
plt.plot(gravity, 'g')

plt.subplot(715)
plt.plot(magx_d)

plt.subplot(716)
plt.plot(magy_d, 'k')

plt.subplot(717)
plt.plot(magz_d, 'r')
plt.show()