import matplotlib.pyplot as plt
import numpy as np
import math
import struct
import sys

data_file = sys.argv[1]

with open(data_file, mode='rb') as file: # b is important -> binary
    fileContent = file.read()

bytecnt_per_frame = 29

datacnt = len(fileContent)
framecnt = int(datacnt/bytecnt_per_frame)

frame_num = []

gyrox_d = []
gyroy_d = []
gyroz_d = []

accelx_d = []
accely_d = []
accelz_d = []

enl_d = []
enr_d = []

gravity = []

for i in range(framecnt):
    # frame number
    frame_num_i = fileContent[i*bytecnt_per_frame + 1]

    frame_num.append(int(frame_num_i))

    # gyro data
    gyrox_i = fileContent[i*bytecnt_per_frame+2:i*bytecnt_per_frame+6]
    gyroy_i = fileContent[i*bytecnt_per_frame+6:i*bytecnt_per_frame+10]
    gyroz_i = fileContent[i*bytecnt_per_frame+10:i*bytecnt_per_frame+14]

    gyrox_d.append(struct.unpack('f', gyrox_i)[0])
    gyroy_d.append(struct.unpack('f', gyroy_i)[0])
    gyroz_d.append(struct.unpack('f', gyroz_i)[0])

    # accel data
    accelx_i = fileContent[i*bytecnt_per_frame+14:i*bytecnt_per_frame+18]
    accely_i = fileContent[i*bytecnt_per_frame+18:i*bytecnt_per_frame+22]
    accelz_i = fileContent[i*bytecnt_per_frame+22:i*bytecnt_per_frame+26]

    accelx_d.append(struct.unpack('f', accelx_i)[0])
    accely_d.append(struct.unpack('f', accely_i)[0])
    accelz_d.append(struct.unpack('f', accelz_i)[0])

    # encoder L data
    enl_i = fileContent[i*bytecnt_per_frame+26]

    enl_d.append(int(enl_i))

    # encoder R data
    enr_i = fileContent[i*bytecnt_per_frame+27]

    enr_d.append(int(enr_i))

fig = plt.figure()

# ax1 = plt.subplot(711)
# plt.plot(frame_num, 'y', marker='o')

ax1 = plt.subplot(321)
plt.plot(frame_num, 'y', marker='o')

ax2 = plt.subplot(322)
plt.plot(gyrox_d, 'b')

ax3 = plt.subplot(323)
plt.plot(accelx_d, 'k')
ax4 = plt.subplot(324)
plt.plot(accely_d, 'g')
ax5 = plt.subplot(325)
plt.plot(accelz_d, 'orange')

ax6 = plt.subplot(326)
plt.plot(enl_d, 'b', marker='o')
# ax7 = plt.subplot(321)
# plt.plot(enr_d, 'r', marker='o')

ax1.title.set_text('First Plot')
ax2.title.set_text('Second Plot')
ax3.title.set_text('Third Plot')
ax4.title.set_text('Fourth Plot')

plt.show()
