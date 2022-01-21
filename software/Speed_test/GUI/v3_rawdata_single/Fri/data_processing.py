import matplotlib.pyplot as plt
import numpy as np
import math
import struct
import sys

data_file = sys.argv[1]

bytecnt_per_frame = 29

with open(data_file, mode='rb') as file: # b is important -> binary
    fileContent = file.read()

datacnt = len(fileContent)
framecnt = int(datacnt/bytecnt_per_frame)

frame_interval = 0.1 # frame interval is 100ms
time_x = frame_interval * np.linspace(0, framecnt-1, num=framecnt)

# print(time_x)

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

speed = []

enl_b = 0
enl_speed = 0

frame_num_b = 0
frame_num_m = []

for i in range(framecnt):
    # frame number
    frame_num_i = int(fileContent[i*bytecnt_per_frame + 1])

    if i == 0:
        frame_num_b = frame_num_i - 1

    frame_num_m.append(frame_num_i - frame_num_b)
    frame_num_b = frame_num_i

    frame_num.append(frame_num_i)

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

    accelx_f = struct.unpack('f', accelx_i)[0]
    accely_f = struct.unpack('f', accely_i)[0]
    accelz_f = struct.unpack('f', accelz_i)[0]

    accelx_d.append(accelx_f)
    accely_d.append(accely_f)
    accelz_d.append(accelz_f)

    gravity.append(math.sqrt(accelx_f**2 + accely_f**2 + accelz_f**2))

    # encoder L data
    enl_i = int(fileContent[i*bytecnt_per_frame+26])

    enl_speed = enl_i - enl_b

    if (enl_speed < 0):
        enl_i = enl_i + 255

    speed.append(enl_i - enl_b)

    enl_b = enl_i

    enl_d.append(int(enl_i))

    # encoder R data
    enr_i = int(fileContent[i*bytecnt_per_frame+27])

    enr_d.append(int(enr_i))

fig = plt.figure()

# ======================== Raw data ========================
ax1 = plt.subplot(221)
plt.plot(time_x, frame_num, 'g', marker='o')
ax1.set_xlabel('time (s)', fontdict = {'fontsize' : 12})
ax1.set_ylabel('Frame number', fontdict = {'fontsize' : 12})

ax2 = plt.subplot(222)
plt.plot(time_x, gyrox_d, 'b')
ax2.set_xlabel('time (s)', fontdict = {'fontsize' : 12})
ax2.set_ylabel('Elevation angle (dgree)', fontdict = {'fontsize' : 12})

ax3 = plt.subplot(223)
plt.plot(time_x, gravity, 'k')
ax3.set_xlabel('time (s)', fontdict = {'fontsize' : 12})
ax3.set_ylabel('Acceleration $(m/s^{2})$', fontdict = {'fontsize' : 12})

ax4 = plt.subplot(224)
plt.plot(time_x, enl_d, 'r', marker='o')
ax4.set_xlabel('time (s)', fontdict = {'fontsize' : 12})
ax4.set_ylabel('Encoder count', fontdict = {'fontsize' : 12})

# ======================== Raw data ========================
# ax1 = plt.subplot(311)
# plt.plot(time_x, frame_num, 'g', marker='o')
# ax1.set_ylabel('Frame number', fontdict = {'fontsize' : 12})

# ax2 = plt.subplot(312)
# plt.plot(time_x, enl_d, 'r', marker='o')
# ax2.set_ylabel('Encoder data', fontdict = {'fontsize' : 12})

# ax3 = plt.subplot(313)
# plt.plot(time_x, speed, 'k', marker='o')
# ax3.set_xlabel('time (s)', fontdict = {'fontsize' : 12})
# ax3.set_ylabel('Speed', fontdict = {'fontsize' : 12})

# ======================== Raw data ========================
# ax= plt.plot(111)
# plt.plot(time_x, frame_num_m, 'b', marker='o')
# plt.xlabel('time (s)', fontdict = {'fontsize' : 12})
# plt.ylabel('Frame number step', fontdict = {'fontsize' : 12})

plt.show()
