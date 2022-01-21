#!/usr/bin/env python
# work with MPU6050_kalman.ino

from PyQt5 import QtCore, QtWidgets, uic, QtGui
from pyqtgraph import PlotWidget
from PyQt5.QtWidgets import QApplication, QVBoxLayout
import pyqtgraph as pg
import numpy as np
import datetime
import serial
import sys
import os
import time
from time import sleep
from colorama import Fore, Back, Style
import csv
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
import random
import struct

start_cmd    = 0x11
interval_cmd = 0x22
sleep_cmd    = 0x33
aq_cmd       = 0x44

nbins = 20
data_len = 200

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def read_current_time():
    now = datetime.datetime.now(datetime.timezone.utc)
    current_time = now.strftime("%Z:%j/%H:%M:%S")
    return current_time

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setFixedSize(1030, 705)

        #Load the UI Page
        uic.loadUi('speed_test.ui', self)

        self.gyroz1.setBackground('w')
        self.gyroz2.setBackground('w')
        self.gyroz3.setBackground('w')

        self.enl1.setBackground('w')
        self.enl2.setBackground('w')
        self.enl3.setBackground('w')

        self.enr1.setBackground('w')
        self.enr2.setBackground('w')
        self.enr3.setBackground('w')

        self.serial_ports_list = []
        self.serial_speed = [1000000]

        # Ref: https://stackoverflow.com/questions/59898215/break-an-infinit-loop-when-button-is-pressed
        self.timer = QtCore.QTimer(self, interval=5, timeout=self.read_port)
        self.ser=serial.Serial()

        self.scan_btn.clicked.connect(self.scan)
        self.open_btn.clicked.connect(self.open_port)
        self.close_btn.clicked.connect(self.close)
        self.start_btn.clicked.connect(self.start_read_port)
        self.stop_btn.clicked.connect(self.stop_read_port)

        self.gyroz1_data = [0] * data_len
        self.gyroz2_data = [0] * data_len
        self.gyroz3_data = [0] * data_len

        self.enl1_data = [0] * data_len
        self.enl2_data = [0] * data_len
        self.enl3_data = [0] * data_len

        self.enr1_data = [0] * data_len
        self.enr2_data = [0] * data_len
        self.enr3_data = [0] * data_len

        self.time_index = list(range(1, data_len+1))

        self.start_id = 0

        for x in self.serial_speed:
            self.speed_comboBox.addItem(str(x))

    def scan(self):
        if os.name == 'nt':  # sys.platform == 'win32':
            from serial.tools.list_ports_windows import comports
        elif os.name == 'posix':
            from serial.tools.list_ports_posix import comports

        for info in comports(False):
            port, desc, hwid = info
        iterator = sorted(comports(False))

        self.serial_ports_list = [] # clear the list first
        for n, (port, desc, hwid) in enumerate(iterator, 1):
            self.serial_ports_list.append("{:20}\n".format(port))

        ports_num = len(self.serial_ports_list)

        self.serial_comboBox.clear() # clear the list first
        for x in self.serial_ports_list:
            self.serial_comboBox.addItem(x)

        self.start_id = 0

    def open_port(self):
        index = self.serial_comboBox.currentIndex()
        serial_ports_port = self.serial_ports_list[index][:-1] # delete the \n at the end
        index = self.speed_comboBox.currentIndex()
        self.ser = serial.Serial(serial_ports_port, self.serial_speed[index])

        current_time = read_current_time()
        print(current_time, self.ser.name + " Opened @ " + str(self.serial_speed[index]) + "bps")

    def start_read_port(self):
        self.gyroz1_data = [0] * data_len
        self.gyroz2_data = [0] * data_len
        self.gyroz3_data = [0] * data_len

        self.enl1_data = [0] * data_len
        self.enl2_data = [0] * data_len
        self.enl3_data = [0] * data_len

        self.enr1_data = [0] * data_len
        self.enr2_data = [0] * data_len
        self.enr3_data = [0] * data_len

        cmd_id = self.start_id & 0xFF
        start_cmd_chk = (cmd_id+0xAA+0xFF) & 0xFF

        start_cmd = [0xAA, 0x01, 0x02, 0x03]
        start_cmd = bytearray(start_cmd)
        self.ser.write(start_cmd)

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"Start cmd sent",Style.RESET_ALL)

        # waiting for the reply
        while (not(self.ser.inWaiting())):
            {}

        current_time = read_current_time()
        start_ack = self.ser.readline()

        # print(start_ack)

        # if (start_ack[0] == 0xAA):
        #     print(Back.LIGHTGREEN_EX, "   -> ", current_time,"got reply from Rovable #1",Style.RESET_ALL)
        # elif (start_ack[0] == 0xFF):
        #     print(Back.LIGHTRED_EX, "   -> ", current_time,"Rovable #1 timeout",Style.RESET_ALL)
        # else:
        #     print(Back.LIGHTBLUE_EX, "   -> ", current_time,"Incorrect reply from Rovable #1",Style.RESET_ALL)

        # if (start_ack[1] == 0xAA):
        #     print(Back.LIGHTGREEN_EX, "   -> ", current_time,"got reply from Rovable #2",Style.RESET_ALL)
        # elif (start_ack[1] == 0xFF):
        #     print(Back.LIGHTRED_EX, "   -> ", current_time,"Rovable #2 timeout",Style.RESET_ALL)
        # else:
        #     print(Back.LIGHTBLUE_EX, "   -> ", current_time,"Incorrect reply from Rovable #2",Style.RESET_ALL)

        # if (start_ack[2] == 0xAA):
        #     print(Back.LIGHTGREEN_EX, "   -> ", current_time,"got reply from Rovable #3",Style.RESET_ALL)
        # elif (start_ack[2] == 0xFF):
        #     print(Back.LIGHTRED_EX, "   -> ", current_time,"Rovable #3 timeout",Style.RESET_ALL)
        # else:
        #     print(Back.LIGHTBLUE_EX, "   -> ", current_time,"Incorrect reply from Rovable #3",Style.RESET_ALL)

        self.data_num = 0
        self.timer.start() # Start the timer

        self.start_id

    def stop_read_port(self):
        self.timer.stop() # Stop the timer

    def read_port(self):
        if (self.ser.inWaiting()):
            current_time = read_current_time()
            recv_data = self.ser.read(29)

            flag_1 = 0
            flag_2 = 0
            flag_3 = 0

            # print(recv_data)

            if (recv_data[0] == 0x01): # dats is from Rovable #1
                flag_1 = 1
                # print("----- from node 1 -----")
                frame1 = recv_data[1]
                # print("frame", frame1)

                gyrox1_i = recv_data[2:6]
                gyroy1_i = recv_data[6:10]
                gyroz1_i = recv_data[10:14]

                # print("gyrox1_i", gyrox1_i)
                # print("gyroy1_i", gyroy1_i)
                # print("gyroz1_i", gyroz1_i)

                gyrox1_d = struct.unpack('f', gyrox1_i)[0]
                gyroy1_d = struct.unpack('f', gyroy1_i)[0]
                gyroz1_d = struct.unpack('f', gyroz1_i)[0]

                accelx1_i = recv_data[14:18]
                accely1_i = recv_data[18:22]
                accelz1_i = recv_data[22:26]

                # print("accelx1_i", accelx1_i)
                # print("accely1_i", accely1_i)
                # print("accelz1_i", accelz1_i)

                accelx1_d = struct.unpack('f', accelx1_i)[0]
                accely1_d = struct.unpack('f', accely1_i)[0]
                accelz1_d = struct.unpack('f', accelz1_i)[0]

                enl1_i = int(recv_data[26])
                enr1_i = int(recv_data[27])

                # print("enl1_i", enl1_i)
                # print("enr1_i", enr1_i)

                chk_1 = recv_data[28]

                # print("chk_1", chk_1)

                # print(current_time, " ---> ", gyrox1_d, gyroy1_d, gyroz1_d, accelx1_d, accely1_d, accelz1_d, enl1_i, enr1_i, chk_1)

            if (recv_data[0] == 0x02): # dats is from Rovable #2
                flag_2 = 1
                # print("----- from node 1 -----")
                frame2 = recv_data[1]
                # print("frame", frame1)

                gyrox2_i = recv_data[2:6]
                gyroy2_i = recv_data[6:10]
                gyroz2_i = recv_data[10:14]

                # print("gyrox1_i", gyrox1_i)
                # print("gyroy1_i", gyroy1_i)
                # print("gyroz1_i", gyroz1_i)

                gyrox2_d = struct.unpack('f', gyrox2_i)[0]
                gyroy2_d = struct.unpack('f', gyroy2_i)[0]
                gyroz2_d = struct.unpack('f', gyroz2_i)[0]

                accelx2_i = recv_data[14:18]
                accely2_i = recv_data[18:22]
                accelz2_i = recv_data[22:26]

                # print("accelx1_i", accelx1_i)
                # print("accely1_i", accely1_i)
                # print("accelz1_i", accelz1_i)

                accelx2_d = struct.unpack('f', accelx2_i)[0]
                accely2_d = struct.unpack('f', accely2_i)[0]
                accelz2_d = struct.unpack('f', accelz2_i)[0]

                enl2_i = int(recv_data[26])
                enr2_i = int(recv_data[27])

                # print("enl1_i", enl1_i)
                # print("enr1_i", enr1_i)

                chk_2 = recv_data[28]

                # print(current_time, " ---> ", gyrox1_d, gyroy1_d, gyroz1_d, accelx1_d, accely1_d, accelz1_d, enl1_i, enr1_i, chk_1)


            if (recv_data[0] == 0x03): # dats is from Rovable #3
                flag_3 = 1
                # print("----- from node 3 -----")
                frame3 = recv_data[1]
                # print("frame", frame1)

                gyrox3_i = recv_data[2:6]
                gyroy3_i = recv_data[6:10]
                gyroz3_i = recv_data[10:14]

                # print("gyrox1_i", gyrox1_i)
                # print("gyroy1_i", gyroy1_i)
                # print("gyroz1_i", gyroz1_i)

                gyrox3_d = struct.unpack('f', gyrox3_i)[0]
                gyroy3_d = struct.unpack('f', gyroy3_i)[0]
                gyroz3_d = struct.unpack('f', gyroz3_i)[0]

                accelx3_i = recv_data[14:18]
                accely3_i = recv_data[18:22]
                accelz3_i = recv_data[22:26]

                # print("accelx1_i", accelx1_i)
                # print("accely1_i", accely1_i)
                # print("accelz1_i", accelz1_i)

                accelx3_d = struct.unpack('f', accelx3_i)[0]
                accely3_d = struct.unpack('f', accely3_i)[0]
                accelz3_d = struct.unpack('f', accelz3_i)[0]

                enl3_i = int(recv_data[26])
                enr3_i = int(recv_data[27])

                # print("enl1_i", enl1_i)
                # print("enr1_i", enr1_i)

                chk_3 = recv_data[28]

                # print("chk_1", chk_1)

                # print(current_time, " ---> ", gyrox1_d, gyroy1_d, gyroz1_d, accelx1_d, accely1_d, accelz1_d, enl1_i, enr1_i, chk_1)


            # =============== Plot for node 1 ===============
            if (flag_1 == 1):
                self.frame1.setText(str(frame1))

                self.gyroz1_label.setText(str(round(gyroz1_d, 2)))
                self.gyroz1_data.pop(0)
                self.gyroz1_data.append(gyroz1_d)
                self.gyroz1.clear()
                self.gyroz1.plot(self.time_index, self.gyroz1_data, pen=pg.mkPen('b', width=2))

                self.enl1_label.setText(str(enl1_i))
                self.enl1_data.pop(0)
                self.enl1_data.append(enl1_i)
                self.enl1.clear()
                self.enl1.plot(self.time_index, self.enl1_data, pen=pg.mkPen('b', width=2))

                self.enr1_label.setText(str(enr1_i))
                self.enr1_data.pop(0)
                self.enr1_data.append(enr1_i)
                self.enr1.clear()
                self.enr1.plot(self.time_index, self.enr1_data, pen=pg.mkPen('b', width=2))

                self.accelx1.setText(str(round(accelx1_d, 2)))
                self.accely1.setText(str(round(accely1_d, 2)))
                self.accelz1.setText(str(round(accelz1_d, 2)))

            # =============== Plot for node 2 ===============
            if (flag_2 == 1):
                self.frame2.setText(str(frame2))

                self.gyroz2_label.setText(str(round(gyroz2_d, 2)))
                self.gyroz2_data.pop(0)
                self.gyroz2_data.append(gyroz2_d)
                self.gyroz2.clear()
                self.gyroz2.plot(self.time_index, self.gyroz2_data, pen=pg.mkPen('r', width=2))

                self.enl2_label.setText(str(enl2_i))
                self.enl2_data.pop(0)
                self.enl2_data.append(enl2_i)
                self.enl2.clear()
                self.enl2.plot(self.time_index, self.enl2_data, pen=pg.mkPen('r', width=2))

                self.enr2_label.setText(str(enr2_i))
                self.enr2_data.pop(0)
                self.enr2_data.append(enr2_i)
                self.enr2.clear()
                self.enr2.plot(self.time_index, self.enr2_data, pen=pg.mkPen('r', width=2))

                self.accelx2.setText(str(round(accelx2_d, 2)))
                self.accely2.setText(str(round(accely2_d, 2)))
                self.accelz2.setText(str(round(accelz2_d, 2)))

            # =============== Plot for node 3 ===============
            if (flag_3 == 1):
                self.frame3.setText(str(frame3))

                self.gyroz3_label.setText(str(round(gyroz3_d, 2)))
                self.gyroz3_data.pop(0)
                self.gyroz3_data.append(gyroz3_d)
                self.gyroz3.clear()
                self.gyroz3.plot(self.time_index, self.gyroz3_data, pen=pg.mkPen('k', width=2))

                self.enl3_label.setText(str(enl3_i))
                self.enl3_data.pop(0)
                self.enl3_data.append(enl3_i)
                self.enl3.clear()
                self.enl3.plot(self.time_index, self.enl3_data, pen=pg.mkPen('k', width=2))

                self.enr3_label.setText(str(enr3_i))
                self.enr3_data.pop(0)
                self.enr3_data.append(enr3_i)
                self.enr3.clear()
                self.enr3.plot(self.time_index, self.enr3_data, pen=pg.mkPen('k', width=2))

                self.accelx3.setText(str(round(accelx3_d, 2)))
                self.accely3.setText(str(round(accely3_d, 2)))
                self.accelz3.setText(str(round(accelz3_d, 2)))

    def save_data(self):
        filename = self.filename_str.text()+'.csv'
        with open(filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(ranging_data)

# driver code 
if __name__ == '__main__': 
    # creating apyqt5 application 
    app = QApplication(sys.argv) 
    # creating a window object 
    main = MainWindow() 
    # showing the window 
    main.show()
    # loop 
    sys.exit(app.exec_()) 
