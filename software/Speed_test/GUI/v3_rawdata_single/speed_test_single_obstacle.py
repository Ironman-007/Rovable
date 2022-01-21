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
import os.path

start_cmd    = 0x11
interval_cmd = 0x22
sleep_cmd    = 0x33
aq_cmd       = 0x44

nbins = 20
data_len = 100

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
        self.setFixedSize(625, 650)

        #Load the UI Page
        uic.loadUi('speed_test_single.ui', self)

        self.gyroz1.setBackground('w')

        self.enl1.setBackground('w')

        self.enr1.setBackground('w')

        self.serial_ports_list = []
        self.serial_speed = [115200]

        # Ref: https://stackoverflow.com/questions/59898215/break-an-infinit-loop-when-button-is-pressed
        self.timer = QtCore.QTimer(self, interval=5, timeout=self.read_port)
        self.ser=serial.Serial()

        self.scan_btn.clicked.connect(self.scan)
        self.open_btn.clicked.connect(self.open_port)
        self.close_btn.clicked.connect(self.close)
        self.start_btn.clicked.connect(self.start_read_port)
        self.stop_btn.clicked.connect(self.stop_read_port)
        self.cali_btn.clicked.connect(self.send_cali_cmd)

        self.gyroz1_data = [0] * data_len

        self.enl1_data = [0] * data_len

        self.enr1_data = [0] * data_len

        self.time_index = list(range(1, data_len+1))

        self.start_id = 0

        self.file = open("temp_no_valid_data", "wb")

        self.botnum = ""
        self.botnumh = 0
        self.address = 0x01

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
        self.enl1_data = [0] * data_len
        self.enr1_data = [0] * data_len

        self.gyroz1.clear()
        self.enr1.clear()
        self.enl1.clear()

        cmd_id = self.start_id & 0xFF
        start_cmd_chk = (cmd_id+0xAA+0xFF) & 0xFF

        self.botnum = self.whichbot.text()
        self.botnumh = int(self.botnum, 16)

        if (self.botnumh == 0x01 or self.botnumh == 0x04 or self.botnumh == 0x0E):
            self.address = 0x01
        elif (self.botnumh == 0x02 or self.botnumh == 0x05 or self.botnumh == 0x09):
            self.address = 0x02
        elif (self.botnumh == 0x06):
            self.address = 0x03
        elif (self.botnumh == 0x07):
            self.address = 0x04
        elif (self.botnumh == 0x08 or self.botnumh == 0x0A):
            self.address = 0x05
        else:
            print(bcolors.WARNING + "Err: wrong Rovable number" + bcolors.ENDC)

        start_cmd = [0xAA, self.address]
        start_cmd = bytearray(start_cmd)
        print(start_cmd)
        self.ser.write(start_cmd)

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"Start cmd sent",Style.RESET_ALL)

        current_time = read_current_time()

        self.log.append(current_time + ": Start Cmd sent." + str(start_cmd) + "\n")

        self.data_num = 0
        self.timer.start() # Start the timer

        self.start_id = self.start_id + 1

        recordingname = self.botnum

        while (os.path.isfile(recordingname)):
            recordingname = recordingname + "1"

        self.file = open(recordingname, "wb")

    def send_cali_cmd(self):
        self.gyroz1_data = [0] * data_len
        self.enl1_data = [0] * data_len
        self.enr1_data = [0] * data_len

        self.gyroz1.clear()
        self.enr1.clear()
        self.enl1.clear()

        self.botnum = self.whichbot.text()
        self.botnumh = int(self.botnum, 16)

        if (self.botnumh == 0x01 or self.botnumh == 0x04 or self.botnumh == 0x0E):
            self.address = 0x01
        elif (self.botnumh == 0x02 or self.botnumh == 0x05 or self.botnumh == 0x09):
            self.address = 0x02
        elif (self.botnumh == 0x06):
            self.address = 0x03
        elif (self.botnumh == 0x07):
            self.address = 0x04
        elif (self.botnumh == 0x08 or self.botnumh == 0x0A):
            self.address = 0x05
        else:
            print(bcolors.WARNING + "Err: wrong Rovable number" + bcolors.ENDC)

        cali_cmd = [0x11, self.address]
        cali_cmd = bytearray(cali_cmd)
        print(cali_cmd)
        self.ser.write(cali_cmd)

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"CALI cmd sent",Style.RESET_ALL)

        current_time = read_current_time()

        self.log.append(current_time + ": CALI Cmd sent." + str(cali_cmd) + "\n")

        self.data_num = 0
        self.timer.start() # Start the timer

        self.start_id = self.start_id + 1

    def stop_read_port(self):
        self.botnum = self.whichbot.text()
        self.botnumh = int(self.botnum, 16)

        if (self.botnumh == 0x01 or self.botnumh == 0x04 or self.botnumh == 0x0E):
            self.address = 0x01
        elif (self.botnumh == 0x02 or self.botnumh == 0x05 or self.botnumh == 0x09):
            self.address = 0x02
        elif (self.botnumh == 0x06):
            self.address = 0x03
        elif (self.botnumh == 0x07):
            self.address = 0x04
        elif (self.botnumh == 0x08 or self.botnumh == 0x0A):
            self.address = 0x05
        else:
            print(bcolors.WARNING + "Err: wrong Rovable number" + bcolors.ENDC)

        stop_cmd = [0xFF, self.address]
        stop_cmd = bytearray(stop_cmd)
        print(stop_cmd)
        self.ser.write(stop_cmd)

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"Stop cmd sent",Style.RESET_ALL)

        current_time = read_current_time()

        self.log.append(current_time + ": Stop Cmd sent." + str(stop_cmd) + "\n")

        self.timer.stop() # Stop the timer

        self.file.close()

    def read_port(self):
        if (self.ser.inWaiting()):
            current_time = read_current_time()
            recv_data = self.ser.read(29)

            self.file.write(recv_data)

            rovable_num = self.whichbot.text()

            frame = recv_data[1]

            gyrox1_i = recv_data[2:6]
            gyroy1_i = recv_data[6:10]
            gyroz1_i = recv_data[10:14]

            gyrox1_d = struct.unpack('f', gyrox1_i)[0]
            gyroy1_d = struct.unpack('f', gyroy1_i)[0]
            gyroz1_d = struct.unpack('f', gyroz1_i)[0]

            accelx1_i = recv_data[14:18]
            accely1_i = recv_data[18:22]
            accelz1_i = recv_data[22:26]

            accelx1_d = struct.unpack('f', accelx1_i)[0]
            accely1_d = struct.unpack('f', accely1_i)[0]
            accelz1_d = struct.unpack('f', accelz1_i)[0]

            enl1_i = int(recv_data[26])
            enr1_i = int(recv_data[27])

            chk_1 = recv_data[28]

            self.rovable_num.setText(str(rovable_num))
            self.frame1.setText(str(frame))

            self.gyroz1_label.setText(str(round(gyroz1_d, 2)))
            self.gyroz1_data.pop(0)
            self.gyroz1_data.append(gyroz1_d)
            self.gyroz1.clear()
            self.gyroz1.plot(self.time_index, self.gyroz1_data, pen=pg.mkPen('b', width=2))

            self.enl1_label.setText(str(enl1_i))
            self.enl1_data.pop(0)
            self.enl1_data.append(enl1_i)
            self.enl1.clear()
            self.enl1.plot(self.time_index, self.enl1_data, pen=pg.mkPen('r', width=2))

            self.enr1_label.setText(str(enr1_i))
            self.enr1_data.pop(0)
            self.enr1_data.append(enr1_i)
            self.enr1.clear()
            self.enr1.plot(self.time_index, self.enr1_data, pen=pg.mkPen('k', width=2))

            self.accelx1.setText(str(round(accelx1_d, 2)))
            self.accely1.setText(str(round(accely1_d, 2)))
            self.accelz1.setText(str(round(accelz1_d, 2)))

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
