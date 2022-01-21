#!/usr/bin/env python
# work with MPU6050_kalman.ino

from PyQt5 import QtCore, QtWidgets, uic, QtGui
from pyqtgraph import PlotWidget
from PyQt5.QtWidgets import QApplication, QVBoxLayout
import pyqtgraph as pg
import datetime
import serial
import sys
import os
import time
from time import sleep
from colorama import Fore, Back, Style
import csv
import struct
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtCore import Qt

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
        self.setFixedSize(1030, 685)
        self.title = "Rovables Speed Test"

        #Load the UI Page
        uic.loadUi('speed_test_v2.ui', self)

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

        self.time_index=list(range(1, data_len+1))

        for x in self.serial_speed:
            self.speed_comboBox.addItem(str(x))

        painter = QPainter(self.track1)

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

        self.start_id    = 0
        self.interval_id = 0
        self.sleep_id    = 0

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

        self.heading1_data = [0] * data_len
        self.heading2_data = [0] * data_len
        self.heading3_data = [0] * data_len

        self.enl1_data = [0] * data_len
        self.enl2_data = [0] * data_len
        self.enl3_data = [0] * data_len

        self.enr1_data = [0] * data_len
        self.enr2_data = [0] * data_len
        self.enr3_data = [0] * data_len

        self.data_num = 0
        self.timer.start() # Start the timer

    def stop_read_port(self):
        self.timer.stop() # Stop the timer

    def read_port(self):
        if (self.ser.inWaiting()):
            current_time = read_current_time()
            recv_data = self.ser.read(24) # 3 double value: gyrox, gyroy, gyroz
            if (recv_data[0] == 0x01): # dats is from Rovable #1
                frame1 = recv_data[1]
                gyroz1_i = recv_data[2]
                heading1_i = recv_data[0:8]
                enl1_i = recv_data[8:16]
                enr1_i = recv_data[16:24]

            gyrox_d=struct.unpack('d', gyrox_i)[0]
            gyroy_d=struct.unpack('d', gyroy_i)[0]
            gyroz_d=struct.unpack('d', gyroz_i)[0]

            # print(current_time, " ---> ", gyrox_d, gyroy_d, gyroz_d)

            self.gyrox_data.pop(0)
            self.gyrox_data.append(gyrox_d)

            self.gyrox.clear()
            self.gyrox.plot(self.time_index, self.gyrox_data, pen=pg.mkPen('b', width=2))

            self.gyroy_data.pop(0)
            self.gyroy_data.append(gyroy_d)

            self.gyroy.clear()
            self.gyroy.plot(self.time_index, self.gyroy_data, pen=pg.mkPen('r', width=2))

            self.gyroz_data.pop(0)
            self.gyroz_data.append(gyroz_d)

            self.gyroz.clear()
            self.gyroz.plot(self.time_index, self.gyroz_data, pen=pg.mkPen('g', width=2))

    def save_data(self):
        filename = self.filename_str.text()+'.csv'
        with open(filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(ranging_data)

    def drawLine(self):
        painter = QtGui.QPainter(self.track1.pixmap())
        painter.drawLine(0, 0, 100, 200)
        painter.end()

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
