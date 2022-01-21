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
import math
import struct

start_cmd    = 0x11
interval_cmd = 0x22
sleep_cmd    = 0x33
aq_cmd       = 0x44

data_len = 30

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
        self.setFixedSize(721, 430)

        #Load the UI Page
        uic.loadUi('station.ui', self)

        self.gyrox.setBackground('w')
        self.gyroy.setBackground('w')
        self.gyroz.setBackground('w')

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
        self.set_btn.clicked.connect(self.set_para)
        self.cali_btn.clicked.connect(self.cali_station)

        self.rec_btn.clicked.connect(self.recording_data)
        self.stop_rec_btn.clicked.connect(self.stop_recording)

        self.threshold = 0.4

        self.threshold_in.setText("0.4")

        self.gyrox_data = [0] * data_len
        self.gyroy_data = [0] * data_len
        self.gyroz_data = [0] * data_len

        self.time_index = list(range(1, data_len+1))

        self.file = open("temp_no_valid_data", "wb")

        self.recording_flag = 0

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
        self.gyrox_data = [0] * data_len
        self.gyroy_data = [0] * data_len
        self.gyroz_data = [0] * data_len

        self.gyrox.clear()
        self.gyroy.clear()
        self.gyroz.clear()

        startcmd = [0x11]
        startcmd = bytearray(startcmd)
        self.ser.write(startcmd)

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"Start cmd sent",Style.RESET_ALL)

        current_time = read_current_time()

        self.timer.start() # Start the timer

    def set_para(self):
        self.threshold = float(self.threshold_in.text())

    def recording_data(self):
        self.recording_flag = 1
        recordingname = self.filename.text()
        self.file = open(recordingname, "wb")

    def stop_recording(self):
        self.recording_flag = 0
        self.file.close()

    def stop_read_port(self):
        stopcmd = [0xFF]
        stopcmd = bytearray(stopcmd)
        self.ser.write(stopcmd)

        self.timer.stop() # Stop the timer

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"STOP cmd sent",Style.RESET_ALL)

    def cali_station(self):
        calicmd = [0xAA]
        calicmd = bytearray(calicmd)
        self.ser.write(calicmd)

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"Cali station",Style.RESET_ALL)

    def read_port(self):
        if (self.ser.inWaiting()):
            current_time = read_current_time()
            recv_data = self.ser.read(36)

            if (self.recording_flag == 1):
                self.file.write(recv_data)

            gyrox_i = recv_data[0:4]
            gyroy_i = recv_data[4:8]
            gyroz_i = recv_data[8:12]

            accelx_i = recv_data[12:16]
            accely_i = recv_data[16:20]
            accelz_i = recv_data[20:24]

            magx_i = recv_data[24:28]
            magy_i = recv_data[28:32]
            magz_i = recv_data[32:36]

            gyrox_d = struct.unpack('f', gyrox_i)[0]
            gyroy_d = struct.unpack('f', gyroy_i)[0]
            gyroz_d = struct.unpack('f', gyroz_i)[0]

            accelx_d = struct.unpack('f', accelx_i)[0]
            accely_d = struct.unpack('f', accely_i)[0]
            accelz_d = struct.unpack('f', accelz_i)[0]

            magx_d = struct.unpack('f', magx_i)[0]
            magy_d = struct.unpack('f', magy_i)[0]
            magz_d = struct.unpack('f', magz_i)[0]

            gravity = math.sqrt(accelx_d**2 + accely_d**2 + accelz_d**2)
            self.gravity_label.setText(str(round(gravity, 1)))

            if (gravity > 9.00 and gravity < 10.50):
                self.gravity.setText("Earth G")
                self.gravity.setStyleSheet("background-color : blue")
            elif (gravity > 10.5):
                self.gravity.setText("High G")
                self.gravity.setStyleSheet("background-color : green")
            elif (gravity > 1.00 and gravity < 2.00):
                self.gravity.setText("Lunar G")
                self.gravity.setStyleSheet("background-color : yellow")
            elif (gravity > 3.00 and gravity < 4.00):
                self.gravity.setText("Martian G")
                self.gravity.setStyleSheet("background-color : red")
            else:
                self.gravity.setText(str(round(gravity, 1)))
                self.gravity.setStyleSheet("background-color : white")

            self.gyrox_label.setText(str(round(gyrox_d, 2)))
            self.gyrox_data.pop(0)
            self.gyrox_data.append(gyrox_d)
            self.gyrox.clear()
            self.gyrox.plot(self.time_index, self.gyrox_data, pen=pg.mkPen('b', width=2))

            self.gyroy_label.setText(str(round(gyroy_d, 2)))
            self.gyroy_data.pop(0)
            self.gyroy_data.append(gyroy_d)
            self.gyroy.clear()
            self.gyroy.plot(self.time_index, self.gyroy_data, pen=pg.mkPen('r', width=2))

            self.gyroz_label.setText(str(round(gyroz_d, 2)))
            self.gyroz_data.pop(0)
            self.gyroz_data.append(gyroz_d)
            self.gyroz.clear()
            self.gyroz.plot(self.time_index, self.gyroz_data, pen=pg.mkPen('k', width=2))

            gyroxmax_i = max(self.gyrox_data)
            gyroymax_i = max(self.gyroy_data)
            gyrozmax_i = max(self.gyroz_data)

            gyroxmin_i = min(self.gyrox_data)
            gyroymin_i = min(self.gyroy_data)
            gyrozmin_i = min(self.gyroz_data)

            if (gyroxmax_i < self.threshold and gyroxmin_i > -1*self.threshold):
                if (gyroymax_i < self.threshold and gyroymin_i > -1*self.threshold):
                    if (gyrozmax_i < self.threshold and gyrozmin_i > -1*self.threshold):
                        self.status_btn.setText("OK")
                        self.status_btn.setStyleSheet("background-color : #00ff00;")
                    else:
                        self.status_btn.setText("Waiting...")
                        self.status_btn.setStyleSheet("background-color : orange")
                else:
                    self.status_btn.setText("Waiting...")
                    self.status_btn.setStyleSheet("background-color : orange")
            else:
                self.status_btn.setText("Waiting...")
                self.status_btn.setStyleSheet("background-color : orange")

            if (abs(gyroxmax_i) > abs(gyroxmin_i)):
                self.gyroxmax.setText(str(round(gyroxmax_i, 2)))
            else:
                self.gyroxmax.setText(str(round(gyroxmin_i, 2)))

            if (abs(gyroymax_i) > abs(gyroymin_i)):
                self.gyroymax.setText(str(round(gyroymax_i, 2)))
            else:
                self.gyroymax.setText(str(round(gyroymin_i, 2)))

            if (abs(gyrozmax_i) > abs(gyrozmin_i)):
                self.gyrozmax.setText(str(round(gyrozmax_i, 2)))
            else:
                self.gyrozmax.setText(str(round(gyrozmin_i, 2)))

            self.accelx.setText(str(round(accelx_d, 2)))
            self.accely.setText(str(round(accely_d, 2)))
            self.accelz.setText(str(round(accelz_d, 2)))

            self.magx.setText(str(round(magx_d, 2)))
            self.magy.setText(str(round(magy_d, 2)))
            self.magz.setText(str(round(magz_d, 2)))

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
