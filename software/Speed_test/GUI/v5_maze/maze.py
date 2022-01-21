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
        uic.loadUi('maze.ui', self)

        self.gyroz1.setBackground('w')

        self.disl.setBackground('w')

        self.disr.setBackground('w')

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

        self.disl_data = [0] * data_len

        self.disr_data = [0] * data_len

        self.time_index = list(range(1, data_len+1))

        self.start_id = 0

        self.file = open("temp_no_valid_data", "wb")

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
        self.disl_data = [0] * data_len
        self.disr_data = [0] * data_len

        self.gyroz1.clear()
        self.disl.clear()
        self.disr.clear()

        cmd_id = self.start_id & 0xFF
        start_cmd_chk = (cmd_id+0xAA+0xFF) & 0xFF

        botnum = self.whichbot.text()
        botnumh = int(botnum, 16)

        start_cmd = [0xAA, botnumh]
        start_cmd = bytearray(start_cmd)
        print(start_cmd)
        self.ser.write(start_cmd)

        current_time = read_current_time()
        print(Back.LIGHTYELLOW_EX, "   -> ", current_time,"Start cmd sent",Style.RESET_ALL)

        # waiting for the reply
        # while (not(self.ser.inWaiting())):
        #     {}

        current_time = read_current_time()
        # start_ack = self.ser.readline()

        self.log.append(current_time + ": Start Cmd sent." + str(start_cmd) + "\n")
# 
        # print(start_ack)

        # if (start_ack[0] == 0xAA):
        #     print(Back.LIGHTGREEN_EX, "   -> ", current_time,"got reply from Rovable #1",Style.RESET_ALL)
        # elif (start_ack[0] == 0xFF):
        #     print(Back.LIGHTRED_EX, "   -> ", current_time,"Rovable #1 timeout",Style.RESET_ALL)
        # else:
        #     print(Back.LIGHTBLUE_EX, "   -> ", current_time,"Incorrect reply from Rovable #1",Style.RESET_ALL)

        self.data_num = 0
        self.timer.start() # Start the timer

        self.start_id = self.start_id + 1

        recordingname = self.filename.text()
        self.file = open(recordingname, "wb")

    def send_cali_cmd(self):
        self.gyroz1_data = [0] * data_len
        self.disl_data = [0] * data_len
        self.disr_data = [0] * data_len

        self.gyroz1.clear()
        self.disl.clear()
        self.disr.clear()

        botnum = self.whichbot.text()
        botnumh = int(botnum, 16)

        cali_cmd = [0x11, botnumh]
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
        botnum = self.whichbot.text()
        botnumh = int(botnum, 16)
        stop_cmd = [0xFF, botnumh]
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
            recv_data = self.ser.read(31)

            self.file.write(recv_data)

            rovable_num = self.whichbot.text()

            frame = recv_data[1]

            gyroz1_i = recv_data[10:14]
            gyroz1_d = struct.unpack('f', gyroz1_i)[0]

            accelx1_i = recv_data[14:18]
            accely1_i = recv_data[18:22]
            accelz1_i = recv_data[22:26]

            accelx1_d = struct.unpack('f', accelx1_i)[0]
            accely1_d = struct.unpack('f', accely1_i)[0]
            accelz1_d = struct.unpack('f', accelz1_i)[0]

            disL = int(recv_data[28])
            disR = int(recv_data[29])

            self.rovable_num.setText(str(rovable_num))
            self.frame1.setText(str(frame))

            self.gyroz1_label.setText(str(round(gyroz1_d, 2)))
            self.gyroz1_data.pop(0)
            self.gyroz1_data.append(gyroz1_d)
            self.gyroz1.clear()
            self.gyroz1.plot(self.time_index, self.gyroz1_data, pen=pg.mkPen('b', width=2))

            self.disl_label.setText(str(disL))
            self.disl_data.pop(0)
            self.disl_data.append(disL)
            self.disl.clear()
            self.disl.plot(self.time_index, self.disl_data, pen=pg.mkPen('r', width=2))

            self.disr_label.setText(str(disR))
            self.disr_data.pop(0)
            self.disr_data.append(disR)
            self.disr.clear()
            self.disr.plot(self.time_index, self.disr_data, pen=pg.mkPen('k', width=2))

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
