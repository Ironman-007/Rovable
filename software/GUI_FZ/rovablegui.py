#!/usr/bin/env python
# pyuic5 LunarGUI.ui -o LunarGUI.py
# problems left:
#   - Not checking the ACK CHK.

from PyQt5 import QtCore, QtWidgets, uic, QtGui
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import numpy as np
import datetime
import serial
import sys
import os
import time
from time import sleep
from colorama import Fore, Back, Style

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setFixedSize(1392, 800)

        #Load the UI Page
        uic.loadUi('Rovables_GUI.ui', self)

        self.data1.setBackground('w')
        self.data2.setBackground('w')
        self.data3.setBackground('w')
        self.data4.setBackground('w')
        self.data5.setBackground('w')
        self.data6.setBackground('w')
        self.data7.setBackground('w')
        self.data8.setBackground('w')
        self.data9.setBackground('w')
        self.data10.setBackground('w')
        self.data11.setBackground('w')
        self.data12.setBackground('w')
        self.data13.setBackground('w')
        self.data14.setBackground('w')
        self.data15.setBackground('w')

        self.data1.plot([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], [1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10], pen=pg.mkPen('b', width=3))

        self.close_btn.clicked.connect(self.close)

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
