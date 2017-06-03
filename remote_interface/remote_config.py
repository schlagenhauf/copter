#!/usr/bin/env python

import sys, serial, time
import itertools
import copcom_pb2

import numpy as np
from PyQt4 import QtCore, QtGui, uic

from serial_interface import *


class RemoteConfig(QtGui.QMainWindow):
    def __init__(self, args, parent=None):
        QtGui.QMainWindow.__init__(self,parent)

        self.s = SerialInterface('/dev/ttyACM0')

        self.pp = ProtobufParser(copcom_pb2.PbCopterCommand())

        self.statusBar = QtGui.QStatusBar()
        self.setStatusBar(self.statusBar)

        w = QtGui.QWidget()
        self.setCentralWidget(w)
        grid = QtGui.QGridLayout()
        w.setLayout(grid)

        # for some reason this achieves to force all elements to minimum size
        grid.setSizeConstraint(QtGui.QLayout.SetFixedSize)


        self.spinBoxes = []
        fns = list(itertools.chain(*self.pp.getFieldNames()))
        for fn in fns:
            # create group box with vertical layout
            gb = QtGui.QGroupBox(w)
            gbLayout = QtGui.QHBoxLayout()
            gb.setLayout(gbLayout)

            # add label
            lb = QtGui.QLabel()
            lb.setText(fn)
            gbLayout.addWidget(lb)

            # add spin box
            sb = QtGui.QDoubleSpinBox(w)
            self.spinBoxes.append(sb)
            gbLayout.addWidget(sb)

            # add group to main grid layout
            grid.addWidget(gb)


        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.checkConnection)
        self.timer.start(1000)

    def checkConnection(self):
        self.statusBar.showMessage(self.s.connect(False))
        if self.s.connected:
            self.timer.stop()


    def changed(self):
        pass






if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    pbv = RemoteConfig(sys.argv)
    pbv.show()
    app.exec_()
