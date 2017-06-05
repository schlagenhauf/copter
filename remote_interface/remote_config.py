#!/usr/bin/env python

import sys, serial, time
import itertools
import copcom_pb2
from google.protobuf.message import EncodeError as PbEncodeException

import numpy as np
from PyQt4 import QtCore, QtGui, uic

from serial_interface import *


class RemoteConfig(QtGui.QMainWindow):
    def __init__(self, args, parent=None):
        QtGui.QMainWindow.__init__(self,parent)

        self.s = SerialInterface('/dev/ttyACM0')
        self.pp = ProtobufParser(copcom_pb2.PbCopterCommand)
        self.msgPath = 'parameters.pbvals'

        try:
            with open(self.msgPath, 'rb') as f:
                byteArray = f.read()
                self.pp.accumMsg.ParseFromString(byteArray)
                # TODO: fill in values into spin boxes
        except IOError:
            print 'Failed to load parameters from file "%s". Using default values.' % self.msgPath


        # set up GUI
        self.statusBar = QtGui.QStatusBar()
        self.setStatusBar(self.statusBar)

        w = QtGui.QWidget()
        self.setCentralWidget(w)
        grid = QtGui.QGridLayout()
        w.setLayout(grid)

        # for some reason this achieves to force all elements to minimum size
        grid.setSizeConstraint(QtGui.QLayout.SetFixedSize)

        self.inputGroups = []
        fns = list(itertools.chain(*self.pp.getFieldNames()))
        for fn in fns:
            # create group box with vertical layout
            gb = QtGui.QGroupBox(w)
            gbLayout = QtGui.QGridLayout()
            gb.setLayout(gbLayout)

            # add label
            lb = QtGui.QLabel()
            lb.setText(fn)
            gbLayout.addWidget(lb)

            # add edit box
            #le = QtGui.QLineEdit()
            #le.setText(str(self.pp.getFieldByPath(fn)))
            #gbLayout.addWidget(le)

            # add spin box
            sb = QtGui.QDoubleSpinBox(w)
            sb.setObjectName(fn)
            sb.setSingleStep(0.01)
            sb.setMaximum(100)
            sb.setMinimum(-100)
            #sb.setDecimals(5)
            sb.setValue(self.pp.getFieldByPath(fn))
            sb.valueChanged.connect(lambda w, name = fn: self.spinBoxChanged(name, w))
            gbLayout.addWidget(sb)

            # add group to main grid layout
            grid.addWidget(gb)


        # TODO: load PB message from file and send it out completely

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.checkConnection)
        self.timer.start(1000)

    def checkConnection(self):
        self.statusBar.showMessage(self.s.connect(False))
        if self.s.connected:
            self.timer.stop()

    def spinBoxChanged(self, name, value):
        self.pp.setFieldByPath(name, value)
        try:
            bts = self.pp.message2bytes(self.pp.accumMsg)
            self.s.transmit(bts)
        except PbEncodeException as pbex:
            print "Error while encoding pb message: %s" % str(pbex)

    def saveValuesAsPbMsg(self):
        bts = self.pp.message2bytes(self.pp.accumMsg)
        with open(self.msgPath, 'wb') as f:
            f.write(bts)

    def quit(self):
        self.s.disconnect()
        self.saveValuesAsPbMsg()



if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    pbv = RemoteConfig(sys.argv)
    pbv.show()
    app.exec_()
    pbv.quit()
