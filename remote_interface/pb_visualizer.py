#!/usr/bin/env python

import sys, serial, time
import copcom_pb2

import numpy as np
from PyQt4 import QtCore, QtGui, uic
import pyqtgraph as pg
from pyqtgraph.ptime import time as pTime
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType
import pyqtgraph.parametertree.parameterTypes as pTypes

from serial_interface import *


form_class = uic.loadUiType("plot.ui")[0]


class GraphicsLayoutWidget:
    def __init__(self, widget):
        # create plot window
        self.w = widget
        self.plots = []
        self.labels = []

        self.data = np.zeros((0,0)) # a matrix containing data for each dimension per row

        # create timer to update plots
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updatePlotData)
        self.timer.start(100)


    def updateNumOfPlots(self):
        numDims = len(self.data)
        while len(self.plots) < numDims:
            self.plots.append(self.w.addPlot())
            self.plots[-1].plot()
            self.plots[-1].showGrid(True, True)
            self.w.nextRow()

        self.updatePlotLabels()


    def updatePlotData(self):
        for i in range(len(self.plots)):
            self.plots[i].listDataItems()[0].setData(self.data[i,:])
        app.processEvents()  ## force complete redraw for every plot


    def addData(self, data, maxLen = 0):
        nd = np.array(data).reshape(len(data),1)

        if self.data.shape[0] == 0:
            self.data = nd
        else:
            # if the new data vector is longer, create a new row in the data field
            if nd.shape[0] > self.data.shape[0]:
                diff = nd.shape[0] - self.data.shape[0]
                self.data = np.vstack((self.data, np.zeros(diff, self.data.shape[1])))

            self.data = np.hstack((self.data, nd))

        if maxLen is not 0 and self.data.shape[1] > maxLen:
            self.data = self.data[:,-maxLen:];

        self.updateNumOfPlots()

    def setYLabels(self, labels):
        self.labels = labels
        self.updatePlotLabels()

    def updatePlotLabels(self):
        # assign y axis labels (if more/less labels are given, list is truncated accordingly)
        for p,l in zip(self.plots, self.labels):
            p.setLabel('left', l)


class ProtobufVisualizer(QtGui.QMainWindow, form_class):
    def __init__(self, args, parent=None):
        QtGui.QMainWindow.__init__(self,parent)
        self.setupUi(self)

        self.s = SerialInterface('/dev/ttyACM0')
        self.s.connect()

        self.pp = ProtobufParser(copcom_pb2.PbCopterState)

        self.glw = GraphicsLayoutWidget(self.plotView)
        self.fieldNames = self.pp.getFieldNames()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

    """
    def setActiveFields(self, fields):
        if not set(self.fieldNames).subset(set(fields)):
            print "Error! Specified field not in list of available fields: " + str(self.fieldnames)

        self.fields = fields
        self.glw.setYLabels(fields)
    """

    def update(self):
        self.s.receive()
        while True:
            pkgStr = self.s.popPacket()
            if not pkgStr:
                break

            pbPkg = self.pp.bytes2message(pkgStr)
            o = pbPkg.orientation
            r = pbPkg.refOrientation
            c = pbPkg.controls
            m = pbPkg.motorSpeed
            a = pbPkg.accl
            #self.glw.addData([a.x, a.y, a.z], 100)
            self.glw.addData([o.w, o.x, o.y, o.z, c.w, c.x, c.y, c.z, r.w, r.x, r.y, r.z, m.w, m.x, m.y, m.z], 100)
            #self.glw.addData([o.w, o.x, o.y, o.z, c.w, c.x, c.y, c.z, m.w, m.x, m.y, m.z, a.x, a.y, a.z], 100)

            """
            data = []
            for fn in self.fields:
                f = pbPkg.getattr('fn')
                data.append([f.w, f.x, f.y, f.z])

            self.glw.addData(data, 100)
            """




if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    pbv = ProtobufVisualizer(sys.argv)
    #pbv.setActiveFields(['controls', 'refOrientation'])
    pbv.show()
    app.exec_()
