import serial
import time
import struct
from vector import *
from quaternion import *
import copcom_pb2


class SerialInterface(object):
    """
    Connects to a microcontroller and does stuff.
    """

    def __init__(self, comport):
        self.comport = comport
        self.baud = 9600
        self.bytesize = serial.SEVENBITS
        self.connected = False
        self.data = ""
        self.header = chr(0xa3) * 3

    def __del__(self):
        self.ser.close()

    def connect(self):
        # establish connection
        while True:
            try:
                self.ser = serial.Serial(self.comport, self.baud, self.bytesize)
                self.connected = True
                print "Connected to " + self.comport

                break
            except serial.SerialException:
                print('Waiting for device ' + self.comport + ' to be available.')
                time.sleep(3)

    def receive(self):
        sizePre = len(self.data)
        # if there is something at the serial port
        if self.connected and self.ser.in_waiting != 0:
            # starttime = time.time()
            # while (timeout is not 0) and (time.time() - starttime) <=
            # 1000*timeout:
            self.data += self.ser.read(self.ser.in_waiting)

        return len(self.data) - sizePre

    def popPacket(self):
        # find next block via the header sequence
        startIdx = self.data.find(self.header)

        if startIdx == -1:
            # buffer only contains garbage, delete everything
            self.data = ""
            return None
        elif startIdx != 0:
            # header is there, but there is garbage in front
            self.data = self.data[startIdx:]

        # remove header
        self.data = self.data[len(self.header):]
        endIdx = self.data.find(self.header)

        packet = []
        if endIdx == -1:
            packet = self.data
            self.data = ""
        else:
            packet = self.data[:endIdx]
            self.data = self.data[endIdx:]

        return packet

    def transmit(self, command):
        self.ser.write(bytes(command, encoding='ascii'))

    def toString(self):
        # return '\t'.join(['Accl: ' + str(self.accl), 'Gyro: ' + str(self.gyro), 'Mag: ' + str(self.mag)])
        return 'Quaternion rotation: ' + str(self.quat.rotAngleInDeg())


class ProtobufParser:
    def __init__(self, message):
        self.message = message
        #self.message = copcom_pb2.PbCopterState()

    # parse a list of bytes to fill the protobuf message
    def __call__(self, data):
        # print ''.join(format(ord(x), '02x') for x in data)
        data = ''.join(data)
        self.message.ParseFromString(data)
        return self.message

    def getFieldNames(self, msg = None):
        if msg is None:
            msg = self.message
        return [f.name if f.type != f.TYPE_MESSAGE else \
                [f.name + '.' + sn for sn in self.getFieldNames(getattr(msg, f.name))] \
                for f in msg.DESCRIPTOR.fields]
