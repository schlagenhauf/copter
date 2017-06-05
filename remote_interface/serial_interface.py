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
        self.baud = 115200
        self.bytesize = serial.SEVENBITS
        self.connected = False
        self.data = ""
        self.header = chr(0xa3) * 3
        self.ser = None

    def __del__(self):
        if self.ser and self.connected:
            self.ser.close()

    def connect(self, tryRepeatedly = True):
        # establish connection
        while True:
            try:
                self.ser = serial.Serial(self.comport, self.baud, self.bytesize)
                self.connected = True
                return "Connected to " + self.comport
            except serial.SerialException:
                if not tryRepeatedly:
                    return 'Waiting for device ' + self.comport + ' to be available.'
                else:
                    print('Waiting for device ' + self.comport + ' to be available.')
                    time.sleep(3)

    def disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connected = False

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

    def transmit(self, message):
        if not self.connected:
            print "Couldn't send message. Not Connected."
        else:
            self.ser.write(bytes(message))



class ProtobufParser:
    def __init__(self, messageCls):
        self.messageCls = messageCls
        self.accumMsg = self.messageCls()

    # parse a list of bytes to fill the protobuf message
    def bytes2message(self, data):
        data = ''.join(data)
        inst = self.messageCls() # create instance from message class
        inst.ParseFromString(data)
        return inst

    def message2bytes(self, message):
        return message.SerializeToString()

    def setFieldByPath(self, path, value, msg = None):
        if not msg:
            msg = self.accumMsg
        fields = path.split('.')
        curField = msg
        for f in fields[:-1]:
            curField = getattr(curField, f)

        setattr(curField, fields[-1], value)

        return self.accumMsg

    def getFieldByPath(self, path, msg = None):
        if not msg:
            msg = self.accumMsg
        fields = path.split('.')
        curField = msg
        for f in fields[:-1]:
            curField = getattr(curField, f)

        return getattr(curField, fields[-1])



    def getFieldNames(self, msg = None):
        if msg is None:
            msg = self.messageCls()
        return [f.name if f.type != f.TYPE_MESSAGE else \
                [f.name + '.' + sn for sn in self.getFieldNames(getattr(msg, f.name))] \
                for f in msg.DESCRIPTOR.fields]
