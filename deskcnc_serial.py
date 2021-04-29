#!/usr/bin/env python3

# HoFaLab DeskCNC interface
# Author: Piet Jarmatz
# Date: June 2019

import serial
import argparse, sys, os
import time
import numpy as np
import logging
import pygcode

log = logging.getLogger('deskcnc_serial')
log.setLevel(level=logging.INFO)

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
log.addHandler(handler)

# already contains checksum
COM_RESET = b'@R\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x01\x00\x00\x00\x00\x00\x12'
MIN_FEEDRATE = 62500
MAX_FEEDRATE = 1250

# XOR over all bytes of command
def checksum(command):
    cs = 0
    for b in command:
        cs = cs ^ b
    return bytes([cs])

# input: value in mm
# output: little endian 4 Byte value in deskCNC units
def encodeValueMM(val):
    numsteps = int(val / 0.015)   # 1 step = 0.015 mm
    return (numsteps).to_bytes(4, byteorder="little", signed=True)

# input: value in mm per minute
# output: little endian 4 Byte value in deskCNC units
def encodeFeedrate(val):
    num = int(3378400 / val)
    num = int(np.clip(num, MAX_FEEDRATE, MIN_FEEDRATE))
    return (num).to_bytes(2, byteorder="little", signed=False)

# talks to the device on the serial port
class DeskCNC():
    def __init__(self, device):
        self._ser = serial.Serial()
        self._ser.port = device
        self._ser.baudrate = 115200
        self._ser.timeout = 1
        self.restart_device()

    # low level device reset without initialisation
    def reset(self):
        self._ser.write(COM_RESET)
        log.debug("Sent reset command")
        time.sleep(0.05)
        response = self._ser.read(self._ser.inWaiting())
        log.debug("Response: " + str(response))
        return response[-5:] == b'\xFB\x32\x32\x34\xF5'

    # sends file and compares response with another file
    def send_firmware(self):
        filename = 'firmware.txt'
        filename2 = 'firmware_response.txt'
        fw = b''
        resp_expected = b''

        log.debug('Opening firmware file ' + filename)
        with open(filename, 'r') as f:
            for line in f:
                for byte in line.split():
                    fw += bytes([int(byte, 16)])

        log.debug('Opening expected response file ' + filename2)
        with open(filename2, 'r') as f2:
            for line in f2:
                for byte in line.split():
                    resp_expected += bytes([int(byte, 16)])

        log.info('Sending firmware...')
        response = b''
        for i in range(0, len(fw), 64):
        	self._ser.write(fw[i:i+64])
        	time.sleep(0.005)
        	response += self._ser.read(self._ser.inWaiting())
        time.sleep(0.05)
        response += self._ser.read(self._ser.inWaiting())
        
        if response == resp_expected:
            log.debug('Correct response')
            return True
        else:
            log.error('Wrong response to firmware download!')
            return False

    # complete reset of device incl. all initialisations
    def restart_device(self):
        log.info('Starting device')
        self._ser.close() 
        log.info('Trying to open port ' + self._ser.port)
        while not self._ser.is_open:
            try:
                self._ser.open()
                log.info('Successfully opened ' + self._ser.port) 
                log.info('Trying to reset device')
                while not self.reset():
                    pass
                while not self.send_firmware():
                    pass
            except (serial.SerialException, OSError):
                self._ser.close() 
                time.sleep(2)  
        log.info('Ready')

    def machineReady(self):
        command = b'\x40\x57'
        command += bytearray(14)
        command += b'\x01\x01'
        command += bytearray(5)
        msg = command + checksum(command)
        self._ser.write(msg)
        log.debug("Sent: " + msg.hex())
        response = self._ser.read(18)
        log.debug("Response: " + str(response))
        if response == b'':
            raise serial.SerialException("device not responding")
        # (response contains x y z positions, throw away)
        return response[-1:] == b'\xEC'

    def executeCommand(self, command, count = 0):
        msg = command + checksum(command)
        self._ser.write(msg)
        log.debug("Sent: " + msg.hex())
        response = self._ser.read(1)
        log.debug("Response: " + str(response))
        while not self.machineReady():
            time.sleep(0.001)
        if not response == b'\x02':
            if count < 2:
                self.executeCommand(command, count + 1)
            else:
                raise serial.SerialException("wrong response from device")

    # rapid positioning
    def G0(self, posX, posY, posZ):
        command = b'\x40\x4E'
        command += encodeValueMM(-posX)
        command += encodeValueMM(posY)
        command += encodeValueMM(posZ)
        command += encodeFeedrate(float("inf"))
        command += b'\xDB\x00'   # TODO what does this mean?
        command += bytearray(5)
        self.executeCommand(command)

    # linear interpolation
    def G1(self, posX, posY, posZ, feedrate=300):
        command = b'\x40\x4D'
        command += encodeValueMM(-posX)
        command += encodeValueMM(posY)
        command += encodeValueMM(posZ)
        command += encodeFeedrate(feedrate)
        command += bytearray(7)
        self.executeCommand(command)

    # circular interpolation Clockwise
    def G2(self):
        log.error("G2 not implemented")
        raise NotImplementedError 

    # circular interpolation Counter-Clockwise
    def G3(self):
        log.error("G3 not implemented")
        raise NotImplementedError

    # xy plane selection (for circular interpolation / arc cutting)
    def G17(self):
        log.info("G17 is default mode")

    # xz plane selection (for circular interpolation / arc cutting)
    def G18(self):
        log.error("G18 not implemented")
        raise NotImplementedError 

    # yz plane selection (for circular interpolation / arc cutting)
    def G19(self):
        log.error("G19 not implemented")
        raise NotImplementedError

    # inch system selection
    def G20(self):
        log.error("G20 not implemented")
        raise NotImplementedError

    # millimeter system selection
    def G21(self):
        log.info("G21 is default mode") 

    # absolute distance mode
    def G90(self):
        log.info("G90 is default mode") 

    # incremental distance mode
    def G91(self):
        log.error("G91 not implemented")
        raise NotImplementedError

    # feed per minute mode
    def G94(self):
        log.info("G94 is default mode") 

    def spindle(self, state):
        command = b'\x40\x4F'
        command += bytearray(14)
        command += b'\x02'
        command += state
        command += bytearray(5)
        res = self.executeCommand(command)
        time.sleep(2)
        return res

    # Spindle on Clockwise
    def M3(self):
        return self.spindle(b'\x01')

    # Spindle on Counter Clockwise
    def M4(self):
        log.error("M4 not supported by hardware")
        return self.spindle(b'\x01')

    # Spindle Off
    def M5(self):
        return self.spindle(b'\x00')

    # Coolant Off
    def M9(self):
        log.error("M9 not supported by hardware")

# Reads gcode and keeps modal state
class ModalGCodeParser():
    def __init__(self, mill, infile):
        self.device = mill
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.feedrate = 300
        self.mode = pygcode.gcodes.GCodeLinearMove
        self.input = infile
        self.offsetX = 0
        self.offsetY = 0
        self.offsetZ = 0
        self.spindleOn = False
        self.absolutepositionmode = True

    def applyWord(self, word):
        if self.absolutepositionmode:
            if(word.letter == 'X'):
                self.posX = word.value
            if(word.letter == 'Y'):
                self.posY = word.value
            if(word.letter == 'Z'):
                self.posZ = word.value
            if(word.letter == 'F'):
                self.feedrate = word.value
        else:
            if(word.letter == 'X'):
                self.posX += word.value
            if(word.letter == 'Y'):
                self.posY += word.value
            if(word.letter == 'Z'):
                self.posZ += word.value
            if(word.letter == 'F'):
                self.feedrate = word.value

    def executeGCode(self, gcode):
        try:
            for word in gcode.params.values():
                self.applyWord(word)

            if type(gcode) is pygcode.gcodes.GCodeFeedRate:
                self.applyWord(gcode.word)

            elif type(gcode) is pygcode.gcodes.GCodeRapidMove:
                self.device.G0(self.posX + self.offsetX, self.posY + self.offsetY, self.posZ + self.offsetZ)
                self.mode = type(gcode)
            elif type(gcode) is pygcode.gcodes.GCodeLinearMove:
                if(self.feedrate > 1620):
                    log.error("GCodeLinearMove: feedrate too high for hardware interpolation, limited to 1620 mm/min")
                    self.feedrate = 1620
                self.device.G1(self.posX + self.offsetX, self.posY + self.offsetY, self.posZ + self.offsetZ, self.feedrate)
                self.mode = type(gcode)

            elif type(gcode) is pygcode.gcodes.GCodeArcMoveCW:
                self.device.G2()
                self.mode = type(gcode)
            elif type(gcode) is pygcode.gcodes.GCodeArcMoveCCW:
                self.device.G3()
                self.mode = type(gcode)

            elif type(gcode) is pygcode.gcodes.GCodeSelectXYPlane:
                self.device.G17()
            elif type(gcode) is pygcode.gcodes.GCodeSelectZXPlane:
                self.device.G18()
            elif type(gcode) is pygcode.gcodes.GCodeSelectYZPlane:
                self.device.G19()

            elif type(gcode) is pygcode.gcodes.GCodeUseInches:
                self.device.G20()
            elif type(gcode) is pygcode.gcodes.GCodeUseMillimeters:
                self.device.G21()

            elif type(gcode) is pygcode.gcodes.GCodeAbsoluteDistanceMode:
                self.absolutepositionmode = True
                #self.device.G90()
            elif type(gcode) is pygcode.gcodes.GCodeIncrementalDistanceMode:
                self.absolutepositionmode = False
                #self.device.G91()

            elif type(gcode) is pygcode.gcodes.GCodeUnitsPerMinuteMode:
                self.device.G94()

            elif type(gcode) is pygcode.gcodes.GCodeStartSpindleCW:
                self.device.M3()
                self.spindleOn = True
            elif type(gcode) is pygcode.gcodes.GCodeStartSpindleCCW:
                self.device.M4()
                self.spindleOn = True
            elif type(gcode) is pygcode.gcodes.GCodeStopSpindle:
                self.device.M5()
                self.spindleOn = False
            elif type(gcode) is pygcode.gcodes.GCodeCoolantOff:
                self.device.M9()

            else:
                log.error("Unsupported gcode " + str(gcode))
        except serial.SerialException as err:
            log.error("Communication error " + str(err))
            self.device.restart_device()
            self.offsetX = -self.posX
            self.offsetY = -self.posY
            self.offsetZ = -self.posZ
            if self.spindleOn:
                self.device.M3()
            self.executeGCode(gcode)

    def parseLine(self, line_text):
        line = pygcode.Line(line_text)
        for word in line.block.modal_params:
            self.applyWord(word)
        for gcode in sorted(line.block.gcodes):
            self.executeGCode(gcode)
        if len(line.block.gcodes) == 0:
            self.executeGCode(self.mode())

    def run(self):
        try:
            for line_text in self.input:
                self.parseLine(line_text)
        except KeyboardInterrupt:
            pass

def main():
    try:
        infile = sys.stdin

        if len(sys.argv) < 2 or len(sys.argv) > 3:
            log.error("Usage: deskcnc_serial.py serialport [gcode-file]")
            sys.exit(1)

        # note: you may have to call 'sudo service easel-driver stop' before this
        mill = DeskCNC(sys.argv[1])

        if len(sys.argv) == 3:
            infile = open(sys.argv[2], 'r')
        parser = ModalGCodeParser(mill, infile)
        parser.run()

    except serial.serialutil.SerialException as err:
        log.error(str(err))
    finally:
        infile.close()

if __name__ == '__main__':
    main()
