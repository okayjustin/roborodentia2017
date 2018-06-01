#!/usr/local/bin/python3
"""
Opens a serial console with STM32 Nucleo-64 development board.

The MIT License (MIT)

Copyright (c) 2018 Justin Ng

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import serial
import serial.tools.list_ports
import time

class SerialConsole():
    def openSerial(self):
        port, success = self.get_STLink_port()
        if (success == False):
            print("Failed to find STLink port.")
            return -1

        self.ser = serial.Serial()
        self.ser.baudrate = 921600
        self.ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.ser.parity = serial.PARITY_NONE #set parity check: no parity
        self.ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        self.ser.timeout = 0.02            #non-block read
        self.ser.xonxoff = False     #disable software flow control
        self.ser.rtscts = False     #disable hardware (rts/cts) flow control
        self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
        self.ser.writeTimeout = 0.02     #timeout for write
        self.ser.port = port
        try:
            self.ser.close()
            self.ser.open()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser_available = True
            print("Connected.")
            return 0
        except serial.serialutil.SerialException:
            self.ser_available = False
            print("Failed to connect.")
            return -1

    def get_STLink_port(self):
        STLink_port = None
        success = False
        # Enumerate all serial ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
#            for element in port:
#                print(element)
#            print(port)

            # Check port string for identifier or VID:PID
            if (("STLink" in port[1]) or ("0483:374B" in port[2])):
                STLink_port = port[0]
                success = True

        return STLink_port, success

    def close(self):
        print("\r\rClosing port.")
        self.ser.close()
        self.ser_available = False

    def readSerial(self):
        try:
            while (1):
                print(self.ser.readline().decode('utf-8'), end='')
        except KeyboardInterrupt:
            pass

    def writeSerial(self):
        try:
            print("\r\rEnter text to send: ", end='')
            cmd = input()
            cmd += '\n'
            self.ser.write(cmd.encode('utf-8'))
            #self.ser.flush()
            return True
        except KeyboardInterrupt:
            return False

    def writeSerialSequence(self, cmd_seq):
        if (self.ser_available):
            for cmd in cmd_seq:
                self.ser.write(cmd.encode('utf-8'))




if __name__ == "__main__":
    console = SerialConsole()
    if (console.openSerial()):
        print("Quitting...")
        quit()
    serInUse = console.ser_available
    while(serInUse == True):
        console.readSerial()
        serInUse = console.writeSerial()
    console.close()


