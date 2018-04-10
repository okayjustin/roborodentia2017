#!/usr/local/bin/python3

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
            #print(port)
            # Check port string for identifier
            if ("STLink" in port[1]):
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
            cmd += '\r\n'
            for character in cmd:
                self.ser.write(character.encode('utf-8'))
                time.sleep(0.1)
            self.ser.flush()
            return True
        except KeyboardInterrupt:
            return False

    def writeSerialSequence(self, cmd_seq):
        if (self.ser_available):
            for cmd in cmd_seq:
                self.ser.write(cmd.encode('utf-8'))




if __name__ == "__main__":
    console = SerialConsole()
    serInUse = True
    while(serInUse == True):
        console.readSerial()
        serInUse = console.writeSerial()
    console.close()


