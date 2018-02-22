#!/usr/local/bin/python3

import serial


def openSerial():
    ports = ['/dev/tty.usbmodem1413', '/dev/tty.usbmodem1423']
    ser = serial.Serial()
    ser.baudrate = 921600
    ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    ser.parity = serial.PARITY_NONE #set parity check: no parity
    ser.stopbits = serial.STOPBITS_ONE #number of stop bits
    ser.timeout = 100            #non-block read
    ser.xonxoff = False     #disable software flow control
    ser.rtscts = False     #disable hardware (rts/cts) flow control
    ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
    ser.writeTimeout = 1     #timeout for write

    for port in ports:
        ser.port = port
        try:
            ser.close()
            ser.open()
            ser.reset_input_buffer()
            print("Connected.")
            return ser
        except serial.serialutil.SerialException:
            pass
    print("Failed to connect.")

def closeSerial(ser):
    ser.close()
    ser_available = False

def readSerial(ser):
    try:
        while (1):
            print(ser.readline())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    ser = openSerial()
    readSerial(ser)
    closeSerial(ser)


