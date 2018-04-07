#!/usr/local/bin/python3

import serial
import serial.tools.list_ports
import time

def get_STLink_port():
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

def openSerial():
    port, success = get_STLink_port()
    if (success == False):
        print("Failed to find STLink port. Quitting.")

    ser = serial.Serial()
    ser.baudrate = 921600
    ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    ser.parity = serial.PARITY_NONE #set parity check: no parity
    ser.stopbits = serial.STOPBITS_ONE #number of stop bits
    ser.timeout = 100            #non-block read
    ser.xonxoff = False     #disable software flow control
    ser.rtscts = False     #disable hardware (rts/cts) flow control
    ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
    ser.writeTimeout = 100     #timeout for write
    ser.port = port
    try:
        ser.close()
        ser.open()
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Connected.")
        return ser
    except serial.serialutil.SerialException:
        pass
    print("Failed to connect.")

def closeSerial(ser):
    print("\r\rClosing port.")
    ser.close()
    ser_available = False

def readSerial(ser):
    try:
        while (1):
            print(ser.readline().decode('utf-8'), end='')
    except KeyboardInterrupt:
        pass

def writeSerial(ser):
    try:
        print("\r\rEnter text to send: ", end='')
        cmd = input()
        cmd += '\r\n'
        for character in cmd:
            ser.write(character.encode('utf-8'))
            time.sleep(0.1)
        ser.flush()
        return True
    except KeyboardInterrupt:
        return False

if __name__ == "__main__":
    ser = openSerial()
    serInUse = True
    while(serInUse == True):
        readSerial(ser)
        serInUse = writeSerial(ser)
    closeSerial(ser)


