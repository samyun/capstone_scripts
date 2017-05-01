#! /usr/bin/env python

# Read the output of an Arduino which may be printing sensor output,
# and at the same time, monitor the user's input and send it to the Arduino.
# See also


import sys, serial, time, datetime


def flush_lines(ser):
    ser.flushOutput()
    ser.flushInput()


def init_serial(baud=9600):
    # Port may vary, so look for it:
    baseports = ['/dev/ttyUSB', '/dev/ttyACM']
    ser = None
    for baseport in baseports:
        if ser : break
        for i in range(0, 8):
            try:
                port = baseport + str(i)
                ser = serial.Serial(port, baud, timeout=1)
                print(("Opened", port))
                break
            except :
                ser = None
                pass

    if not ser :
        raise RuntimeError("Couldn't open a serial port")


    ser.write_timeout = 1
    ser.timeout = 1

    # wait for initial arduino message
    flush_lines(ser)

    # tare sensors
    input("Ready to tare? Enter a key with no load")
    print('Tareing...')
    for x in range(0, 9):
        ser.readline().strip()
        time.sleep(.1)
    # send config command
    ser.write(bytes("x", 'UTF-8'))
    for x in range(0, 20):
        ser.readline().strip()
        time.sleep(.1)
    # wait for messages
    flush_lines(ser)
    # send tare command
    ser.write(bytes("1", 'UTF-8'))
    for x in range(0, 20):
        ser.readline().strip()
        time.sleep(.1)
    # wait for messages
    flush_lines(ser)
    # exit config
    ser.write(bytes("x", 'UTF-8'))
    for x in range(0, 3):
        ser.readline().strip()
        time.sleep(.1)
    # wait for messages
    flush_lines(ser)

    return ser


def run(ser, data):
    # Send characters:
    ser.write(bytes(data, 'UTF-8'))

    # check for serial output:
    return ser.readline().strip()

try:
    serial_port = init_serial()

    while True:
        input("enter")
        output = run(serial_port, "0")
        print(output.decode("UTF-8")[:-5])
except serial.SerialException:
    print("Disconnected (Serial exception)")
except IOError:
    print("Disconnected (I/O Error)")
except KeyboardInterrupt:
    print("Interrupt")
