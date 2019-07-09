import serial
import sys

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.02)
except serial.serialutil.SerialException:
    print("IMU not found at port " + '/dev/ttyUSB0' + ". Check the port in the launch file.")
    sys.exit(0)
#buf_out="AT+EOUT=0\r\n"
buf_out="AT+SETPTL=A5,B0,C0,D0,D1\r\n"
ser.write(buf_out)
