#!/usr/bin/env python

import serial
import rospy
import sys
import struct as st
import binascii
import math

from time import time
from sensor_msgs.msg import Imu, Temperature, MagneticField


# BOSCH BNO055 IMU Registers map and other information
# Page 0 registers
ACCEL_DATA = 0x08

#  Operation modes
OPER_MODE_NDOF = 0x0C

# Communication constants
START_BYTE_WR = 0xaa
START_BYTE_RESP = 0x55
READ = 0x01


# Read data from IMU
def read_from_dev(ser, reg_addr, length):
    try:
        buf_in = bytearray(ser.read(length))
        #print("Reading, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
    except:
        return 0
    # Check if response is correct
    if (buf_in.__len__() != (length)) or (buf_in[0] != START_BYTE_RESP):
        #rospy.logerr("Incorrect Bosh IMU device response.")
        return 0
    return buf_in

def con(a,b):
    sum=int(a) | (int(b)<<8)
    if sum>0x8000:
        sum-=(1<<16)
    return sum

imu_data = Imu()            # Filtered data
imu_raw = Imu()             # Raw IMU data
temperature_msg = Temperature() # Temperature
mag_msg = MagneticField()       # Magnetometer data


# Main function
if __name__ == '__main__':
    rospy.init_node("imu_node")

    # Sensor measurements publishers
    pub_data = rospy.Publisher('imu/data', Imu, queue_size=1)
    pub_raw = rospy.Publisher('imu/raw', Imu, queue_size=1)

    # Get parameters values
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    frame_id = rospy.get_param('~frame_id', 'imu_link')
    frequency = rospy.get_param('frequency', 100)

    # Open serial port
    #rospy.loginfo("Opening serial port: %s...", port)
    try:
        ser = serial.Serial(port, 115200, timeout=0.02)
    except serial.serialutil.SerialException:
        rospy.logerr("IMU not found at port " + port + ". Check the port in the launch file.")
        sys.exit(0)

    rate = rospy.Rate(frequency)

    # Factors for unit conversions
    acc_fact = 1000.0
    gyr_fact = 1000.0
    seq = 0

    while not rospy.is_shutdown():
        rxBuffer = read_from_dev(ser, ACCEL_DATA, 24)
        if rxBuffer!=0 and rxBuffer[0]==0x55 and rxBuffer[1]==0xAA:
            sLen = con(rxBuffer[2],rxBuffer[3])
            sYaw = con(rxBuffer[4],rxBuffer[5])
            sPitch = con(rxBuffer[6],rxBuffer[7])
            sRoll = con(rxBuffer[8],rxBuffer[9])
            sGx =  con(rxBuffer[10],rxBuffer[11])
            sGy =  con(rxBuffer[12],rxBuffer[13])
            sGz =  con(rxBuffer[14],rxBuffer[15])
            sAx =  con(rxBuffer[16],rxBuffer[17])
            sAy =  con(rxBuffer[18],rxBuffer[19])
            sAz =  con(rxBuffer[20],rxBuffer[21])
            sCheck = con(rxBuffer[22],rxBuffer[23])%(1<<16)
            checkSum = (sLen + sYaw + sPitch + sRoll + sGx +sGy + sGz + sAx + sAy +sAz)%(1<<16)
            if sCheck != checkSum:
                rospy.logerr("Check error")
            yaw = float(sYaw) / 100.0
            roll = float(sYaw) / 100.0 
            pitch = float(sYaw) / 100.0
            cosRoll = math.cos(roll * 0.5);
            sinRoll = math.sin(roll * 0.5)

            cosPitch = math.cos(pitch * 0.5)
            sinPitch = math.sin(pitch * 0.5)

            cosHeading = math.cos(yaw * 0.5)
            sinHeading = math.sin(yaw * 0.5)

            w = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
            x = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
            y = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
            z = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading; 
            
            # Publish raw data
            imu_raw.header.stamp = rospy.Time.now()
            imu_raw.header.frame_id = frame_id
            imu_raw.header.seq = seq
            imu_raw.orientation_covariance[0] = -1
            imu_raw.linear_acceleration.x = float(sAx) / acc_fact*9.8
            imu_raw.linear_acceleration.y = float(sAy) / acc_fact*9.8
            imu_raw.linear_acceleration.z = float(sAz) / acc_fact*9.8
            imu_raw.linear_acceleration_covariance[0] = -1
            imu_raw.angular_velocity.x = float(sGx) / gyr_fact
            imu_raw.angular_velocity.y = float(sGy) / gyr_fact
            imu_raw.angular_velocity.z = float(sGz) / gyr_fact
            imu_raw.angular_velocity_covariance[0] = -1
            pub_raw.publish(imu_raw)

            # Publish filtered data
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = frame_id
            imu_data.header.seq = seq
            imu_data.orientation.w = float(w)
            imu_data.orientation.x = float(x)
            imu_data.orientation.y = float(y)
            imu_data.orientation.z = float(z)
            imu_data.linear_acceleration.x = float(sAx) / acc_fact*9.8
            imu_data.linear_acceleration.y = float(sAy) / acc_fact*9.8
            imu_data.linear_acceleration.z = float(sAz) / acc_fact*9.8
            imu_data.linear_acceleration_covariance[0] = -1
            imu_data.angular_velocity.x = float(sGx) / gyr_fact
            imu_data.angular_velocity.y = float(sGy) / gyr_fact
            imu_data.angular_velocity.z = float(sGz) / gyr_fact
            imu_data.angular_velocity_covariance[0] = -1
            pub_data.publish(imu_data)

            seq = seq + 1
        rate.sleep()
    ser.close()
