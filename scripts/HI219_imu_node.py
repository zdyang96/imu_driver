#!/usr/bin/env python

import serial
import rospy
import sys
import struct as st
import binascii
import math

from time import time
from sensor_msgs.msg import Imu, Temperature, MagneticField
from tf.transformations import euler_from_quaternion


# BOSCH BNO055 IMU Registers map and other information
# Page 0 registers

#  Operation modes
OPER_MODE_NDOF = 0x0C

# Communication constants
START_BYTE_WR = 0xA5
START_BYTE_RESP = 0x5A
READ = 0x01


# Read data from IMU
def read_from_dev(ser, length):
    try:
        buf_in = bytearray(ser.read(length))
        #print("Reading, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
    except:
        #rospy.logerr("response.")
        return 0
    # Check if response is correct
    # for i in range(100):
    #     try:
    #         print(i,hex(buf_in[i]))
    #     except:
    #         pass
    if (buf_in.__len__() != (length)):
        # rospy.logerr("Incorrect Bosh IMU device response.")
        return 0
    return buf_in

def con(a,b):
    return float(st.unpack('<h', st.pack('BB',a,b))[0])


imu_data = Imu()
magneticField_data=MagneticField()

# Main function
if __name__ == '__main__':
    rospy.init_node("imu_node")

    # Sensor measurements publishers
    pub_data = rospy.Publisher('imu/data', Imu, queue_size=1)
    pub_magneticfield = rospy.Publisher('magneticfield/data',MagneticField , queue_size=1)

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
    gyr_fact = 10.0
    seq = 0
    buf=0
    buf_out="AT+SETPTL=A5,B0,C0,D0,D1\r\n"
    ser.write(buf_out)
    rate.sleep()
    while not rospy.is_shutdown():
        while (buf==0 or buf[0]!=0x5A):
            buf=read_from_dev(ser, 1)
        buf=read_from_dev(ser, 1)
        if buf!=0 and buf[0]==0xA5:
            buf=read_from_dev(ser, 4)   
            if buf==0:
                continue
            Len=con(buf[0],buf[1])
            CRC=con(buf[2],buf[3])
            buf=read_from_dev(ser, 64)
            if buf==0:
                continue
            offset=0
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = frame_id
            imu_data.header.seq = seq
            magneticField_data.header.stamp = rospy.Time.now()
            magneticField_data.header.frame_id = frame_id
            magneticField_data.header.seq = seq
            while offset<Len and offset<47: 
                if  buf[offset]==0x90:
                    id=buf[offset+1]
                    offset+=2
                elif buf[offset]==0xA0:
                    imu_data.linear_acceleration.x=con(buf[offset+1],buf[offset+2])*0.0098
                    imu_data.linear_acceleration.y=con(buf[offset+3],buf[offset+4])*0.0098
                    imu_data.linear_acceleration.z=con(buf[offset+5],buf[offset+6])*0.0098
                    imu_data.linear_acceleration_covariance[0] = 0
                    offset+=7
                elif buf[offset]==0xA5:
                    offset+=7
                elif buf[offset]==0xB0:
                    imu_data.angular_velocity.x=con(buf[offset+1],buf[offset+2])/10.0
                    imu_data.angular_velocity.y=con(buf[offset+3],buf[offset+4])/10.0
                    imu_data.angular_velocity.z=con(buf[offset+5],buf[offset+6])/10.0
                    imu_data.angular_velocity_covariance[0] = 0
                    offset+=7
                elif buf[offset]==0xC0:
                    magneticField_data.magnetic_field.x=con(buf[offset+1],buf[offset+2])/1000.0
                    magneticField_data.magnetic_field.y=con(buf[offset+3],buf[offset+4])/1000.0
                    magneticField_data.magnetic_field.z=con(buf[offset+5],buf[offset+6])/1000.0
                    magneticField_data.magnetic_field_covariance[0]=0
                    offset+=7
                elif buf[offset]==0xD0:
                    # print("euler")
                    # print(con(buf[offset+1],buf[offset+2])/100/180*3.14,con(buf[offset+3],buf[offset+4])/100/180*3.14,con(buf[offset+5],buf[offset+6])/10/180*3.14)
                    offset+=7
                elif buf[offset]==0xD9:
                    offset+=13
                elif buf[offset]==0xD1:
                    imu_data.orientation.w = st.unpack('<f', st.pack('BBBB', buf[offset+1], buf[offset+2], buf[offset+3], buf[offset+4]))[0]
                    imu_data.orientation.x = st.unpack('<f', st.pack('BBBB', buf[offset+5], buf[offset+6], buf[offset+7], buf[offset+8]))[0]
                    imu_data.orientation.y = st.unpack('<f', st.pack('BBBB', buf[offset+9], buf[offset+10], buf[offset+11], buf[offset+12]))[0]
                    imu_data.orientation.z = st.unpack('<f', st.pack('BBBB', buf[offset+13], buf[offset+14], buf[offset+15], buf[offset+16]))[0]
                    imu_data.orientation_covariance[0] = 0
                    offset+=17
                    # print("quate")
                    # print(euler_from_quaternion([imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w]))
                elif buf[offset]==0xF0:
                    offset+=5
                else:
                    offset+=1
                    Len+=1
            pub_data.publish(imu_data)
            pub_magneticfield.publish(magneticField_data)
            seq = seq + 1
        rate.sleep()
    ser.close()
