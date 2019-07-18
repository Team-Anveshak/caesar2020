#!/usr/bin/env python


import rospy
import serial
import string
import math
import sys


from sensors.msg import Imu
from tf.transformations import quaternion_from_euler

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0
imu_pitch_calibration = 0.0

rospy.init_node("razor_node")
pub = rospy.Publisher('imu', Imu, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsg = Imu()

default_port='/dev/imu'
port = rospy.get_param('/imu_port', default_port)


#accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)



# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=57600, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

roll=0
pitch=0
yaw_deg=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
rospy.loginfo("Giving the razor IMU board 4 seconds to boot...")
rospy.sleep(4) # Sleep for 5 seconds to wait for the board to boot
'''
ser.write('#o0' + chr(13))

discard = ser.readlines()

#set output mode
ser.write('#ox' + chr(13)) # To start display angle and sensor reading in text
'''
rospy.loginfo("Writing calibration values to razor IMU board...")
#set calibration values
ser.write('#caxm' + str(accel_x_min) + chr(13))
ser.write('#caxM' + str(accel_x_max) + chr(13))
ser.write('#caym' + str(accel_y_min) + chr(13))
ser.write('#cayM' + str(accel_y_max) + chr(13))
ser.write('#cazm' + str(accel_z_min) + chr(13))
ser.write('#cazM' + str(accel_z_max) + chr(13))

if (not calibration_magn_use_extended):
    ser.write('#cmxm' + str(magn_x_min) + chr(13))
    ser.write('#cmxM' + str(magn_x_max) + chr(13))
    ser.write('#cmym' + str(magn_y_min) + chr(13))
    ser.write('#cmyM' + str(magn_y_max) + chr(13))
    ser.write('#cmzm' + str(magn_z_min) + chr(13))
    ser.write('#cmzM' + str(magn_z_max) + chr(13))
else:
    ser.write('#ccx' + str(magn_ellipsoid_center[0]) + chr(13))
    ser.write('#ccy' + str(magn_ellipsoid_center[1]) + chr(13))
    ser.write('#ccz' + str(magn_ellipsoid_center[2]) + chr(13))
    ser.write('#ctxX' + str(magn_ellipsoid_transform[0][0]) + chr(13))
    ser.write('#ctxY' + str(magn_ellipsoid_transform[0][1]) + chr(13))
    ser.write('#ctxZ' + str(magn_ellipsoid_transform[0][2]) + chr(13))
    ser.write('#ctyX' + str(magn_ellipsoid_transform[1][0]) + chr(13))
    ser.write('#ctyY' + str(magn_ellipsoid_transform[1][1]) + chr(13))
    ser.write('#ctyZ' + str(magn_ellipsoid_transform[1][2]) + chr(13))
    ser.write('#ctzX' + str(magn_ellipsoid_transform[2][0]) + chr(13))
    ser.write('#ctzY' + str(magn_ellipsoid_transform[2][1]) + chr(13))
    ser.write('#ctzZ' + str(magn_ellipsoid_transform[2][2]) + chr(13))

ser.write('#cgx' + str(gyro_average_offset_x) + chr(13))
ser.write('#cgy' + str(gyro_average_offset_y) + chr(13))
ser.write('#cgz' + str(gyro_average_offset_z) + chr(13))
'''
#print calibration values for verification by user
ser.flushInput()
ser.write('#p' + chr(13))
calib_data = ser.readlines()
calib_data_print = "Printing set calibration values:\r\n"
for line in calib_data:
    calib_data_print += line
rospy.loginfo(calib_data_print)

#start datastream
ser.write('#o1' + chr(13))
'''

rospy.loginfo("Flushing first 100 IMU entries...")
for x in range(0, 100):
    line = ser.readline()

rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

while not rospy.is_shutdown():
    line = ser.readline()
    lines = line.replace("#YPR=","")   # Delete "#YPRAG="
    words = string.split(lines,",")    # Fields split
    if len(words) > 2:
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        try:

            yaw_deg = float(words[0])+180
            yaw_deg = yaw_deg + imu_yaw_calibration
            if yaw_deg > 180.0:
                yaw_deg = yaw_deg - 360.0
            if yaw_deg < -180.0:
                yaw_deg = yaw_deg + 360.0

            rospy.loginfo("word0 %s" % (yaw_deg))
            yaw = yaw_deg*degrees2rad
            pitch = float(words[1])*-1
            rospy.loginfo("word1 %s" % (pitch))
            pitch = pitch*degrees2rad

        except Exception as e:
            print e
            # pass

        imuMsg.yaw = yaw_deg
        imuMsg.pitch = pitch
        pub.publish(imuMsg)

ser.close
#f.close
