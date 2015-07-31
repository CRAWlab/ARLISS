#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

################################# DC motor test!
rightMotor = mh.getMotor(1)
leftMotor = mh.getMotor(2)
jumpMotor = mh.getMotor(3)



# set the speed to start, from 0 (off) to 255 (max speed)
rightMotor.setSpeed(60)
leftMotor.setSpeed(60)
jumpMotor.setSpeed(0)

rightMotor.run(Adafruit_MotorHAT.FORWARD);
leftMotor.run(Adafruit_MotorHAT.FORWARD);
jumpMotor.run(Adafruit_MotorHAT.FORWARD);


# turn on motor
rightMotor.run(Adafruit_MotorHAT.RELEASE);
leftMotor.run(Adafruit_MotorHAT.RELEASE);
jumpMotor.run(Adafruit_MotorHAT.RELEASE);



#######################################################

import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

# while True:
#   if imu.IMURead():
#     # x, y, z = imu.getFusionData()
#     # print("%f %f %f" % (x,y,z))
#     data = imu.getIMUData()
#     fusionPose = data["fusionPose"]
#     print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), 
#         math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
#     time.sleep(poll_interval*1.0/1000.0)

#######################################################





while (True):
	print "Forward! "
	rightMotor.run(Adafruit_MotorHAT.FORWARD)
	leftMotor.run(Adafruit_MotorHAT.FORWARD)

#######################################################
	if imu.IMURead():
		data = imu.getIMUData()
		fusionPose = data["fusionPose"]
		print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
		time.sleep(poll_interval*1.0/1000.0)
		
		
		data = imu.getIMUData()
		compass = data["compass"]
		print("r_comp: %f p_comp: %f y_comp: %f" % (math.degrees(compass[0]), math.degrees(compass[1]), math.degrees(compass[2])))
		time.sleep(poll_interval*1.0/1000.0)


		data_list = data.viewkeys()
		list(data_list)

#######################################################


#############lists all of the data sets within the dict.		
#		for f in sorted(set(data_list)):
#			print f

		
	


	print "\tgo this max speed..."

	rightMotor.setSpeed(250)
	leftMotor.setSpeed(250)
	time.sleep(1)
		
	print "\tSlow down right wheel for right turn..."
	leftMotor.setSpeed(255)
	rightMotor.setSpeed(170)		
	time.sleep(.75)               #Change this value to vary turn
	

	
	if imu.IMURead():
		data = imu.getIMUData()
		fusionPose = data["fusionPose"]
		print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
		time.sleep(poll_interval*1.0/1000.0)
		
	
	

	print "\tSlow down..."
	for i in reversed(range(255)):
		rightMotor.setSpeed(i)
		leftMotor.setSpeed(i)
		time.sleep(.001)
		

	print "\tJUMP!"
	jumpMotor.setSpeed(250)
	time.sleep(1)
	
		
		
		
		
		
		
	time.sleep(1)
		

# """	
# ###########################
# #Quick Reference:
# 
# Turn Time : Heading Change
# 
# 1.0 : 90												these i let speed up longer...(.8s)
# .75 :  imu -> 83 , 66 , 80 , 80 , 75 , 69 , 78 , 83 , 84 , 75, 90 , 84 , 85, 87 , 92 , 91, 103, 86 , 98
# .5 : 45
# 
# 
# ##########################
#
		

#	print "Backward! "
#	rightMotor.run(Adafruit_MotorHAT.BACKWARD)
#	leftMotor.run(Adafruit_MotorHAT.BACKWARD)


#	print "\tSpeed up..."
#	for i in range(255):
#		rightMotor.setSpeed(i)
#		leftMotor.setSpeed(i)

#		time.sleep(0.01)

#	print "\tSlow down..."
#	for i in reversed(range(255)):
#		rightMotor.setSpeed(i)
#		leftMotor.setSpeed(i)

#		time.sleep(0.01)

	print "Release"
	rightMotor.run(Adafruit_MotorHAT.RELEASE)
	leftMotor.run(Adafruit_MotorHAT.RELEASE)

	time.sleep(.01)
