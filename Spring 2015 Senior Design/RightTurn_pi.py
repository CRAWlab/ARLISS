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


# set the speed to start, from 0 (off) to 255 (max speed)
rightMotor.setSpeed(60)
leftMotor.setSpeed(60)

rightMotor.run(Adafruit_MotorHAT.FORWARD);
leftMotor.run(Adafruit_MotorHAT.FORWARD);

# turn on motor
rightMotor.run(Adafruit_MotorHAT.RELEASE);
leftMotor.run(Adafruit_MotorHAT.RELEASE);


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


##IMU PRINT DATA FUNCTION
def imu_print():	
	if imu.IMURead():
		data = imu.getIMUData()
		fusionPose = data["fusionPose"]
		print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
		time.sleep(poll_interval*1.0/1000.0)


#######################################################
#				   Turn Functions
#######################################################
# function for deceleration
def decelMotors(speed):
	for i in reversed(range(speed)):
		rightMotor.setSpeed(i)
		leftMotor.setSpeed(i)
		time.sleep(0.001)


def accelMotors(speed):
	for i in range(speed):
		rightMotor.setSpeed(i)
		leftMotor.setSpeed(i)
		time.sleep(0.001)
		
def driveStraight(turn_time):
	rightMotor.setSpeed(250)
	leftMotor.setSpeed(250)
	time.sleep(float(turn_time))
		
		
def left_Turn(turn_time):
	rightMotor.setSpeed(250)
	leftMotor.setSpeed(170)
	time.sleep(float(turn_time))
	
def right_Turn(turn_time):
	rightMotor.setSpeed(170)
	leftMotor.setSpeed(250)
	time.sleep(float(turn_time))

#######################################################
#			Driving 
#######################################################

while (True):
	print "Forward! "
	rightMotor.run(Adafruit_MotorHAT.FORWARD)
	leftMotor.run(Adafruit_MotorHAT.FORWARD)

#######################################################
#			Print IMU Data Before Turn
#######################################################
	imu_print()
	
# 		data = imu.getIMUData()
# 		compass = data["compass"]
# 		print("r_comp: %f p_comp: %f y_comp: %f" % (math.degrees(compass[0]), math.degrees(compass[1]), math.degrees(compass[2])))
# 		time.sleep(poll_interval*1.0/1000.0)

	print "\tgo this max speed..."
	driveStraight(1)
	
	print "\tSlow down right wheel for right turn..."
	right_Turn(.75)       #Change this value to vary turn
	
	
#######################################################
#			Print IMU Data After Turn
#######################################################	
	imu_print()

	decelMotors(250)
		
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
		


	print "Release"
	rightMotor.run(Adafruit_MotorHAT.RELEASE)
	leftMotor.run(Adafruit_MotorHAT.RELEASE)

	time.sleep(.01)
