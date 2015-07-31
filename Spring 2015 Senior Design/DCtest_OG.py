#!/usr/bin/python

# this is the file that came in the examples folder... i just changed the object names of the motors

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor


import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x61)

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
rightMotor.setSpeed(150)
leftMotor.setSpeed(150)
rightMotor.run(Adafruit_MotorHAT.FORWARD);
leftMotor.run(Adafruit_MotorHAT.FORWARD);
# turn on motor
rightMotor.run(Adafruit_MotorHAT.RELEASE);
leftMotor.run(Adafruit_MotorHAT.RELEASE);


while (True):
	print "Forward! "
	rightMotor.run(Adafruit_MotorHAT.FORWARD)
	leftMotor.run(Adafruit_MotorHAT.FORWARD)
	
	print "\tSpeed up..."
	for i in range(255):
		rightMotor.setSpeed(i)
		leftMotor.setSpeed(i)
		time.sleep(0.01)

	print "\tSlow down..."
	for i in reversed(range(255)):
		rightMotor.setSpeed(i)
		leftMotor.setSpeed(i)
		time.sleep(0.01)

	print "Backward! "
	rightMotor.run(Adafruit_MotorHAT.BACKWARD)
	leftMotor.run(Adafruit_MotorHAT.FORWARD)

	print "\tSpeed up..."
	for i in range(255):
		rightMotor.setSpeed(i)
		leftMotor.setSpeed(i)
		time.sleep(0.01)

	print "\tSlow down..."
	for i in reversed(range(255)):
		rightMotor.setSpeed(i)
		leftMotor.setSpeed(i)
		time.sleep(0.01)

	print "Release"
	rightMotor.run(Adafruit_MotorHAT.RELEASE)
	leftMotor.run(Adafrruit_MotorHAT.RELEASE)
	time.sleep(1.0)