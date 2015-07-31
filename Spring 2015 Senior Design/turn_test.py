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
myMotor = mh.getMotor(1)
myMotor2 = mh.getMotor(2)

# function for deceleration
def decelMotors(speed):
	for i in reversed(range(speed)):
		myMotor.setSpeed(i)
		myMotor2.setSpeed(i)
		time.sleep(0.001)


def accelMotors(speed):
	for i in range(speed):
		myMotor.setSpeed(i)
		myMotor2.setSpeed(i)
		time.sleep(0.001)


myMotor.run(Adafruit_MotorHAT.FORWARD);
myMotor2.run(Adafruit_MotorHAT.FORWARD);

# set the speed to start, from 0 (off) to 255 (max speed)
myMotor.setSpeed(0)
myMotor2.setSpeed(0)



# Set increments for turn timing during test
time_interval = input('Timing increments: ')

# Set max turning time
max_time = input('Set max turning time in seconds (recommended 1.2): ')

TurnResults = open('TurnResults.txt','w')


# Run tests at each turning increment in the max range
for i in range(int(max_time/time_interval)):
	
	print "Drive straight first"
	accelMotors(210)
	time.sleep(1)
	
	turn_time = str((i+1)*time_interval);
	print "Turning for %s seconds." % turn_time
	myMotor.setSpeed(250)
	myMotor2.setSpeed(170)

	time.sleep(float(turn_time))
	
	## Write turn_time and yaw (delta?) to a text file
	TurnResults.write(turn_time)
	TurnResults.write('\t')
	TurnResults.write('imu angle')
	TurnResults.write('\n')



	decelMotors(210)	
	
	print "Hit return to continue, CTRL-C to abort."
	raw_input()
	

print "Release"
myMotor.run(Adafruit_MotorHAT.RELEASE)
myMotor2.run(Adafruit_MotorHAT.RELEASE)

time.sleep(1.0)

TurnResults.close()
