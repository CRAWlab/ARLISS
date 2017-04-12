#walking_test.py
#Created:4/10/2017
# -Joseph Fuentes
# -jafuentes3594@yahoo.com
from machine import Pin,  PWM
import time 
from motor_class import motor,  encoder,  stand

#Pinsetup
motorA = motor(16, 38,  0, 0)
motorB = motor(20, 39,  1, 1)
motorC = motor(22, 18,  2, 2)
motorD = motor(42, 17,  3, 3)

encoderA = encoder(5, 6)
encoderB = encoder(7, 8)
encoderC = encoder(11, 10)
encoderD = encoder(12, 13)
stand(motorA, motorB, motorC, motorD)


