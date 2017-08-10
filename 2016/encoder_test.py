
from pyb import Pin, Timer
import time
import math


pin_a = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
pin_b = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)

enc_timer = pyb.Timer(2, prescaler=0, period=65535)
while True:
    time.sleep(0.5)
    print("\nMotorA =", enc_timer.counter())
