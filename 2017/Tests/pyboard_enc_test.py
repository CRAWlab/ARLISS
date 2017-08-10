
from pyb import Pin, Timer
import time
import math


pin_a = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
pin_b = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF1_TIM2)
pin_c = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)
pin_d = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_UP, af=pyb.Pin.AF2_TIM4)

enc_timer = pyb.Timer(2, prescaler=0, period=65535)
enc_timer_1 = pyb.Timer(4, prescaler=0, period=65535)
enc_channel = enc_timer.channel(1, pyb.Timer.ENC_AB)
enc_channel_1 = enc_timer_1.channel(2, pyb.Timer.ENC_AB)
while True:
    time.sleep(0.5)
    print("\nMotorA =", enc_timer.counter())
    print("\nMotorB =", enc_timer_1.counter())
