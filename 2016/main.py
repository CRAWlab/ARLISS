# main.py -- put your code here!
import pyb
import stm

# This script sets up a timer to do quadrature decoding
#
# It was tested using a switch similar to https://www.sparkfun.com/products/9117
# with some debounce wired up like this: https://hifiduino.files.wordpress.com/2010/10/analogdeb.jpg
# Note: the debounce is only really required for mechanical switches. 
#
# I also tested this with one of these: http://www.lynxmotion.com/p-448-quadrature-motor-encoder-wcable.aspx

pin_a = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF1_TIM2)
pin_b = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF1_TIM2)

pin_c = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF2_TIM4)
pin_d = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE, af=pyb.Pin.AF2_TIM4)

# The prescaler is ignored. When incrementing, the counter will count up-to
# and including the period value, and then reset to 0.
enc_timer = pyb.Timer(2, prescaler=0, period=65535)
enc_timer_1 = pyb.Timer(4, prescaler=0, period=65535)
# ENC_AB will increment/decrement on the rising edge of either the A channel or the B
# channel.
enc_channel = enc_timer.channel(1, pyb.Timer.ENC_AB)
enc_channel_1 = enc_timer_1.channel(2, pyb.Timer.ENC_AB)

while True:

    print("\nCounter 1 =", enc_timer.counter());
    pyb.delay(100)
    print("\nCounter 2 =", enc_timer_1.counter());
    pyb.delay(100)