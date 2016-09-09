from pyb import Pin, Timer
import motor_class
import utime
import pyboard_PID as pyPID


# Set up motorA
DIRA = 'Y9'
PWMA = 'X8'
TIMA = 14
CHANA = 1
motorA = motor_class.motor(PWMA, DIRA, TIMA, CHANA)

# Set up encoder timers
pin_a = pyb.Pin('X1', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF1_TIM2)

pin_b = pyb.Pin('X2', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF1_TIM2)

pin_c = pyb.Pin('X9', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF2_TIM4)

pin_d = pyb.Pin('X10', pyb.Pin.AF_PP, pull=pyb.Pin.PULL_NONE,
                af=pyb.Pin.AF2_TIM4)

enc_timer = pyb.Timer(2, prescaler=0, period=65535)
enc_timer_1 = pyb.Timer(4, prescaler=0, period=65535)

enc_channel = enc_timer.channel(1, pyb.Timer.ENC_AB)
enc_channel_1 = enc_timer_1.channel(2, pyb.Timer.ENC_AB)

# Start the motor
motorA.start(20,'cw')
init = enc_timer.counter()
total = 0
another = 0
correction = 0
pid = 0
# Get inital encoder reading
def encoder(timer):
    global init
    global total
    global correction
    current_encoder = enc_timer.counter() + (65535 * another)
    total = (current_encoder - init)
    init = current_encoder
    # print(total)
    # print(init)

def plus(timer):
    global another
    another += 1

tim_call = pyb.Timer(12, freq=10)
tim_call.callback(encoder)

enc_timer.callback(plus)

kp = 1
ki = 0.0000001
kd = 0.01
pid = pyPID.PID(kp, ki, kd, 100000, 568, 44)
while True:
    correction = pid.compute_output(400, total)
    conversion = (correction + 14.222) / 5.8222
    # print(conversion)
    motorA.change_speed(conversion)
    # utime.sleep(0.01)
    # print(correction)
