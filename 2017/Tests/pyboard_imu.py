import pyb
accel = pyb.Accel()
light = pyb.LED(3)
SENSITIVITY = 1

while True:
    x = accel.x()
    if abs(x) > SENSITIVITY:
        light.on()
    else:
        light.off()

    pyb.delay(1000)
