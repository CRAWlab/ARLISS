
import pyb
accel = pyb.Accel()
SENSITIVITY = 3

while True:
    x = accel.x()
    y = accel.y()
    z = accel.z()
    print(z)
    pyb.delay(500)
