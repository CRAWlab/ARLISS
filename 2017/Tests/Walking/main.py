#walking_test.py
#Created:4/10/2017
# -Joseph Fuentes
# -jafuentes3594@yahoo.com
from motor_class import motor, encoder
A=motor('P10', 'P11', 0, 0)
B= encoder('P22', 'P21')
B.reset_count()
while True:

    B.trigger('CW')
    A.PHpin(1)
    A.change_speed(0.4)
    if abs(B.count)==10250:
        del B.count
        A.change_speed(0)
        B.reset_count()
        break
        
