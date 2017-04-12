from motor_class import motor, encoder

class rover:
    motorA = motor(16, 38,  0, 0)
    motorB = motor(20, 39,  1, 1)
    motorC = motor(22, 18,  2, 2)
    motorD = motor(42, 17,  3, 3)

    encoderA = encoder(5, 6)
    encoderB = encoder(7, 8)
    encoderC = encoder(11, 10)
    encoderD = encoder(12, 13)
    def __init__(self, motorA, motorB,  motorC,  motorD, encoderA, encoderB, encoderC, encoderD):
        self.motorA= motorA
        self.motorB= motorB
        self.motorC= motorC
        self.motorD= motorD
        
        self.encoderA=encoderA
        self.encoderB=encoderB
        self.encoderC=encoderC
        self.encoderD=encoderD
    
    def calibrate(self):
    ###############################User input section#################################
        current_orientation_A=60# int(input(' facing shaft current bottom point of leg degree from positive x dir in degrees '))
        desired_orientation_A= 30#int(input('facing shaft desired location of leg in degrees'))

        current_orientation_B= int((' facing shaft current bottom point of leg degree from positive x dir in degrees '))
        desired_orientation_B= int(('facing shaft desired location of leg in degrees'))

        current_orientation_C=int( input(' facing shaft current bottom point of leg degree from positive x dir in degrees '))
        desired_orientation_C=int( input('facing shaft desired location of leg in degrees'))

        current_orientation_D= int((' facing shaft current bottom point of leg degree from positive x dir in degrees '))
        desired_orientation_D=int( input('facing shaft desired location of leg in degrees from positive x dir'))

    ################################Calculations#####################################
        pulses_per_revolution= 1800
        degrees_per_revolution=360
        pulses_A= (desired_orientation_A-current_orientation_A)*(pulses_per_revolution/degrees_per_revolution)
        pulses_B= (desired_orientation_B-current_orientation_B)*(pulses_per_revolution/degrees_per_revolution)
        pulses_C= (desired_orientation_C-current_orientation_C)*(pulses_per_revolution/degrees_per_revolution)
        pulses_D= (desired_orientation_D-current_orientation_D)*(pulses_per_revolution/degrees_per_revolution)

    ##################################Moving legs Based on pulses#######################
    #Moving Motor A
        self.encoderA.trigger('CW')
    
        while True:

            self.motorA.direction('CW')
            self.motorA.change_speed(0.2)
            if abs(self.encoderA.get_count())==pulses_A:
                self.motorA.change_speed(0)
            break

    #Moving Motor B
        self.encoderB.trigger('CW')
    
        while True:

            self.motorB.direction('CW')
            self.motorB.change_speed(0.2)
            if abs(self.encoderB.get_count())==pulses_B:
                self.motorB.change_speed(0)
            break

    #Moving Motor C
        self.encoderC.trigger('CW')

        while True:
            self. motorC.direction('CW')
            self.motorC.change_speed(0.2)
            if abs(self.encoderC.get_count())==pulses_C:
                self.motorC.change_speed(0)
            break

    #Moving Motor D
        self.encoderD.trigger('CW')
    
        while True:

            self.motorD.direction('CW')
            self. motorD.change_speed(0.2)
            if abs(self.encoderD.get_count())==pulses_D:
                self.motorD.change_speed(0)
            break
        self.encoderA.set_count()
        self.encoderB.set_count()
        self.encoderC.set_count()
        self.encoderD.set_count()
    
        
    def lay_down(self):
        pass
        
    def stand(self):
        '''''Called When rover lands as expected'''''
        self.encoderA.trigger(0)
        self.encoderB.trigger(1)
        while True:
            self.motorA.change_speed(0.2)
            self.motorA.direction('CCW')
            self.motorB.change_speed=0.2
            self.motorB.direction('CW')
        #Change Value of front leg rotation  to value until legs are perpendicular to body
        #1800 pulses should be one revolution
            front_legs_rotation= 1800
            if abs(self.encoderA.count & self.encoderB.count)  == front_legs_rotation:
                self.motorA.change_speed(0)   
                self.motorB.change_speed(0)
            break
        
        self.encoderC.trigger(0)
        self.encoderD.trigger(1)
        while True:
            self.motorC.motor.change_speed(0.2)
            self.motorC.direction('CCW')
            self.motorD.change_speed=0.2
            self.motorD.direction('CW')
            
        #Change Value of back leg rotation to value until legs are perpendicular to body
        #1800 pulses should be one revolution
            back_legs_rotation= 1800
            if abs(self.encoderC.count & self.encoderD.count)  == back_legs_rotation:
                self.motorC.change_speed(0)   
                self.motorD.change_speed(0)
            break
            

    def upside_down_stand(self,  speed, motorA, motorB, motorC, motorD):  
        '''''Called When rover lands unexpectedly'''''
        encoderA.trigger(0)
        encoderB.trigger(1)
        while True:
            motorA.change_speed(0.2)
            motorA.direction('CCW')
            motorB.change_speed=0.2
            motorB.direction('CW')
            if abs(encoderA.count & encoderB.count)  == 1800:
                motorA.change_speed(0)   
                motorB.change_speed(0)
            break
    
        encoderC.trigger(0)
        encoderD.trigger(1)
        
        while True:
            motorC.motor.change_speed(0.2)
            motorC.direction('CCW')
            motorD.change_speed=0.2
            motorD.direction('CW')
            if abs(encoderC.count & encoderD.count)  == 1800:
                motorC.change_speed(0)   
                motorD.change_speed(0)
            break   
        
    def stop(self, speed, motorA, motorB, motorC, motorD):
            motorA.stop
            motorB.stop
            motorC.stop
            motorD.stop
        
    def forward(self, speed, motorA, motorB, motorC, motorD):
        
            pass
    def backward(self, speed, motorA, motorB, motorC, motorD):
            pass
    def turn_left(self, speed, motorA, motorB, motorC, motorD):
            pass
    def turn_right(self, speed, motorA, motorB, motorC, motorD):
            pass
        

