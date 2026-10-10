import math as m 
import cv2
from ultralytics import YOLO

class Drive:
    def __init__(self, max=255):
        self.max= max #storing the max value of the pwm signal being sent
    
    def calculate (self, vx, vy, w, R=150) :
        #vx is for forward/backward speed
        #vy is for right/left speed
        #w is for rotation speed
        #R is for distance from center

        #kinematics equations based on angles 0, 120, 240 for the wheels wrt the forward (0 along Y axis)
        m1= -(vx) +(R* w)
        m2= (0.5 * vx) - ((m.sqrt(3)/2.0) * vy) + (R*w)
        m3= (0.5 * vx) + ((m.sqrt(3)/2.0) * vy) + (R*w)

        #maximum magnitude check to make sure it doesnt exceed the selfmax variable we have assigned
        #as if they get exceeded, the motor might slow down for no reason (cont.)
        max_val= max(abs(m1), abs(m2), abs(m3))

        #(cont.) here the wraparound is done, incase the motor speed through the pwm signal is sent higher than 255
        if max_val > self.max:
            m1= (m1/max_val)* self.max
            m2= (m2/max_val)* self.max
            m3= (m3/max_val)* self.max
        
        #returning back in int as pwm signals cant accept decimals
        return int(m1), int (m2), int (m3) 

class Aligner:
    def __init__(self, fh=1600, fw=1200, deadbp=15, kp=0.15):
        self.fc= fw/2.0
        self.d=deadbp
        self.kp=kp
        self.drive=Drive(255) #we'll change this to 70% when the electronics give us the dc specifications
    
    def pd(self,box):
        if box is None:
            return Drive.calculate(0,0,0), False, 0

        x1