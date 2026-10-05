import math as m

class Drive:
    def __init__(self, max=255):
        self.max= max #storing the max value of the pwm signal being sent
    
    def calculate (self, vx, vy, w, R) :
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

#testing the code
if __name__ == "__main__":

    r= Drive(max=255)
    #center value (R) in mm

    #forward motion only
    m1, m2, m3= r.calculate(vx=200, vy=0, w=0, R=450)
    print(f"Foward (Vx= 200) \n M1= {m1}, M2= {m2}, M3= {m3} ")

    #right motion only
    m1, m2, m3= r.calculate(vx=0, vy=200, w=0, R=450)
    print(f"Right (Vy= 200) \n M1= {m1}, M2= {m2}, M3= {m3} ")

    #clockwise motion only
    m1, m2, m3= r.calculate(vx=0, vy=0, w=150, R=450)
    print(f"Clockwise (w=150) \n M1= {m1}, M2= {m2}, M3= {m3} ")

    #combined motion
    m1, m2, m3= r.calculate(vx=150, vy=0, w=50, R=450)
    print(f"Combined (Vx= 150, w=50) \n M1= {m1}, M2= {m2}, M3= {m3} ")


