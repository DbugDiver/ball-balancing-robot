'''
Dimensions of robot
    Plate radius = 420 mm
    Lₚ = 200 mm (radius of the platform connection triangle)
    Lb = 148mm
    L1 = 117 mm (dis from c1 to A1)
    L2 = 93 mm
    h0 = 238 mm
'''

import math
import time
radius = 420
LP = 200
LB = 148
L1 = 117 
L2 = 93 
H =  140#140 #0#150 #L1 + L2 -10
sqrt_3 = math.sqrt(3)
class Rob_Kin:
    def __init__(self):

        self.alpha = None
        self.beta = None
        self.gamma = None
        
        # motor 1 lies on Positive axis, y = 0 in that plane.
        self.B1 = [LB, 0, 0]  #B1 = Lb, 0, 0
        self.slope1 = 0

        # motor 2 lies on line tan(120)
        self.B2 = [-LB/2, sqrt_3*LB/2, 0]     # B2 = -lb/2 , sqrt3*Lb/2, 0
        self.slope2 = -math.sqrt(3)

        #motor 3 lies on line tan(240)
        self.B3 = [-LB/2, sqrt_3*LB/2, 0]      #B3 = -Lb/2 , -sqrt3*Lb/2, 0
        self.slope3 = math.sqrt(3)

    def compute_normal_vector(self, theta, phi):
        '''
        cartesian format from sperical
        '''
        nx = math.sin(math.radians(theta)) * math.cos(math.radians(phi))
        ny = math.sin(math.radians(theta)) * math.sin(math.radians(phi))
        nz = math.cos(math.radians(theta))

        n = [nx, ny, nz]
        self.set_normal_vector(n)
        return
    
    def set_normal_vector(self, normal_vect):
        self.alpha, self.beta, self.gamma = normal_vect[0], normal_vect[1], normal_vect[2]
        return

    def sol_motor1(self):
        # B cords are given for particular motor
        Bx, By , Bz = self.B1

        #Compute A cordinates:
        
        Ax =  (self.gamma * LP)/ math.sqrt( self.gamma**2 + self.alpha**2)
        Ay = 0
        Az1 = H - (self.alpha * LP)/ math.sqrt( self.gamma**2 + self.alpha**2) #------------------
        Az2 = H + (self.alpha * LP)/ math.sqrt( self.gamma**2 + self.alpha**2)
        #print(f"A1_z neg : {Az1}")
        #print(f"A1_z pos : {Az2}")
        # using Az1 only for now
        Az = Az1

        #compute C cordinates:
        p = (LB - Ax)/Az
        q = (Ax**2 + Az**2  + L2**2 - L1**2 - LB**2)/ (2*Az)

        a = (1 + p**2)
        b = 2 * ((p*q) - LB)
        c = LB**2 + q**2 -L2**2
        '''
        coff = b**2 - (4*a*c)
        if coff>= 0: return True
        else: return False
        '''
        #taking only positive values for now -------------------------------------------------------
        cx = (-b + (math.sqrt( b**2 - (4*a*c))))/(2*a)
        cy = 0
        cz = p*cx + q

        theta1  = math.degrees(math.atan2(cz, (cx - LB)))  #-24
        #print(f"theta1 : {theta1}") 
        return theta1
    
    def sol_motor2(self):
        Bx, By , Bz = self.B2

        #solving A cordinates
        den = ( math.sqrt(4*(self.gamma**2) + (self.alpha - (sqrt_3*self.beta))**2 ) )
        Ax = (- self.gamma * LP) / den
        Ay = self.slope2 * Ax
        Az1 = H - ((LP * (self.alpha - (sqrt_3*self.beta) )) / den )
        Az2 = H + ((LP * (self.alpha - (sqrt_3*self.beta) )) / den )
        #print(f"A2_z neg : {Az1}")
        #print(f"A2_z pos : {Az2}")

        #use positve for other motors
        Az = Az2

        # computing C cordinates:
        p = (Ax - sqrt_3*Ay + 2*LB)/ -Az
        q = (L1**2 - L2**2 - Ax**2 - Ay**2 - Az**2 + LB**2)/ (-2*Az)
        a = (4 + p**2)
        b = (4* LB + (2*p*q) )
        c = (LB**2 + q**2 - L2**2)

        
        coff = b**2 - (4*a*c)
        '''
        if coff>= 0: 
            return True
        else: return False
        '''
        cx = (-b - (math.sqrt( b**2 - (4*a*c))))/(2*a)
        cy = self.slope2 * cx
        cz = cz = p*cx + q

        #compute theta
        theta2 = math.degrees(math.atan2(cz , (math.sqrt(cx**2 + cy**2) - LB) )) #- 24
        #print(f"theta2 : {theta2}")
        return theta2
    
    def sol_motor3(self):

        Bx, By , Bz = self.B3

        #solving A cordinates:
        den = ( math.sqrt(4*(self.gamma**2) + (self.alpha + (sqrt_3*self.beta))**2 ) )
        Ax = (- self.gamma * LP) / den
        Ay = self.slope3 * Ax
        Az1 = H - ((LP * (self.alpha + (sqrt_3*self.beta) )) / den )
        Az2 = H + ((LP * (self.alpha + (sqrt_3*self.beta) )) / den )
        #print(f"A3_z neg : {Az1}")
        #print(f"A3_z pos : {Az2}")

        #use positve for other motors
        Az = Az2

        # computing C cordinates:
        p = (Ax + sqrt_3*Ay + 2*LB)/ -Az
        q = (L1**2 - L2**2 - Ax**2 - Ay**2 - Az**2 + LB**2)/ (-2*Az)
        a = (4 + p**2)
        b = (4* LB + (2*p*q) )
        c = (LB**2 + q**2 - L2**2)

        '''
        coff = b**2 - (4*a*c)
        if coff>= 0: return True
        else: return False
        '''
        cx = (-b - (math.sqrt( b**2 - (4*a*c))))/(2*a)
        cy = self.slope3 * cx
        cz = p*cx + q

        #compute theta
        theta3 = math.degrees(math.atan2(cz , (math.sqrt(cx**2 + cy**2) - LB) )) #-24
        #print(f"theta3 : {theta3}")
        return theta3 

    def max_theta(self, tol=1e-3):
        theta_low, theta_high = 0.0, math.radians(20)
        def valid(theta):
            c = math.cos(theta)
            for s in (1, -1):
                a21 = LP * c
                a23 = H - LP * (s * math.sin(theta))
                try:
                    p2 = (LB - a21) / a23
                    q2 = (a21**2 + a23**2 - LB**2 + L2**2 - L1**2) / (2 * a23)
                    r2 = p2**2 + 1
                    s2 = 2 * (p2 * q2 - LB)
                    t2 = q2**2 - L2**2 + LB**2
                    disc = s2**2 - 4 * r2 * t2
                    if disc < 0: return False
                    c21 = (-s2 + math.sqrt(disc)) / (2 * r2)
                    delta = L2**2 - (c21 - LB)**2
                    if delta < 0: return False
                    c23 = math.sqrt(delta)
                    if abs(math.sqrt((a21-c21)**2 + (a23-c23)**2) - L1) > 1e-3: return False
                    if abs(math.sqrt((LB-c21)**2 + c23**2) - L2) > 1e-3: return False
                except:
                    return False
            return True
        while theta_high - theta_low > tol:
            theta_mid = (theta_low + theta_high) / 2
            if valid(theta_mid): theta_low = theta_mid
            else: theta_high = theta_mid

        maxtheta = max(0, math.degrees(round(theta_low, 4)) - 0.5)
        return maxtheta

    def run_kinematics(self, theta, phi):
        #max_tilt = self.max_theta()
        #print(f" Max tilt of plate: {max_tilt}")
        self.compute_normal_vector(theta, phi)
        theta1 = self.sol_motor1() - 18.048898555611178
        theta2 = self.sol_motor2() - 18.048898555611178
        theta3 = self.sol_motor3() - 18.048898555611178

        return theta1 , theta2, theta3


        
def main():
    kin = Rob_Kin()
    dir = 180
    global H
    theta1, theta2, theta3  = kin.run_kinematics(0, 180)
    print(f"angles: {theta1+18.048898555611178} { theta2+18.048898555611178} {theta3+18.048898555611178}")
    

    '''
    for h in range(120,200, 10):
        H = h
        for i in range(30):
            bool_val1, bool_val2, bool_val3 = kin.run_kinematics(i, dir)
            if bool_val1 and bool_val2 and bool_val3:
                print(f"valid thetas {i} with height h{H}")
    '''
if __name__ == "__main__":
    main()



        




