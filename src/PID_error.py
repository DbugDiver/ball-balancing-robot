'''
1- This file would get point and goal, compute pixel error.
2 - Convert to pid error for x, y
3 - Apply smoothening filter
4 - compute angle to tilt plate
5 - compute direction in which to tilt
'''


import math
import time
import os
'''
To react faster:
    increase KP (0.015 → 0.02 → 0.03)
    increase BETA (0.01 → 0.02 → 0.03)
To reduce shaking:
    increase ALPHA (0.5 → 0.6 → 0.7)
    increase KD slightly (0.001 → 0.0015)
'''
# ====== GLOBAL CONTROL PARAMETERS ======
'''
KP        = 0.01
KD        = 0.001
KI = 0
ALPHA     = 0.5
BETA      = 0.6
'''
'''

ALPHA = 0.4
BETA = 0.8
KP    = 0.01
KD    = 0.001
'''
KP = 0.005 * 3.0 #1.75 #2 #2.75  #0.0063 #0.0063   0.0046  #KP responds to the POSITION ERROR.
KD = 0.00005*2*2*2* 9.0  #2 #0.00005          #KD responds to the SPEED of the error (rate of change).
ALPHA = 0.3                   #Alpha controls how much of the NEW signal you use vs. the OLD signal.
BETA = 0.3
VEL_GAIN = 0.04
KI = 0.005
MUL = 1
MAG = 160

BOUNCE_COFF = 0.6
BOUNCE_INC = 0.15


#PID = PIDcontroller(kp, ki, kd, alpha, beta, max_theta= 15, conversion="tanh")
class PID_C:
    def __init__(self):
        # Gains
        self.kp, self.ki, self.kd = KP, KI, KD
        self.mul = MUL
        self.alpha = ALPHA 
        self.beta = BETA
        self.bounce_phase = 0.0
        self.mag = MAG
        self.bounce_coff = BOUNCE_COFF
        self.bounce_inc = BOUNCE_INC
        self.last_impulse = time.time()

        # Previous values for derivative
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_time = 0.001   # just initialize to small number to avoid dividing by 0

        # integral error
        self.integ_x = 0
        self.integ_y = 0
        # 
        self.prev_smooth_x  = 0
        self.prev_smooth_y = 0


        self.config_path = "pid_config.txt"
        self.last_mtime = os.path.getmtime(self.config_path)
        self.set_PD()
        self.spiral = self.generate_spiral()
        self.spiral_index = 0
    def increment(self):
        self.spiral_index +=1
    def print_spiral(self):
        print(self.spiral)
    def get_spiral(self):
        return self.spiral[min(self.spiral_index, len(self.spiral)-1)]
    
    def get_error(self, point, goal):
        #Note we may need to change this based on how camera is placed so x,y cordin
        ex =  goal[0] - point[0]
        ey =  point[1] - goal[1]
        return [ex, ey]

    def load_config(self):
        vals = {}
        with open("pid_config.txt") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    continue
                if "=" in line:
                    k, expr = line.strip().split("=")
                    k = k.strip()
                    expr = expr.strip()
                    try:
                        vals[k] = eval(expr)   # evaluate expression
                    except:
                        print(f"Error evaluating: {k} = {expr}")
        return vals

    def set_PD(self):
        current_mtime = os.path.getmtime(self.config_path)
        if current_mtime != self.last_mtime:
            self.last_mtime = current_mtime
            
            config = self.load_config()

            self.kp = config.get("KP", KP)
            self.kd = config.get("KD", KD)
            self.alpha = config.get("ALPHA", ALPHA)
            self.beta = config.get("BETA", BETA)
            self.ki = config.get("KI", KI)
            self.mul =  config.get("MUL", MUL)
            self.mag =  config.get("MAG", MAG)
            self.bounce_coff = config.get("B_COFF", BOUNCE_COFF)
            self.bounce_inc = config.get("B_INC", BOUNCE_INC)

            print("\n=== CONFIG UPDATED (Ctrl+S detected) ===")
            print(f"KP       = {self.kp}")
            print(f"KD       = {self.kd}")
            print(f"KI      = {self.ki}")
            print(f"ALPHA    = {self.alpha}")
            print(f"BETA     = {self.beta}")
            print(f"Mult = {self.mul}")
            print(f"MAG_ERROR = {self.mag}")
            print(f"Bounce coff = {self.bounce_coff}")
            print(f"Bounce INCREMENT = {self.bounce_inc}")
            
            print("========================================\n")
    
        
    
    def pid_loop(self,point, goal):
        # pixel error
        
        self.set_PD()
        ex, ey = self.get_error(point,goal)
        error_mag = (math.sqrt(ex*ex + ey*ey))
        if error_mag <= 25:
            ex, ey, error_mag = 0, 0, 0
        
        #find change in time
        curr_time = time.perf_counter()
        dt = curr_time - self.prev_time

        # safe division
        if dt<=0: dt = 1e-6
        dex = (ex - self.prev_ex) / dt
        dey = (ey - self.prev_ey) / dt
        
        
        #pid loops
        
        pid_x = self.kp*ex + self.kd*dex 
        pid_y = self.kp*ey + self.kd*dey 

        #smoothen the pid_error using exp smootheing but if dont work start looking into kalman filter
        # smoohteing formula : a* pid_x + (1-a)* prev_smooht_x
        smooth_x = self.alpha* pid_x + (1-self.alpha)* self.prev_smooth_x
        smooth_y = self.alpha* pid_y + (1-self.alpha)* self.prev_smooth_y

        # use directly these smmothx, smooth y as vector
        vx = smooth_x
        vy = smooth_y

        # hypotenues
        #r = math.hypot(vx, vy)   # same as sqrt(x**2 + y**2)
        r = math.sqrt(vx**2 + vy**2)
        # theta is smoothened by tan
        # -----------------------------------------------------------------------------change 15 to max_tilt
        #----------------------------------may need chnages here
        if error_mag >= self.mag:
            theta = max(0.0, 10 * math.tanh(self.beta * self.mul * r))
        else:
            theta = max(0.0, 10* math.tanh(self.beta * r))
            
            
                
       
        theta = max(0, theta)

        #theta = min(MAX_TILT, 0.05 * r + 0.50 * speed_term)

        #print(f" tilt angle {theta}")
        #phi = math.degrees(math.atan2(vy, vx)) # direction in plane (-π..π)
        phi = math.degrees(math.atan2(vy, vx)) + 180
        phi = (phi + 360) % 360


        self.prev_time = curr_time
        self.prev_ex , self.prev_ey = ex, ey
        self.prev_smooth_x, self.prev_smooth_y = smooth_x, smooth_y

        return  theta, phi

    def generate_spiral(self, revolutions=4, points_per_rev=200,
                    r_start=50, r_end=100):
        """
        Generates a spiral where radius smoothly grows from r_start to r_end.
        Guaranteed to sweep all 4 quadrants each revolution.
        Returns list of (x, y)
        """
        spiral = []
        total_points = revolutions * points_per_rev

        for i in range(total_points):
            # angle increases: full revolution = 2π
            t = (2 * math.pi) * (i / points_per_rev)

            # linear radius interpolation 50 → 100
            r = r_start + (r_end - r_start) * (i / total_points)

            # spiral coordinate
            x = r * math.cos(t)
            y = r * math.sin(t)

            spiral.append((x, y))

        return spiral


    '''
    def generate_spiral(self, steps=400, start_radius=50, growth=0.5, dt=0.1):
        """
        Generate a spiral of XY coordinates.
    
        start_radius : starting distance from center (e.g., 50 px)
        growth       : how fast radius expands per step
        dt           : angular speed factor
    
        Returns list of (x, y)
        """
        spiral = []

        for i in range(steps):
            t = i * dt                  # spiral angle
            r = start_radius + growth*t # spiral radius

            x = r * math.cos(t)
            y = r * math.sin(t)

            spiral.append((x, y))

        return spiral
    '''

def main():
    pid = PID_C()
    pid.print_spiral()

if __name__ == "__main__":
    main()