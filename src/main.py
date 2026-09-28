#################################################################################################################################
### This file implements color specific sphere tracking. It can potentially be used for tracking recovery by object detection ###
#################################################################################################################################

# Requires refactoring. This is just a "proof of concept" implementation
import asyncio
import cv2
import numpy as np
import threading
import time
from queue import Queue
#from server import Server
import math
from PID_error import PID_C
from PID import PIDcontroller
from tracker import Tracker
from rob_kin import Rob_Kin
from control import controller

from bounce import SinusoidalBounce
########################################
kp = 0.0063 #0.0063   0.0046
ki = 0.00005 #0.00005
kd = 0.006025 #0.00595    0.00595
alpha = 0.65
beta = 0.3
#prev_time = 0
######################################
# ==== THROW PARAMETERS ====
#THROW_DELTA    = 20    # degrees added for upward throw
#RECOVER_DELTA  = 4.0    # degrees subtracted for smooth recovery
#UP_TIME        = 0.07   # fast throw timing
#DOWN_TIME      = 0.12   # slower recovery timing
#NORMAL_SETTLE = 5


async def start_bounce(control, base1=0, base2=0, base3=0,
                       freq=10.0, amp_deg= 10):
    """
    base1, base2, base3 = home positions (in degrees or rev as you prefer)
    freq   = bounce frequency (Hz)
    amp_deg = bounce amplitude (± degrees)
    """

    dt = 0.008   # 200 Hz control loop
    t0 = time.time()

    while True:
        t = time.time() - t0

        # θ(t) = base + amp * sin(2π f t)
        #offset = amp_deg * math.sin(2 * math.pi * freq * t)
        #offset = (2 * amp_deg / math.pi) * math.asin(math.sin(2 * math.pi * freq * t))
        offset = amp_deg * (1 - math.cos(2 * math.pi * freq * t)) / 2


        theta1 = base1 + offset
        theta2 = base2 + offset
        theta3 = base3 + offset

        print(f"θ1={theta1:.2f}° θ2={theta2:.2f}° θ3={theta3:.2f}°")
        cond = await control.run_controller(theta1, theta2, theta3)
        if not cond : return False

        await asyncio.sleep(0)



async def tracker_pid_loop(control, tracker, pid, robot_kin):
    #time.sleep(5)
    cond = True
    prev_time = time.perf_counter()
    UP_TIME = 0.055    #0.053
    UP_DELTA = 1.75   #2
    DOWN_TIME = 0.053   #0.55
    DOWN_DELTA = 1.45   #1.5
    
    while cond:
        current_time  = time.perf_counter()
        point, goal = tracker.get_pixels()
        
        ##################3333
        
        #print(goal)
        #print(f"point: {point}, goal {goal}")
        ex, ey = tracker.get_error(point, goal)
        #print(f"Error in x {ex}, Error in y{ey}")
        error_strength = math.sqrt(ex**2 + ey**2 )
        
        if error_strength <= 250 and cond:
            #goal = pid.get_spiral()
            #pid.increment()
            theta , dir = pid.pid_loop(point,goal)
            #print(f"1 - > tilted angle = {theta}, direction: {dir}")
            #print(f"spiral goal {goal}")
            theta1, theta2, theta3 = robot_kin.run_kinematics(theta, phi = dir)
            #theta1, theta2, theta3 = robot_kin.run_kinematics(0, 180)

            current_time  = time.perf_counter()
            
            if current_time - prev_time >=3:
                print(f"point: {point}, goal {goal}")
                print(f"Error in x {ex}, Error in y{ey}")
                print(f"error strength : {error_strength}")
                print(" ")
                print(f"1 - > tilted angle = {theta}, direction: {dir}")
                print(" ")
                print(f"theta1 : {theta1}") 
                print(f"theta2 : {theta2}")
                print(f"theta3 : {theta3}")
                print("*-----------------------------------------------------------*")
                prev_time = current_time
            
            '''

             # Strong upward throw (main impulse)
            await control.run_controller(theta1 + UP_DELTA,
                              theta2 + UP_DELTA,
                              theta3 + UP_DELTA)
            await asyncio.sleep(UP_TIME)


            await control.run_controller(theta1 - DOWN_DELTA,
                              theta2 - DOWN_DELTA,
                              theta3 - DOWN_DELTA)
            await asyncio.sleep(DOWN_TIME)

            # Return to flat / balancing
            await control.run_controller(theta1, theta2, theta3)
            await asyncio.sleep(UP_TIME)
            
            cond = await control.run_controller(theta1, theta2, theta3)
            await asyncio.sleep(0)
            '''
            cond = await control.run_controller(theta1, theta2, theta3)
            await asyncio.sleep(0)

        else:
            #cond = await control.run_controller(theta1, theta2, theta3)
            cond = await control.run_controller(0, 0, 0)


        await asyncio.sleep(0.0001)
        
        




async def run_motors(control):
    await control.test_motors()

async def main():
    
    # Intiialize tracker first
    print("Initializing tracker...")
    pid = PID_C()
    tracker = Tracker(pointColor="r", goalColor="y")  # track green point and red goal
    robot_kin = Rob_Kin()
    control = controller()
    #bounce = SinusoidalBounce()
    #await start_bounce(control)

    
    
    await control.clear_defaults()
    
    #await tracker_pid_loop(control, tracker, pid, robot_kin)

    await asyncio.gather(
        run_motors(control),
        #await asyncio.sleep(5),
        tracker_pid_loop(control, tracker, pid, robot_kin)
        
    )
    
    
    
    

if __name__ == "__main__":
   # main()
   asyncio.run(main())

'''
theta , dir = pid.pid_loop(point,goal)
        theta1, theta2, theta3 = robot_kin.run_kinematics(theta, phi = dir)
        print(f"point: {point}, goal {goal}")
        print(f"Error in x {ex}, Error in y{ey}")
        print(f"error strength: {error_strength}")
        print(" ")
        print(f"1 - > tilted angle = {theta}, direction: {dir}")
        print(" ")
        print(f"theta1 : {theta1}") 
        print(f"theta2 : {theta2}")
        print(f"theta3 : {theta3}")
        print("*-----------------------------------------------------------*")
        #prev_time = current_time
        time.sleep(5)

'''
'''
            # Strong upward throw (main impulse)
            #await control.run_controller(theta1 + UP_DELTA,
                              theta2 + UP_DELTA,
                              theta3 + UP_DELTA)
            #await asyncio.sleep(UP_TIME)


            #await control.run_controller(theta1 - DOWN_DELTA,
                              theta2 - DOWN_DELTA,
                              theta3 - DOWN_DELTA)
            #await asyncio.sleep(DOWN_TIME)

            # Return to flat / balancing
            #await control.run_controller(theta1, theta2, theta3)
            #await asyncio.sleep(UP_TIME)
'''