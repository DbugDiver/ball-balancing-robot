import moteus
import math
import asyncio
import time
class controller():
    def __init__(self):
        self.theta1  = 0
        self.theta2 = 0
        self.theta3 =  0

        self.min_deg = -30
        self.max_deg = 30
        self.max_diff = 60
        # Initialize motors
        '''
        self.m1 = moteus.Controller(id=1)
        self.m2 = moteus.Controller(id=2)
        self.m3 = moteus.Controller(id=3)
        '''
        self.transport = moteus.Fdcanusb()   # open COM3 ONCE

        self.m1 = moteus.Controller(id=1, transport=self.transport)
        self.m2 = moteus.Controller(id=2, transport=self.transport)
        self.m3 = moteus.Controller(id=3, transport=self.transport)

        self.prev_theta1 = 0
        self.prev_theta2 = 0
        self.prev_theta3 = 0


        #self.clear_defaults()
    async def setup(self):
        # Query actual motor angles at startup
        s1 = await self.m1.set_position(query=True)
        s2 = await self.m2.set_position(query=True)
        s3 = await self.m3.set_position(query=True)

        # Extract position (in radians)
        self.prev_theta1 = s1.position
        self.prev_theta2 = s2.position
        self.prev_theta3 = s3.position

        print("Initial motor angles:", 
              self.prev_theta1, 
              self.prev_theta2, 
              self.prev_theta3)

    async def clear_defaults(self):
        await self.m1.set_stop()
        await self.m2.set_stop()
        await self.m3.set_stop()
        await asyncio.sleep(0.05)

        state = await self.m1.query()
        mode = state.values.get(moteus.Register.MODE, None)
        print("Motor mode:", mode)

        await self.read_position()
        return

    async def read_position(self):
        
        motors = [self.m1, self.m2, self.m3]
        i  = 1
        for c in motors:
            state = await c.query()
            position_rev = state.values[moteus.Register.POSITION]
            setattr(self, f"prev_theta{i}", position_rev)
            
            
            i+=1
        

        return

    def print_angles(self):
        print("Program stoppping: ")
        print(f"Theta1 :{self.theta1}")
        print(f"Theta2 :{self.theta2}")
        print(f"Theta3 :{self.theta3}")

    def set_angles(self, theta1, theta2, theta3):
        #print("ANgles being set-------------------------------------------------------------")
        if abs(float(theta1 / 360) - self.theta1)>= 0.002:self.theta1  = float(theta1 / 360)
        if abs(float(theta2 / 360) - self.theta2)>= 0.002:self.theta2  = float(theta2 / 360)
        if abs(float(theta3 / 360) - self.theta3)>= 0.002:self.theta3  = float(theta3 / 360)
        #self.theta1 = float(theta1 /360)
        #self.theta2 = float(theta2 /360)
        #self.theta3 =  float(theta3 /360)
        return
    def clamp_angles(self, theta1, theta2, theta3):
        # home positoin is at 0.2 rev and can only go 0.1 more = 36 degree:
        

        # home positoin is at 0.2 rev and can only go 0.1 more = 36 degree:
        theta1 = max(min(theta1, self.max_deg), self.min_deg)
        theta2 = max(min(theta2, self.max_deg), self.min_deg)
        theta3 = max(min(theta3, self.max_deg), self.min_deg)

        # clamping absoulte difference
        
        diff = max( abs(theta1- theta2), abs(theta1- theta3), abs(theta2 -theta3))
        if diff > self.max_diff:
            print(f"max_diff limit exceeded = {diff}")
            return False
        
        self.set_angles(theta1, theta2, theta3)
        return True
    
    def stop_moteius(self):
        self.theta1, self.theta2, self.theta3 = 0 , 0, 0
        asyncio.run(self.run_moteius())

    
    async def run_controller(self, theta1, theta2, theta3):
        clamp_success = self.clamp_angles(theta1, theta2, theta3)

        if not clamp_success:
            self.print_angles()
            self.set_angles(0,0,0)
            return False
        else:
            return True

    async def test_motors(self):
        #asyncio.run(self.run_moteius())
        print(" starting motors")
        await self.run_moteius()
        #await self.read_position()
        return
    
    
    async def run_moteius(self): #20 .2.5
        while True:
            await asyncio.gather(
                    self.m1.set_position(position=self.theta1, velocity_limit= 20, accel_limit= 2.5),
                    self.m2.set_position(position=self.theta2, velocity_limit= 20, accel_limit= 2.5),
                    self.m3.set_position(position= self.theta3, velocity_limit= 20, accel_limit= 2.5),
                )
            await asyncio.sleep(0)
    '''
    async def run_moteius(self):
        STEPS = 130
        DT = 0.015

        prev1 = self.prev_theta1
        prev2 = self.prev_theta2
        prev3 = self.prev_theta3

        while True:
            t1, t2, t3 = self.theta1, self.theta2, self.theta3

            change = (
                abs(t1 - prev1) > float(2/360) and
                abs(t2 - prev2) > float(2/360) and
                abs(t3 - prev3) > float(2/360)
            )

            if change:
                # INTERPOLATE
                for i in range(1, STEPS + 1):
                    a = i / STEPS
                    a1 = prev1 + (t1 - prev1) * a
                    a2 = prev2 + (t2 - prev2) * a
                    a3 = prev3 + (t3 - prev3) * a

                    await asyncio.gather(
                        self.m1.set_position(position=a1,velocity_limit=float('nan'), accel_limit=float('nan')),
                        self.m2.set_position(position=a2,velocity_limit=float('nan'), accel_limit=float('nan')),
                        self.m3.set_position(position=a3,velocity_limit=float('nan'), accel_limit=float('nan')),
                    )
                    await asyncio.sleep(DT)

            # update prev after interpolation
                prev1, prev2, prev3 = t1, t2, t3
                

            else:
                # HOLD POSITION — CRITICAL
                await asyncio.gather(
                    self.m1.set_position(position=t1, velocity_limit= 40, accel_limit= 20),
                    self.m2.set_position(position=t2, velocity_limit= 40, accel_limit= 20),
                    self.m3.set_position(position=t3, velocity_limit= 40, accel_limit= 20),
                    
                )
                await asyncio.sleep(0)

            self.prev_theta1, self.prev_theta2, self.prev_theta3 = t1,t2,t3
    '''

    
        

async def main():
    contr = controller()
    await contr.clear_defaults()
    #await contr.read_position()
    await contr.test_motors()
if __name__ == "__main__":
   # main()
   asyncio.run(main())


'''
async def run_moteius(self):
        STEPS = 130
        DT = 0.015

        prev1 = self.prev_theta1
        prev2 = self.prev_theta2
        prev3 = self.prev_theta3

        while True:
            t1, t2, t3 = self.theta1, self.theta2, self.theta3

            change = (
                abs(t1 - prev1) > 0.02 and
                abs(t2 - prev2) > 0.02 and
                abs(t3 - prev3) > 0.02
            )

            if change:
                # INTERPOLATE
                for i in range(1, STEPS + 1):
                    a = i / STEPS
                    a1 = prev1 + (t1 - prev1) * a
                    a2 = prev2 + (t2 - prev2) * a
                    a3 = prev3 + (t3 - prev3) * a

                    await asyncio.gather(
                        self.m1.set_position(position=a1, velocity_limit=float('nan'), accel_limit=float('nan')),
                        self.m2.set_position(position=a2, velocity_limit=float('nan'), accel_limit=float('nan')),
                        self.m3.set_position(position=a3, velocity_limit=float('nan'), accel_limit=float('nan')),
                    )
                    await asyncio.sleep(DT)

            # update prev after interpolation
                prev1, prev2, prev3 = t1, t2, t3
                

            else:
                # HOLD POSITION — CRITICAL
                await asyncio.gather(
                    self.m1.set_position(position=t1, velocity_limit= 20, accel_limit= 2.5),
                    self.m2.set_position(position=t2, velocity_limit= 20, accel_limit= 2.5),
                    self.m3.set_position(position=t3, velocity_limit= 20, accel_limit= 2.5),
                    
                )
                await asyncio.sleep(0)

            self.prev_theta1, self.prev_theta2, self.prev_theta3 = t1,t2,t3
'''












            

