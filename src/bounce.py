import math
import asyncio
import time

class SinusoidalBounce:
    def __init__(self, frequency=3.0, amplitude=10.0, rate=200):
        """
        frequency: Hz (how fast it oscillates)
        amplitude: deg/sec velocity amplitude (strength)
        rate: control frequency (Hz)
        """
        self.frequency = frequency
        self.amplitude = amplitude     # controls bounce / throw power
        self.dt = 1.0 / rate
        self.phase = 0.0
        self.last_time = time.time()

    def step(self):
        """Compute sinusoidal velocity and integrated angle."""
        t = time.time()
        self.phase += 2 * math.pi * self.frequency * (t - self.last_time)
        self.last_time = t

        # keep phase in reasonable range
        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi

        # sinusoidal velocity
        vel = self.amplitude * math.sin(self.phase)

        # return the velocity (deg/sec)
        return vel
#-------------------------------------------------------
async def start_bounce(control, base1, base2, base3,
                            freq=3.0, amp=10.0):
    """
    base1, base2, base3 = home angles of your motors
    """
    bounce = SinusoidalBounce(frequency=freq, amplitude=amp)

    theta1 = base1
    theta2 = base2
    theta3 = base3

    dt = bounce.dt

    while True:
        # get sinusoidal velocity
        vel = bounce.step()   # deg/sec

        # integrate each motor angle (synchronous movement)
        theta1 += vel * dt
        theta2 += vel * dt
        theta3 += vel * dt

        await control.run_controller(theta1, theta2, theta3)
        await asyncio.sleep(dt)