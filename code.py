import time
import math
import board
from digitalio import DigitalInOut, Direction, Pull
from pulseio import PWMOut
from busio import SPI

def cycle(p):
    """Make an iterator returning elements from the iterable and saving a copy
    of each. When the iterable is exhausted, return elements from the saved
    copy. Repeats indefinitely.

    :param p: the iterable from which to yield elements

    """
    try:
        len(p)
    except TypeError:
        # len() is not defined for this type. Assume it is
        # a finite iterable so we must cache the elements.
        cache = []
        for i in p:
            yield i
            cache.append(i)
        p = cache
    while p:
        yield from p

def lerp(x, in_min=0, in_max=1023, out_min=0, out_max=5):
    '''linear interpolation. Same as arduino map.'''
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def guess_angle(encoder):
    '''interpolate the raw encoder value to an angle'''
    return lerp(encoder, in_max=16383, out_max=360)

def get_distance(current_angle, target):
    '''return the distance between two points on a circle.

    - first get the raw difference, mod 360 if negative or > 360
    - then if the value is > 180, the shortest path is the other way
    '''
    difference = abs(current_angle - target) % 360 
    if difference > 180:
        difference = 360 - difference
    return difference

def get_direction(curr, target, distance):
    '''get the direction to rotate the motor. Find out if we need
    pass through 0. If we do, then the direction is reversed.
    
    return True if clockwise, False if anti-clockwise.'''
    clockwise = True
    if abs(curr - target) > distance:
        # passes through clock 0
        if curr > target:
            return clockwise
        else:
            return not clockwise
    else:
        if curr > target:
            return not clockwise
        else:
            return clockwise 

class Mechaduino:
    # for one step rotations
    pin_values = ((True, False, False, True),
              (True, True, False, False),
              (False, True, True, False),
              (False, False, True, True))
    pin_values_r = ((True, False, False, True),
                (False, False, True, True),
                (False, True, True, False),
                (True, True, False, False))
    cycle_pin_values = cycle(pin_values)
    cycle_pin_values_r = cycle(pin_values_r)

    # generally useful constants
    STEPS_PER_REV = 200
    DEG_PER_STEP = 360 / STEPS_PER_REV

    def __init__(self):
        # pins setup
        self.in2 = DigitalInOut(board.D7)
        self.in2.direction = Direction.OUTPUT
        self.in4 = DigitalInOut(board.D6)
        self.in4.direction = Direction.OUTPUT
        self.in1 = DigitalInOut(board.D8)
        self.in1.direction = Direction.OUTPUT
        self.in3 = DigitalInOut(board.D5)
        self.in3.direction = Direction.OUTPUT

        self.vr12 = PWMOut(board.D9, frequency=200000, duty_cycle=65000)
        self.vr34 = PWMOut(board.D4, frequency=200000, duty_cycle=65000)
        # SPI setup
        self.spi = SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        self.cs = DigitalInOut(board.A2)
        self._setup_spi()

        # led setup
        self.led = DigitalInOut(board.D13)
        self.led.direction = Direction.OUTPUT
        self.led.value = True

        # buf for spi data
        self.buf = bytearray(2)
    
    def _setup_spi(self):
        self.spi.try_lock()
        self.spi.configure(baudrate=10000000, polarity=0, phase=1)
        self.spi.unlock()
        self.cs.direction = Direction.OUTPUT
        self.cs = True

    def read_encoder(self):
    '''Send SPI signal to read the raw value from the magnetic encoder.'''
        self.cs.value = False
        self.spi.try_lock()
        self.spi.write_readinto(b'\xFF\xFF', self.buf)
        reading = ((buf[0] << 8) | buf[0]) & 0B0011111111111111
        self.cs.value = True
        self.spi.unlock()
        return reading

    def one_step(self, forwards=True):
        if forwards:
            self.in1.value, self.in3.value, self.in2.value, self.in4.value = next(cycle_pin_values)
        else:
            self.in1.value, self.in3.value, self.in2.value, self.in4.value = next(cycle_pin_values_r)

    def micro_one_step(self, theta):
        phase_multiplier = STEPS_PER_REV / 4
        max_duty_cycle = 30000

        angle = (phase_multiplier * theta) % 360
        sin_a = math.cos(math.pi*angle/180)
        sin_b = math.sin(math.pi*angle/180)
        a_coil = int(sin_a * max_duty_cycle)
        b_coil = int(sin_b * max_duty_cycle)
        self.vr12.duty_cycle = abs(a_coil)
        self.vr34.duty_cycle = abs(b_coil)
        if a_coil >= 0:
            self.in1.value = False
            self.in2.value = True
        else:
            self.in1.value = True
            self.in2.value = False
        if b_coil >= 0:
            self.in3.value = False
            self.in4.value = True
        else:
            self.in3.value = True
            self.in4.value = False

        def goto(self, target, step_size=1):
            '''Moves the motor to a target angle via the shortest path.

            :param target: the target angle in degrees
            :param step_size: the step size in degrees
            '''
            current_angle = guess_angle(self.read_encoder())
            distance = get_distance(current_angle, target)
            clockwise = get_direction(current_angle, target, distance)
            num_steps = int(distance/step_size)
            for step in range(num_steps):
                if clockwise:
                    self.micro_one_step(step/step_size)
                else:
                    self.micro_one_step(-step/step_size)

mech = Mechaduino()
while True:
    mech.goto(5)
    time.sleep(0.01)
    mech.goto(355)
    time.sleep(0.01)

# for i in range(10000):
#     micro_one_step(i/2)
#     # one_step()
#     #time.sleep(0.001)

# x = 0
# while True:
#     goto(x % 360)
#     x += 10#math.pi / 180
#     time.sleep(1)

