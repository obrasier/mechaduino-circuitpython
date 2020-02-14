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

steps_per_rev = 200
deg_per_step = 360 / steps_per_rev

led = DigitalInOut(board.D13)
led.direction = Direction.OUTPUT
led.value = True

spi = SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

spi.try_lock()
spi.configure(baudrate=10000000, polarity=0, phase=1)
spi.unlock()
CS_PIN = board.A2
cs = DigitalInOut(CS_PIN)
cs.direction = Direction.OUTPUT

def lerp(x, in_min=0, in_max=1023, out_min=0, out_max=5):
    '''linear interpolation. Same as arduino map.'''
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def read_encoder():
    '''Send SPI signal to read the raw value from the magnetic encoder.'''
    cs.value = False
    buf = bytearray(2)
    spi.try_lock()
    spi.write_readinto(b'\xFF\xFF', buf)
    reading = ((buf[0] << 8) | buf[0]) & 0B0011111111111111
    cs.value = True
    spi.unlock()
    return reading

def guess_angle(encoder):
    '''interpolate the raw encoder value to an angle'''
    return lerp(encoder, in_max=16383, out_max=360)

def one_step(forwards=True):
    if forwards:
        in1.value, in3.value, in2.value, in4.value = next(cycle_pin_values)
    else:
        in1.value, in3.value, in2.value, in4.value = next(cycle_pin_values_r)

def micro_one_step(theta):
    phase_multiplier = steps_per_rev / 4
    max_duty_cycle = 30000

    angle = (phase_multiplier * theta) % 360
    sin_a = math.cos(math.pi*angle/180)
    sin_b = math.sin(math.pi*angle/180)
    a_coil = int(sin_a * max_duty_cycle)
    b_coil = int(sin_b * max_duty_cycle)
    vr12.duty_cycle = abs(a_coil)
    vr34.duty_cycle = abs(b_coil)
    if a_coil >= 0:
        in1.value = False
        in2.value = True
    else:
        in1.value = True
        in2.value = False
    if b_coil >= 0:
        in3.value = False
        in4.value = True
    else:
        in3.value = True
        in4.value = False

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


def goto(target, step_size=1):
    '''Moves the motor to a target angle via the shortest path.

    :param target: the target angle in degrees
    :param step_size: the step size in degrees
    '''
    current_angle = guess_angle(read_encoder())
    distance = get_distance(current_angle, target)
    clockwise = get_direction(current_angle, target, distance)
    num_steps = int(distance/step_size)
    for step in range(num_steps):
        if clockwise:
            micro_one_step(step/step_size)
        else:
            micro_one_step(-step/step_size)

# A is in1/in2

in2 = DigitalInOut(board.D7)
in2.direction = Direction.OUTPUT
in4 = DigitalInOut(board.D6)
in4.direction = Direction.OUTPUT
in1 = DigitalInOut(board.D8)
in1.direction = Direction.OUTPUT
in3 = DigitalInOut(board.D5)
in3.direction = Direction.OUTPUT

vr12 = PWMOut(board.D9, frequency=200000, duty_cycle=65000)
vr34 = PWMOut(board.D4, frequency=200000, duty_cycle=65000)


for i in range(10000):
    micro_one_step(i/2)
    # one_step()
    #time.sleep(0.001)

# x = 0
# while True:
#     goto(x % 360)
#     x += 10#math.pi / 180
#     time.sleep(1)

