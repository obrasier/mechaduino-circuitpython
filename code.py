import time
import math
import board

from digitalio import DigitalInOut, Direction, Pull
from pulseio import PWMOut
from busio import SPI


_MICROSTEPS = const(16)
_STEPS = const(200)
_ENCODER_SAMPLES = const(20)
_PWM_MAX = const(30000)

_MICROSTEPS_PER_REV = const(_MICROSTEPS * _STEPS // 2)
_MICROSTEPS_PER_CYCLE = const(_MICROSTEPS * 2)


def _pwm_for_microstep(ms):
    x = math.cos(4 * math.pi * ms / _MICROSTEPS_PER_CYCLE)
    return int((x + 1) * (_PWM_MAX // 2))

_PWM_TABLE = [_pwm_for_microstep(ms) for ms in range(_MICROSTEPS_PER_CYCLE)]


# Use the LED to indicate when the coils are energised.
led = DigitalInOut(board.D13)
led.direction = Direction.OUTPUT
led.value = True


# Bipolar stepper -- the two coils are A0-A1 and B0-B1.
pin_a0 = DigitalInOut(board.D8)
pin_a0.direction = Direction.OUTPUT
pin_a1 = DigitalInOut(board.D7)
pin_a1.direction = Direction.OUTPUT
pin_b0 = DigitalInOut(board.D5)
pin_b0.direction = Direction.OUTPUT
pin_b1 = DigitalInOut(board.D6)
pin_b1.direction = Direction.OUTPUT

# The h-bridge takes a PWM input, one for each of A and B.
pin_pwm_a = PWMOut(board.D9, frequency=200000, duty_cycle=0)
pin_pwm_b = PWMOut(board.D4, frequency=200000, duty_cycle=0)

# SPI bus connected to the encoder.
spi = SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
spi.try_lock()
spi.configure(baudrate=10000000, polarity=0, phase=1)
spi.unlock()
spi_cs = DigitalInOut(board.A2)
spi_cs.direction = Direction.OUTPUT


def read_encoder():
    """
    Returns averaged encoder value in the range (0,_MICROSTEPS_PER_REV).
    """
    reading = 0
    for _ in range(_ENCODER_SAMPLES):
        # Send SPI signal to read the raw value from the magnetic encoder.
        spi_cs.value = False
        buf = bytearray(2)
        spi.try_lock()
        spi.write_readinto(b'\xFF\xFF', buf)
        reading += ((buf[0] << 8) | buf[0]) & 0B0011111111111111
        spi_cs.value = True
        spi.unlock()
    reading = reading // _ENCODER_SAMPLES
    return reading * _MICROSTEPS_PER_REV // 2**14


def get_direction(start, finish):
    """
    Get the direction to rotate the motor for the shortest distance.
    """
    if finish > start:
        clockwise = finish - start < _MICROSTEPS_PER_REV // 2
    else:
        clockwise = start - finish > _MICROSTEPS_PER_REV // 2
    return 1 if clockwise else -1


def turn_absolute(target, hold=False):
    """
    Turn to the specified microstep number.
    """
    led.value = True
    current = read_encoder()
    target = target % _MICROSTEPS_PER_REV
    delta = get_direction(current, target)
    while current != target:
        current = (current + delta) % _MICROSTEPS_PER_REV
        index = current % _MICROSTEPS_PER_CYCLE
        full_index = index // (_MICROSTEPS // 2)
        a0_output = full_index & 2
        b0_output = (full_index + 1) & 2
        pin_a0.value = a0_output
        pin_a1.value = not a0_output
        pin_b0.value = not b0_output
        pin_b1.value = b0_output
        pwm = _PWM_TABLE[index]
        pin_pwm_a.duty_cycle = _PWM_MAX - pwm
        pin_pwm_b.duty_cycle = pwm

    if not hold:
        pin_pwm_a.duty_cycle = 0
        pin_pwm_b.duty_cycle = 0
        led.value = False


def turn_relative(delta, hold=False):
    current = read_encoder()
    turn_absolute(current + delta, hold=hold)


_MODE_RELATIVE = const(0)
_MODE_ABSOLUTE = const(1)

def main():
    mode = _MODE_ABSOLUTE
    zero = 0
    last_cmd = 'q'
    hold = False
    while True:
        cmd = input('> ')
        if not cmd:
            cmd = last_cmd
        if cmd == 'r':
            mode = _MODE_RELATIVE
        elif cmd == 'a':
            mode = _MODE_ABSOLUTE
        elif cmd == 'z':
            zero = read_encoder()
        elif cmd == 'q':
            break
        elif cmd == 'h':
            hold = not hold
        else:
            angle = float(cmd)
            steps = int((angle * _MICROSTEPS_PER_REV) / 360)
            if mode == _MODE_RELATIVE:
                turn_relative(steps, hold=hold)
            elif mode == _MODE_ABSOLUTE:
                turn_absolute(steps + zero, hold=hold)
        last_cmd = cmd


if __name__ == '__main__':
    main()
