from machine import Pin, PWM
from time import sleep
import time


class PWMMotor:
    def __init__(self, pin1, pin2, pwm_pin):
        '''Initializator.'''
        self._pin1 = pin1
        self._pin2 = pin2
        self._pwm_pin = pwm_pin
        self._init_pins()

    def _init_pins(self):
        '''Initialize the two pins for the controller.'''
        self.IN1 = Pin(self._pin1, Pin.OUT)
        self.IN2 = Pin(self._pin2, Pin.OUT)
        self.speed = PWM(Pin(self._pwm_pin))
        self.speed.freq(1000)

    def start_dc(self):
        '''Start the DC motor.'''
        self.speed.duty_u16(17250)
        self.IN1.low()
        self.IN2.high()

    def stop_dc(self):
        '''Stop the DC motor.'''
        self.speed.duty_u16(0)



