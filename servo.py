import time
from time import sleep
from machine import Pin, PWM


class ServoMotor:
    def __init__(self):
        '''Initializator.'''
        self.pwm = PWM(Pin(16))
        self.pwm.freq(50)

    def start(self):
        '''Open window.'''
        # self.pwm.duty_u16(9000)

        for position in range(1000, 9000, 50):
            self.pwm.duty_u16(position)
            sleep(0.01)
            '''
        for position in range(9000,1000,-50):
            self.pwm.duty_u16(position)
            sleep(0.01)
            '''

    def stop(self):
        self.pwm.duty_u16(1000)


if __name__ == '__main__':
    pass
    # servo = ServoMotor()
    # servo.stop()
#     while True:
#         servo.start()
