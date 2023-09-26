import os
import math
import time
import utime
from time import sleep
from machine import Pin, I2C, UART, Timer
from dht import DHT11, InvalidChecksum
from moistre_sensor import SoilMoisture
from pwm_motor import PWMMotor
from servo import ServoMotor



# UART
uart = UART(0, baudrate=9600)
soil_sensor = SoilMoisture(26)

# Timer interval in milliseconds (1 second)
TIMER_INTERVAL_MS = 5000
# Create a Timer object
timer = Timer()
# Set up the timer to call my_function at the desired interval
data_received = None

class Greenhouse:
    # PI regulator vars
    t = 250
    k = 6 / 100
    Kp = 4
    Ti = 30 / 4
    T0 = 5
    Ui = 0
    sat = 0

    def __init__(self):
        '''Initialize components'''
        # Temp sensor
        self.dht_pin = Pin(15, Pin.OUT, Pin.PULL_DOWN)
        self.sensor = DHT11(self.dht_pin)
        # Actuators
        self.fan = PWMMotor(3, 2, 4)
        self.bulb = PWMMotor(19, 20, 21)
        self.bulb2 = PWMMotor(11, 12, 13)
        self.motor = ServoMotor()
        self.pump = PWMMotor(7, 8, 6)
        self.sensor_readings = []

    @staticmethod
    def format_data(number):
        '''Format numbers t have same amount of digits.'''
        if number // 10 == 0:
            number = "{:.2f}".format(number)
        else:
            number = math.trunc(number * 10) / 10
        return number

    def pi_regulator(self, set_point, current_value):
        '''PI regulator'''
        e = set_point - current_value
        self.Up = self.Kp * e
        if self.sat == 0:
            self.Ui = self.Ui + self.Kp * e * self.T0 / self.Ti
        U = self.Up + self.Ui
        if U > 100:
            U = 100
            self.sat = 1
        elif U < 0:
            U = 0
            self.sat = 1
        else:
            self.sat = 0
        return U

    def cooling(self, start=None):
        '''Open window and start fan.'''
        # Start fan

        if start == True:
            self.fan.start_dc()
            # Start Servo
            self.motor.start()
            print('ima vent')
        elif start == False:
            print('nqma vent')
            self.fan.stop_dc()
            self.motor.stop()

    def heating(self, current_temp=None, set_point=None, start=None):
        '''Turn on the bulbs.'''
        # Start led bulb
        max_pwm = 65535
        k_pwm = 65535 / 100  # Koefficient, pwm is recieved (0:100)%
        if start == True:
            pwm = self.pi_regulator(set_point, current_temp)
            print(pwm)
            print(pwm * k_pwm)
            self.bulb.start_dc(int(pwm * k_pwm))
            self.bulb2.start_dc(int(pwm * k_pwm))
        elif start == False:
            self.bulb2.stop_dc()
            sleep(0.1)
            self.bulb.stop_dc()

    def watering(self, start=None):
        '''Start water pump.'''
        if start == True:
            self.pump.start_dc(20000)
        elif start == False:
            self.pump.stop_dc()

    @staticmethod
    def moving_average(values):
        return sum(values) / len(values)

    def get_data(self):
        '''Send sensors data via serial port.'''
        t, h = self.sensor.send()
        # print(t,h)
        t = str(self.format_data(t))
        h = str(self.format_data(h))
        try:
            moisture = Greenhouse.moving_average(self.sensor_readings)
        except ZeroDivisionError:
            moisture, adc = soil_sensor.get_moisture()
        moisture = str(self.format_data(moisture))
        # Send the DATA
        return t, h, moisture

        # return data_to_send

    def control(self, timer):
        # if data_received:
        data_received = None
        print(self.t, self.h, self.moisture)
        if float(self.t) < self.wanted_temp - self.temp_hist:
            print('heat')
            self.heating(current_temp=float(self.t), set_point=self.wanted_temp, start=True)
            self.cooling(False)
        if float(self.t) >self.wanted_temp + self.temp_hist:
            print('cool')
            self. heating(start=False)
            self.cooling(True)
        else:
            self.cooling(False)

        if float(self.moisture) < self.wanted_moist:
            self.watering(True)
        else:
            self.watering(False)

    def manual(self, data):
        '''Manual start of actuators.'''
        print(data)
        actuator = data[1]
        pwm_percent = int(data[2])
        max_pwm = 65535
        pwm = int(max_pwm * (pwm_percent / 100))
        if actuator == 'fan':
            print(f'FAN: {pwm}')
            if pwm != 0:
                self.fan.start_dc(pwm)
            else:
                self.fan.stop_dc()
        elif actuator == 'lamps':
            if pwm != 0:
                self.bulb.start_dc(pwm)
                self.bulb2.start_dc(pwm)
            else:
                self.bulb.stop_dc()
                self.bulb2.stop_dc()
        elif actuator == 'pump':
            if pwm == 0:
                self.pump.stop_dc()
            else:
                self.pump.start_dc(17000)
        elif actuator == 'window':
            if pwm == 0:
                self.motor.stop()
            else:
                self.motor.start()

    def start_greenhouse(self):
        self.t, self.h, self.moisture = self.get_data()
        start_time = utime.time()
        timer.init(period=TIMER_INTERVAL_MS, mode=Timer.PERIODIC, callback=self.control)
        while True:
            sleep(0.2)
            # Moving average for Moisture data.
            self.moisture, adc = soil_sensor.get_moisture()
            if len(self.sensor_readings) > 10:
                self.sensor_readings.pop(0)
            self.sensor_readings.append(self.moisture)

            if uart.any() and uart.any() != 1:
                data_received = uart.readline().decode()
                print(data_received)
                data_received = data_received.split(':')
                if data_received[0] == 'manual':
                    self.manual(data_received)
                else:
                    if int(data_received[0]) == 1:
                        self.wanted_temp = float(data_received[1])
                        print(self.wanted_temp)
                    elif int(data_received[0]) == 2:
                        self.temp_hist = float(data_received[1])
                        print(self.temp_hist)
                    elif int(data_received[0]) == 3:
                        self.wanted_moist = float(data_received[1])
                        print(self.wanted_moist)
                    elif int(data_received[0]) == 4:
                        self.moist_hist = float(data_received[1])
                        print(self.moist_hist)

            # Calculate the time taken by the function call
            current_time = utime.time()
            time_taken = current_time - start_time
            if time_taken >= 3:
                try:
                    start_time = current_time
                    sensor_data = self.get_data()
                    self.t, self.h, self.moisture = sensor_data
                    data_to_send = '{0},{1},{2}'.format(self.t, self.h, self.moisture)
                    # print(sensor_data)
                    print(data_to_send)
                    uart.write(data_to_send)

                except KeyError as ex:
                    pass
if __name__ == "__main__":
    greenhouse = Greenhouse()