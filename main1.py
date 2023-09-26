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

# Instantiate all components
# UART
uart = UART(0, baudrate=9600)
soil_sensor = SoilMoisture(26)
# Temp sensor
dht_pin = Pin(15, Pin.OUT, Pin.PULL_DOWN)
sensor = DHT11(dht_pin)
# Actuators
fan = PWMMotor(3, 2, 4)
bulb = PWMMotor(19, 20, 21)
bulb2 = PWMMotor(11, 12, 13)
motor = ServoMotor()
pump = PWMMotor(7, 8, 6)
# PI regulator vars
t = 250
k = 6 / 100
Kp = 4
Ti = 30 / 4
T0 = 5
Ui = 0
sat = 0

'''
DESCRIPTION OF SERIAL DATA:
1 : Send air temperature data
2 : Send air humidity data
3 : Send Soil Moisture data
4 : Send sensor data alltogether (Temperature, Humidity, Soil Moisture)
5 : Open window, start ServoMotor
6 : Start Fan
7 : Start heating with both lamps
8 : Start water pump
9 : Exit
'''


def serial_data():
    if uart.any():
        data = uart.readline().decode().strip()
        return data


def format_data(number):
    '''Format numbers t have same amount of digits.'''
    if number // 10 == 0:
        number = "{:.2f}".format(number)
    else:
        number = math.trunc(number * 10) / 10
    return number


def pi_regulator(set_point, current_value):
    '''PI regulator'''
    global sat
    global Ui
    e = set_point - current_value
    Up = Kp * e
    if sat == 0:
        Ui = Ui + Kp * e * T0 / Ti
    U = Up + Ui
    if U > 100:
        U = 100
        sat = 1
    elif U < 0:
        U = 0
        sat = 1
    else:
        sat = 0
    return U


def cooling(start=None):
    '''Open window and start fan.'''
    # Start fan

    if start == True:
        fan.start_dc()
        # Start Servo
        motor.start()
        print('ima vent')
    elif start == False:
        print('nqma vent')
        fan.stop_dc()
        motor.stop()


def heating(current_temp=None, set_point=None, start=None):
    '''Turn on the bulbs.'''
    # Start led bulb
    max_pwm = 65535
    k_pwm = 65535 / 100  # Koefficient, pwm is recieved (0:100)%
    if start == True:
        pwm = pi_regulator(set_point, current_temp)
        print(pwm)
        print(pwm * k_pwm)
        bulb.start_dc(int(pwm * k_pwm))
        bulb2.start_dc(int(pwm * k_pwm))
    elif start == False:
        bulb2.stop_dc()
        sleep(0.1)
        bulb.stop_dc()


def watering(start=None):
    '''Start water pump.'''
    if start == True:
        pump.start_dc(20000)
    elif start == False:
        pump.stop_dc()


sensor_readings = []


def moving_average(values):
    return sum(values) / len(values)


def get_data():
    '''Send sensors data via serial port.'''
    t, h = sensor.send()
    # print(t,h)
    t = str(format_data(t))
    h = str(format_data(h))
    try:
        moisture = moving_average(sensor_readings)
    except ZeroDivisionError:
        moisture, adc = soil_sensor.get_moisture()
    moisture = str(format_data(moisture))
    # Send the DATA
    return t, h, moisture

    # return data_to_send


# Timer interval in milliseconds (1 second)
TIMER_INTERVAL_MS = 5000
# Create a Timer object
timer = Timer()

# Set up the timer to call my_function at the desired interval

data_received = None


def control(timer):
    # if data_received:
    data_received = None
    print(t, h, moisture)
    if float(t) < wanted_temp - temp_hist:
        print('heat')
        heating(current_temp=float(t), set_point=wanted_temp, start=True)
        cooling(False)
    if float(t) > wanted_temp + temp_hist:
        print('cool')
        heating(start=False)
        cooling(True)
    else:
        cooling(False)

    if float(moisture) < wanted_moist:
        watering(True)
    else:
        watering(False)


def manual(data):
    '''Manual start of actuators.'''
    print(data)
    actuator = data[1]
    pwm_percent = int(data[2])
    max_pwm = 65535
    pwm = int(max_pwm * (pwm_percent / 100))
    if actuator == 'fan':
        print(f'FAN: {pwm}')
        if pwm != 0:
            fan.start_dc(pwm)
        else:
            fan.stop_dc()
    elif actuator == 'lamps':
        if pwm != 0:
            bulb.start_dc(pwm)
            bulb2.start_dc(pwm)
        else:
            bulb.stop_dc()
            bulb2.stop_dc()
    elif actuator == 'pump':
        if pwm == 0:
            pump.stop_dc()
        else:
            pump.start_dc(17000)
    elif actuator == 'window':
        if pwm == 0:
            motor.stop()
        else:
            motor.start()


t, h, moisture = get_data()
start_time = utime.time()
timer.init(period=TIMER_INTERVAL_MS, mode=Timer.PERIODIC, callback=control)
while True:
    sleep(0.2)
    # Moving average for Moisture data.
    moisture, adc = soil_sensor.get_moisture()
    if len(sensor_readings) > 10:
        sensor_readings.pop(0)
    sensor_readings.append(moisture)

    if uart.any() and uart.any() != 1:
        data_received = uart.readline().decode()
        print(data_received)
        data_received = data_received.split(':')
        if data_received[0] == 'manual':
            manual(data_received)
        else:
            if int(data_received[0]) == 1:
                wanted_temp = float(data_received[1])
                print(wanted_temp)
            elif int(data_received[0]) == 2:
                temp_hist = float(data_received[1])
                print(temp_hist)
            elif int(data_received[0]) == 3:
                wanted_moist = float(data_received[1])
                print(wanted_moist)
            elif int(data_received[0]) == 4:
                moist_hist = float(data_received[1])
                print(moist_hist)

    # Calculate the time taken by the function call
    current_time = utime.time()
    time_taken = current_time - start_time
    if time_taken >= 3:
        try:
            start_time = current_time
            sensor_data = get_data()
            t, h, moisture = sensor_data
            data_to_send = '{0},{1},{2}'.format(t, h, moisture)
            # print(sensor_data)
            print(data_to_send)
            uart.write(data_to_send)

        except KeyError as ex:
            pass

