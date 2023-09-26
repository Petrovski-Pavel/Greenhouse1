from machine import ADC, Pin
import utime


class SoilMoisture:
    '''Class for soil moisture sensor.'''
    # Calibraton values
    min_moisture = 0
    max_moisture = 65535

    def __init__(self, sensor_pin):
        '''Initializator.'''
        self._pin = sensor_pin
        self._soil = ADC(Pin(self._pin))
        # self._get_moisture()

    def get_moisture(self):
        '''Return humidity of the soil.'''
        moisture = (self.max_moisture - self._soil.read_u16()) * 100 / (self.max_moisture - self.min_moisture)
        return moisture, str(self._soil.read_u16())  # Real moisture and result from ADC

