import pyb, math
from pyb import Pin, Timer, ADC

TOP_RAD_FANS_PWM_PIN = Pin.board.X2
BOTTOM_RAD_TOP_FANS_PWM_PIN = Pin.board.X3
BOTTOM_RAD_BOTTOM_FANS_PWM_PIN = Pin.board.X4
CPU_IN_WATER_TEMP_ADC_PIN = Pin.board.X19

BOTOM_RAD_BOTTOM_FAN1_TACH_PIN = Pin.board.Y1
BOTOM_RAD_BOTTOM_FAN2_TACH_PIN = Pin.board.Y2
BOTOM_RAD_BOTTOM_FAN3_TACH_PIN = Pin.board.Y3
BOTOM_RAD_BOTTOM_FAN4_TACH_PIN = Pin.board.Y4

BOTTOM_RAD_TOP_FAN1_TACH_PIN = Pin.board.Y5
BOTTOM_RAD_TOP_FAN2_TACH_PIN = Pin.board.Y6
BOTTOM_RAD_TOP_FAN3_TACH_PIN = Pin.board.Y7
BOTTOM_RAD_TOP_FAN4_TACH_PIN = Pin.board.Y8

TOP_RAD_FAN1_TACH_PIN = Pin.board.X9
TOP_RAD_FAN2_TACH_PIN = Pin.board.X10
TOP_RAD_FAN3_TACH_PIN = Pin.board.X11
TOP_RAD_FAN4_TACH_PIN = Pin.board.X12

TEMPERATURE_READING_ISR_TIMER = 9
FANS_PWM_TIMER = 2
TOP_RAD_FANS_PWM_CHANNEL = 4    # PyBoard Lite only !
BOTTOM_RAD_TOP_FANS_PWM_CHANNEL = 1    # PyBoard Lite only !
BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL = 2    # PyBoard Lite only !

MINIMUM_RPM_DUTY_TIME = 10

TEMPERATURE_SENSOR_DIVIDER_RESISTANCE = 2200    # we have a 2k2 from adc pin to ground

NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND = 10
NUMBER_OF_READINGS_ARRAY_SIZE = NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND * 5


def readTemperatureInterruptServiceRoutine(timer):
    global controller
    controller._probeCpuInWaterTemperature()

class Controller:
    def __init__(self):
        self.temperatureReadingCounter = 0
        self._cpuInTemperatureDataReady = False
        self._timerFansPwm = Timer(FANS_PWM_TIMER, freq=25000)
        self._timerTemperatureReadingIsr = Timer(TEMPERATURE_READING_ISR_TIMER, freq = NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND)

        self._channelTopRadPwm = self._timerFansPwm.channel(TOP_RAD_FANS_PWM_CHANNEL, Timer.PWM)
        self._channelBottomRadTopFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_TOP_FANS_PWM_CHANNEL, Timer.PWM)
        self._channelBottomRadBottomFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL, Timer.PWM)

        self.setAllFansMinSpeed()

        self._adcCpuInWaterTemp = ADC(CPU_IN_WATER_TEMP_ADC_PIN)
        # 725 = reading for 25 deg. C
        self._cpuInWaterAdcReadings = [725 for c in range(NUMBER_OF_READINGS_ARRAY_SIZE)]
        self._timerTemperatureReadingIsr.callback(readTemperatureInterruptServiceRoutine)


    def _probeCpuInWaterTemperature(self):   # To be called only by ISR
        self._cpuInWaterAdcReadings[self.temperatureReadingCounter] = self._adcCpuInWaterTemp.read()
        self.temperatureReadingCounter += 1
        self.temperatureReadingCounter = self.temperatureReadingCounter % NUMBER_OF_READINGS_ARRAY_SIZE


    def setAllFansMinSpeed(self):
        self._channelTopRadPwm.pulse_width_percent(MINIMUM_RPM_DUTY_TIME)
        self._channelBottomRadTopFansPwm.pulse_width_percent(MINIMUM_RPM_DUTY_TIME)
        self._channelBottomRadBottomFansPwm.pulse_width_percent(MINIMUM_RPM_DUTY_TIME)


    def cpuInWaterTemperature(self):
        irqState = pyb.disable_irq()    # critical section
        averageAdcReading = sum(self._cpuInWaterAdcReadings) / (NUMBER_OF_READINGS_ARRAY_SIZE * 1.0)
        pyb.enable_irq(irqState)        # end of critical section
        sensorResistance = ((4095.0 / averageAdcReading) - 1.0) * TEMPERATURE_SENSOR_DIVIDER_RESISTANCE
        # From MatLab curve fitting, adding 0.2 for compensation with othe temp indicator
        return 0.2 + (-.0019 * sensorResistance * sensorResistance + 38.14 * sensorResistance + 43870) / (sensorResistance - 827)


def mainLoop():
    global controller

    while True:
        pyb.delay(5000)
        print("%3.1f" % controller.cpuInWaterTemperature())


controller = Controller()
mainLoop()
