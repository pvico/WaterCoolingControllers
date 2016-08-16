import pyb, math
from pyb import Pin, Timer, ADC

TOP_RAD_FANS_PWM_PIN = Pin.board.X2
BOTTOM_RAD_TOP_FANS_PWM_PIN = Pin.board.X3
BOTTOM_RAD_BOTTOM_FANS_PWM_PIN = Pin.board.X4
CPU_IN_WATER_TEMP_ADC_PIN = Pin.board.X19

BOTOM_RAD_BOTTOM_FAN1_TACH_PIN = Pin.board.Y1   # PC6 for
BOTOM_RAD_BOTTOM_FAN2_TACH_PIN = Pin.board.Y2   # PC7
BOTOM_RAD_BOTTOM_FAN3_TACH_PIN = Pin.board.Y3   # PB10 for PyBoard Lite
BOTOM_RAD_BOTTOM_FAN4_TACH_PIN = Pin.board.Y4   # PB9

BOTTOM_RAD_TOP_FAN1_TACH_PIN = Pin.board.Y5     # PB12
BOTTOM_RAD_TOP_FAN2_TACH_PIN = Pin.board.Y6     # PB13
BOTTOM_RAD_TOP_FAN3_TACH_PIN = Pin.board.Y7     # PB14
BOTTOM_RAD_TOP_FAN4_TACH_PIN = Pin.board.Y8     # PB15

TOP_RAD_FAN1_TACH_PIN = Pin.board.X9            #PB6
TOP_RAD_FAN2_TACH_PIN = Pin.board.X10           #PB7
TOP_RAD_FAN3_TACH_PIN = Pin.board.X11           #PC4
TOP_RAD_FAN4_TACH_PIN = Pin.board.X12           #PC5

TEMPERATURE_READING_ISR_TIMER = 9
FANS_PWM_TIMER = 2
TOP_RAD_FANS_PWM_CHANNEL = 4    # PyBoard Lite only !
BOTTOM_RAD_TOP_FANS_PWM_CHANNEL = 1    # PyBoard Lite only !
BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL = 2    # PyBoard Lite only !

MINIMUM_RPM_DUTY_TIME = 10

TEMPERATURE_SENSOR_DIVIDER_RESISTANCE = 2200    # we have a 2k2 from adc pin to ground


SECONDS_BETWEEN_DISPLAY_UPDATE = 5
NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND = 10
NUMBER_OF_READINGS_ARRAY_SIZE = NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND * SECONDS_BETWEEN_DISPLAY_UPDATE


def readTemperatureISR(timer):
    global controller

    controller._probeCpuInWaterTemperature()

def fortyNineDaysMillis():
    now = pyb.millis() # pyb.millis() can be negative after 24 days wrap around (32 bit integer, 2^32 = 49 days)
    return now if now >= 0 else 0x80000000 - now


class Controller:
    def __init__(self):
        self._topRadFansTachPins = [Pin(pinId, mode=Pin.IN, pull=Pin.PULL_NONE) for pinId in (TOP_RAD_FAN1_TACH_PIN, TOP_RAD_FAN2_TACH_PIN, TOP_RAD_FAN3_TACH_PIN, TOP_RAD_FAN4_TACH_PIN)]
        self._bottomRadTopFansTachPins = [Pin(pinId, mode=Pin.IN, pull=Pin.PULL_NONE) for pinId in (BOTTOM_RAD_TOP_FAN1_TACH_PIN, BOTTOM_RAD_TOP_FAN2_TACH_PIN, BOTTOM_RAD_TOP_FAN3_TACH_PIN, BOTTOM_RAD_TOP_FAN4_TACH_PIN)]
        self._bottomRadBottomFansTachPins = [Pin(pinId, mode=Pin.IN, pull=Pin.PULL_NONE) for pinId in (BOTOM_RAD_BOTTOM_FAN1_TACH_PIN, BOTOM_RAD_BOTTOM_FAN2_TACH_PIN, BOTOM_RAD_BOTTOM_FAN3_TACH_PIN, BOTOM_RAD_BOTTOM_FAN4_TACH_PIN)]

        self._topRadFansTachPinsLevels = [(0,0), (0,0), (0,0), (0,0)]       # each tuple is (last_level, last_micros)
        self._bottomRadTopFansTachPinsLevels = [(0,0), (0,0), (0,0), (0,0)]
        self._bottomRadBottomFansTachPinsLevels = [(0,0), (0,0), (0,0), (0,0)]

        self._temperatureReadingCounter = 0
        self._cpuInTemperatureDataReady = False
        self._timerFansPwm = Timer(FANS_PWM_TIMER, freq=25000)
        self._timerTemperatureReadingIsr = Timer(TEMPERATURE_READING_ISR_TIMER, freq = NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND)

        self._channelTopRadPwm = self._timerFansPwm.channel(TOP_RAD_FANS_PWM_CHANNEL, Timer.PWM)
        self._channelBottomRadTopFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_TOP_FANS_PWM_CHANNEL, Timer.PWM)
        self._channelBottomRadBottomFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL, Timer.PWM)

        self._setTopRadFansPwnInPercent(MINIMUM_RPM_DUTY_TIME)
        self._setBottomRadTopFansPwnInPercent(MINIMUM_RPM_DUTY_TIME)
        self._setBottomRadBottomFansPwnInPercent(MINIMUM_RPM_DUTY_TIME)

        self._adcCpuInWaterTemp = ADC(CPU_IN_WATER_TEMP_ADC_PIN)
        # 725 = reading for 25 deg. C
        self._cpuInWaterAdcReadings = [725 for c in range(NUMBER_OF_READINGS_ARRAY_SIZE)]
        self._timerTemperatureReadingIsr.callback(readTemperatureISR)
        self._nextDisplayTime = 1000 * SECONDS_BETWEEN_DISPLAY_UPDATE


    def _probeCpuInWaterTemperature(self):   # To be called only by ISR
        counter = self._temperatureReadingCounter
        self._cpuInWaterAdcReadings[counter] = self._adcCpuInWaterTemp.read()
        counter += 1
        self._temperatureReadingCounter = counter % NUMBER_OF_READINGS_ARRAY_SIZE


    def _setTopRadFansPwnInPercent(self, dutyTimeInPercent):
        self._channelTopRadPwm.pulse_width_percent(min(100, max(MINIMUM_RPM_DUTY_TIME, dutyTimeInPercent)))

    def _setBottomRadTopFansPwnInPercent(self, dutyTimeInPercent):
        self._channelBottomRadTopFansPwm.pulse_width_percent(min(100, max(MINIMUM_RPM_DUTY_TIME, dutyTimeInPercent)))

    def _setBottomRadBottomFansPwnInPercent(self, dutyTimeInPercent):
        self._channelBottomRadBottomFansPwm.pulse_width_percent(min(100, max(MINIMUM_RPM_DUTY_TIME, dutyTimeInPercent)))

    def _cpuInWaterTemperature(self):
        irqState = pyb.disable_irq()    # critical section
        averageAdcReading = sum(self._cpuInWaterAdcReadings) / (NUMBER_OF_READINGS_ARRAY_SIZE * 1.0)
        pyb.enable_irq(irqState)        # end of critical section

        sensorResistance = ((4095.0 / averageAdcReading) - 1.0) * TEMPERATURE_SENSOR_DIVIDER_RESISTANCE
        # From MatLab curve fitting, adding 0.2 for compensation with the other temp indicator
        return 0.2 + (-.0019 * sensorResistance * sensorResistance + 38.14 * sensorResistance + 43870) / (sensorResistance - 827)

    def _displayIfDisplayTimeElapsed(self):
        now = fortyNineDaysMillis()
        if now > self._nextDisplayTime:
            self._nextDisplayTime = now + 1000 * SECONDS_BETWEEN_DISPLAY_UPDATE
            if self._nextDisplayTime > 0xffffffff:
                self._nextDisplayTime = 1000 * SECONDS_BETWEEN_DISPLAY_UPDATE
            print("%3.1f" % self._cpuInWaterTemperature())
            print(pyb.elapsed_micros(0))
            print()

    def _adjustFansSpeeds(self):
        pass

    def _pollTachPins(self):
        for index, pin in enumerate(self._topRadFansTachPins):
            lastTransitionLevel, lastTransitionMicros = self._topRadFansTachPinsLastLevels[index]
            currentLevel = pin.value()
            if currentLevel != lastTransitionLevel:
                nowMicros = pyb.elapsed_micros(0)




    def _calculateFansSpeed(self):
        self._pollTachPins()


    def mainLoop(self):
        while True:
            self._displayIfDisplayTimeElapsed()
            # self._calculateFansSpeed()
            self._adjustFansSpeeds()


controller = Controller()   # controller global variable needed by ISR
controller.mainLoop()
