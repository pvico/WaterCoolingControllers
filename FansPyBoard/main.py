import pyb, stm, math
from pyb import Pin, Timer, ADC

# The following pins should be in IN mode, no pull
CPU_IN_WATER_TEMP_ADC_PIN = Pin.board.X19
TOP_RAD_FANS_PWM_PIN = Pin.board.X2
BOTTOM_RAD_TOP_FANS_PWM_PIN = Pin.board.X3
BOTTOM_RAD_BOTTOM_FANS_PWM_PIN = Pin.board.X4

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

TEMPERATURE_READING_ISR_TIMER = const(9)
FANS_PWM_TIMER = const(2)
TOP_RAD_FANS_PWM_CHANNEL = const(4)    # PyBoard Lite only !
BOTTOM_RAD_TOP_FANS_PWM_CHANNEL = const(1)    # PyBoard Lite only !
BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL = const(2)    # PyBoard Lite only !

MINIMUM_RPM_DUTY_TIME = const(10)

TEMPERATURE_SENSOR_DIVIDER_RESISTANCE = const(2200)    # we have a 2k2 from adc pin to ground


SECONDS_BETWEEN_DISPLAY_UPDATE = const(2)
NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND = const(10)
NUMBER_OF_READINGS_ARRAY_SIZE = const(NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND * SECONDS_BETWEEN_DISPLAY_UPDATE)

@micropython.viper
def readTemperatureISR(timer):
    global controller

    controller._probeCpuInWaterTemperature()


def fortyNineDaysMillis():
    now = pyb.millis() # pyb.millis() can be negative after 24 days wrap around (32 bit integer, 2^32 = 49 days)
    return now if now >= 0 else 0x80000000 - now

@micropython.asm_thumb
def readGPIOBInput():
    movwt(r1, stm.GPIOB)        # r1 contains the base address of GPIOB
    ldr(r0, [r1, stm.GPIO_IDR]) # The content of GPIOB base address + offset of IDR is loaded in r0, r0 is the result of the function

@micropython.asm_thumb
def readGPIOCInput():
    movwt(r1, stm.GPIOC)        # r1 contains the base address of GPIOC
    ldr(r0, [r1, stm.GPIO_IDR]) # The content of GPIOC base address + offset of IDR is loaded in r0, r0 is the result of the function


class Controller:
    def __init__(self):
        self._topRadFansTachPins = [TOP_RAD_FAN1_TACH_PIN, TOP_RAD_FAN2_TACH_PIN, TOP_RAD_FAN3_TACH_PIN, TOP_RAD_FAN4_TACH_PIN]
        self._bottomRadTopFansTachPins = [BOTTOM_RAD_TOP_FAN1_TACH_PIN, BOTTOM_RAD_TOP_FAN2_TACH_PIN, BOTTOM_RAD_TOP_FAN3_TACH_PIN, BOTTOM_RAD_TOP_FAN4_TACH_PIN]
        self._bottomRadBottomFansTachPins = [BOTOM_RAD_BOTTOM_FAN1_TACH_PIN, BOTOM_RAD_BOTTOM_FAN2_TACH_PIN, BOTOM_RAD_BOTTOM_FAN3_TACH_PIN, BOTOM_RAD_BOTTOM_FAN4_TACH_PIN]

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

    @micropython.native
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
            print("%s" % bin(readGPIOCInput()))
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
