import pyb, utime, stm, math
from pyb import Pin, Timer, ADC
from array import array

# The following pins should be in IN mode, no pull
CPU_IN_WATER_TEMP_ADC_PIN = Pin.board.X19
TOP_RAD_FANS_PWM_PIN = Pin.board.X2
BOTTOM_RAD_TOP_FANS_PWM_PIN = Pin.board.X3
BOTTOM_RAD_BOTTOM_FANS_PWM_PIN = Pin.board.X4

TOTAL_NUMBER_OF_RADIATOR_FANS = const(12)
# For the following indexes, the byte MSB is 0 for GPIOB & 1 for GPIOC
TOP_RAD_FAN1_TACH_PIN_IDR_INDEX = const(0x80 + 2)           # PC2 - X21
TOP_RAD_FAN2_TACH_PIN_IDR_INDEX = const(0x80 + 3)           # PC3 - X22
TOP_RAD_FAN3_TACH_PIN_IDR_INDEX = const(0x80 + 4)           # PC4 - X11
TOP_RAD_FAN4_TACH_PIN_IDR_INDEX = const(0x80 + 5)           # PC5 - X12
BOTTOM_RAD_TOP_FAN1_TACH_PIN_IDR_INDEX = const(12)          # PB12 - Y5
BOTTOM_RAD_TOP_FAN2_TACH_PIN_IDR_INDEX = const(13)          # PB13 - Y6
BOTTOM_RAD_TOP_FAN3_TACH_PIN_IDR_INDEX = const(14)          # PB14 - Y7
BOTTOM_RAD_TOP_FAN4_TACH_PIN_IDR_INDEX = const(15)          # PB15 - Y8
BOTOM_RAD_BOTTOM_FAN1_TACH_PIN_IDR_INDEX = const(0x80 + 6)  # PC6 - Y1
BOTOM_RAD_BOTTOM_FAN2_TACH_PIN_IDR_INDEX = const(0x80 + 7)  # PC7 - Y2
BOTOM_RAD_BOTTOM_FAN3_TACH_PIN_IDR_INDEX = const(10)        # PB10 - Y3 only for PyBoard Lite (PB8 on full PyBoard)
BOTOM_RAD_BOTTOM_FAN4_TACH_PIN_IDR_INDEX = const(9)         # PB9 - Y4

TEMPERATURE_READING_ISR_TIMER = const(9)
FANS_SPEED_UPDATE_ISR_TIMER = const(10)
FANS_PWM_TIMER = const(2)
TOP_RAD_FANS_PWM_CHANNEL = const(4)             # PyBoard Lite only !
BOTTOM_RAD_TOP_FANS_PWM_CHANNEL = const(1)      # PyBoard Lite only !
BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL = const(2)   # PyBoard Lite only !

MINIMUM_RPM_DUTY_TIME = const(20)

TEMPERATURE_SENSOR_DIVIDER_RESISTANCE = const(2200)    # we have a 2k2 from adc pin to ground

SECONDS_BETWEEN_DISPLAY_UPDATE = const(2)
NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND = const(10)
NUMBER_OF_READINGS_ARRAY_SIZE = const(NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND * SECONDS_BETWEEN_DISPLAY_UPDATE)

@micropython.viper
def readTemperatureISR(timer):
    global controller
    controller._probeCpuInWaterTemperature()

@micropython.viper
def calculateFansSpeedISR(timer):
    global controller
    controller._calculateFansSpeed()


def twentyFourDaysMillis():
    now = pyb.millis() # pyb.millis() can be negative after 12 days, wrap around after 24 days (micropython small integers use 31 bits)
    return now if now >= 0 else 0x80000000 - now

@micropython.asm_thumb
def readGPIOB_IDR():
    movwt(r1, stm.GPIOB)        # r1 contains the base address of GPIOB
    ldr(r0, [r1, stm.GPIO_IDR]) # The content of GPIOB base address + offset of IDR is loaded in r0, r0 is the result of the function

@micropython.asm_thumb
def readGPIOC_IDR():
    movwt(r1, stm.GPIOC)        # r1 contains the base address of GPIOC
    ldr(r0, [r1, stm.GPIO_IDR]) # The content of GPIOC base address + offset of IDR is loaded in r0, r0 is the result of the function


class Controller:
    def __init__(self):
        self._radFansTachPinsIndexes = array('B', (TOP_RAD_FAN1_TACH_PIN_IDR_INDEX, TOP_RAD_FAN2_TACH_PIN_IDR_INDEX,
                                                TOP_RAD_FAN3_TACH_PIN_IDR_INDEX, TOP_RAD_FAN4_TACH_PIN_IDR_INDEX,
                                                BOTTOM_RAD_TOP_FAN1_TACH_PIN_IDR_INDEX, BOTTOM_RAD_TOP_FAN2_TACH_PIN_IDR_INDEX,
                                                BOTTOM_RAD_TOP_FAN3_TACH_PIN_IDR_INDEX, BOTTOM_RAD_TOP_FAN4_TACH_PIN_IDR_INDEX,
                                                BOTOM_RAD_BOTTOM_FAN1_TACH_PIN_IDR_INDEX, BOTOM_RAD_BOTTOM_FAN2_TACH_PIN_IDR_INDEX,
                                                BOTOM_RAD_BOTTOM_FAN3_TACH_PIN_IDR_INDEX, BOTOM_RAD_BOTTOM_FAN4_TACH_PIN_IDR_INDEX))
        self._radFansTachPinsLastLevels = array('B', [0 for _ in range(TOTAL_NUMBER_OF_RADIATOR_FANS)])
        nowTimeStamp = utime.ticks_us()
        self._radFansTachPinsLastTimeStamps = array('i', [nowTimeStamp for _ in range(TOTAL_NUMBER_OF_RADIATOR_FANS)])
        self._radFansTachPulseCounters = array('i', [0 for _ in range(TOTAL_NUMBER_OF_RADIATOR_FANS)])
        self._radFansRPMs = array('i', [0 for _ in range(TOTAL_NUMBER_OF_RADIATOR_FANS)])

        self._temperatureReadingCounter = 0
        self._cpuInTemperatureDataReady = False
        self._timerFansPwm = Timer(FANS_PWM_TIMER, freq=25000)
        self._timerTemperatureReadingIsr = Timer(TEMPERATURE_READING_ISR_TIMER, freq = NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND)
        # ISR will trigger every 3.75", this allows rpm = numPulses << 3
        self._timerCalculateFansSpeedIsr = Timer(FANS_SPEED_UPDATE_ISR_TIMER, prescaler=11718, period=30719)

        self._channelTopRadPwm = self._timerFansPwm.channel(TOP_RAD_FANS_PWM_CHANNEL, Timer.PWM, pin=TOP_RAD_FANS_PWM_PIN)
        self._channelBottomRadTopFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_TOP_FANS_PWM_CHANNEL, Timer.PWM, pin=BOTTOM_RAD_TOP_FANS_PWM_PIN)
        self._channelBottomRadBottomFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL, Timer.PWM, pin=BOTTOM_RAD_BOTTOM_FANS_PWM_PIN)

        self._setAllFansPwmInPercent(MINIMUM_RPM_DUTY_TIME)

        self._adcCpuInWaterTemp = ADC(CPU_IN_WATER_TEMP_ADC_PIN)
        # 725 = reading for 25 deg. C
        self._cpuInWaterAdcReadings = [725 for c in range(NUMBER_OF_READINGS_ARRAY_SIZE)]
        self._timerTemperatureReadingIsr.callback(readTemperatureISR)
        self._timerCalculateFansSpeedIsr.callback(calculateFansSpeedISR)
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

    def _setAllFansPwmInPercent(self, dutyTimeInPercent):
        self._setTopRadFansPwnInPercent(dutyTimeInPercent)
        self._setBottomRadTopFansPwnInPercent(dutyTimeInPercent)
        self._setBottomRadBottomFansPwnInPercent(dutyTimeInPercent)

    @micropython.native
    def _cpuInWaterTemperature(self):
        irqState = pyb.disable_irq()    # critical section
        averageAdcReading = sum(self._cpuInWaterAdcReadings) / (NUMBER_OF_READINGS_ARRAY_SIZE * 1.0)
        pyb.enable_irq(irqState)        # end of critical section

        sensorResistance = ((4095.0 / averageAdcReading) - 1.0) * TEMPERATURE_SENSOR_DIVIDER_RESISTANCE
        # Coefficients from MatLab curve fitting, adding 0.2 for compensation with the other temp indicator
        return 0.2 + (-.0019 * sensorResistance * sensorResistance + 38.14 * sensorResistance + 43870) / (sensorResistance - 827)

    def _displayIfDisplayTimeElapsed(self):
        now = twentyFourDaysMillis()
        if now > self._nextDisplayTime:
            self._nextDisplayTime = now + 1000 * SECONDS_BETWEEN_DISPLAY_UPDATE
            if self._nextDisplayTime > 0xffffffff:
                self._nextDisplayTime = 1000 * SECONDS_BETWEEN_DISPLAY_UPDATE
            print("%3.1f" % self._cpuInWaterTemperature())
            for i in range(TOTAL_NUMBER_OF_RADIATOR_FANS):
                irqState = pyb.disable_irq()    # critical section
                rpm = self._radFansRPMs[i]
                pyb.enable_irq(irqState)        # end of critical section
                print("%d " % (50 * round(rpm / 50.0)), end='')
            print('\n')

    def _adjustFansSpeeds(self):
        pass

    @micropython.native
    def _pollTachPins(self):
        nowTimeStamp = utime.ticks_us()
        for i, gpioIdrIndex in enumerate(self._radFansTachPinsIndexes):
            bitNumber = gpioIdrIndex & 0x0f
            gpioLevels = readGPIOC_IDR() if gpioIdrIndex & 0x80 else readGPIOB_IDR()
            newLevel = (gpioLevels & (1 << bitNumber)) >> bitNumber # Must shift right the bit as we store the result in a byte array
            lastlevel = self._radFansTachPinsLastLevels[i]
            if newLevel != lastlevel:
                lastTimeStamp = self._radFansTachPinsLastTimeStamps[i]
                elapsedTime = utime.ticks_diff(lastTimeStamp, nowTimeStamp)
                if elapsedTime > 1000:      # if it is less than 1 ms, we consider it a bounce and disregard it
                    # We record the change on any transition, L to H or H to L
                    self._radFansTachPinsLastLevels[i] = newLevel
                    self._radFansTachPinsLastTimeStamps[i] = nowTimeStamp
                    # But we only count rising edges
                    if newLevel:            # it's a rising edge
                        irqState = pyb.disable_irq()    # critical section
                        self._radFansTachPulseCounters[i] += 1
                        pyb.enable_irq(irqState)        # end of critical section

    #Called by ISR every 3.75"
    @micropython.native
    def _calculateFansSpeed(self):
        arrPC = self._radFansTachPulseCounters
        arrRPM = self._radFansRPMs
        for i in range(TOTAL_NUMBER_OF_RADIATOR_FANS):
            arrRPM[i] = arrPC[i] << 3
            arrPC[i] = 0

    def mainLoop(self):
        while True:
            self._displayIfDisplayTimeElapsed()
            self._pollTachPins()


controller = Controller()   # controller global variable needed by ISR
controller.mainLoop()
