import pyb, utime, stm, math
from pyb import Pin, Timer, ADC
from array import array

# The following pins should be in IN mode, no pull
CPU_IN_WATER_TEMP_ADC_PIN = Pin.board.X19
TOP_RAD_FANS_PWM_PIN = Pin.board.X2
BOTTOM_RAD_TOP_FANS_PWM_PIN = Pin.board.X3
BOTTOM_RAD_BOTTOM_FANS_PWM_PIN = Pin.board.X4

# For the following indexes, the byte MSB is 0 for GPIOB & 1 for GPIOC
BOTOM_RAD_BOTTOM_FAN1_TACH_PIN_IDR_INDEX = const(0x80 + 6)  # PC6 - Y1
BOTOM_RAD_BOTTOM_FAN2_TACH_PIN_IDR_INDEX = const(0x80 + 7)  # PC7 - Y2
BOTOM_RAD_BOTTOM_FAN3_TACH_PIN_IDR_INDEX = const(10)        # PB10 - Y3for PyBoard Lite (PB8 on full PyBoard)
BOTOM_RAD_BOTTOM_FAN4_TACH_PIN_IDR_INDEX = const(9)         # PB9 - Y4

BOTTOM_RAD_TOP_FAN1_TACH_PIN_IDR_INDEX = const(12)          # PB12 - Y5
BOTTOM_RAD_TOP_FAN2_TACH_PIN_IDR_INDEX = const(13)          # PB13 - Y6
BOTTOM_RAD_TOP_FAN3_TACH_PIN_IDR_INDEX = const(14)          # PB14 - Y7
BOTTOM_RAD_TOP_FAN4_TACH_PIN_IDR_INDEX = const(15)          # PB15 - Y8

TOP_RAD_FAN1_TACH_PIN_IDR_INDEX = const(0x80 + 2)           # PC2 - X21
TOP_RAD_FAN2_TACH_PIN_IDR_INDEX = const(0x80 + 3)           # PC3 - X22
TOP_RAD_FAN3_TACH_PIN_IDR_INDEX = const(0x80 + 4)           # PC4 - X11
TOP_RAD_FAN4_TACH_PIN_IDR_INDEX = const(0x80 + 5)           # PC5 - X12

TEMPERATURE_READING_ISR_TIMER = const(9)
FANS_SPEED_UPDATE_ISR_TIMER = const(10)
FANS_PWM_TIMER = const(2)
TOP_RAD_FANS_PWM_CHANNEL = const(4)    # PyBoard Lite only !
BOTTOM_RAD_TOP_FANS_PWM_CHANNEL = const(1)    # PyBoard Lite only !
BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL = const(2)    # PyBoard Lite only !

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


def fortyNineDaysMillis():
    now = pyb.millis() # pyb.millis() can be negative after 24 days wrap around (32 bit integer, 2^32 = 49 days)
    return now if now >= 0 else 0x80000000 - now

@micropython.asm_thumb
def readGPIOBIdr():
    movwt(r1, stm.GPIOB)        # r1 contains the base address of GPIOB
    ldr(r0, [r1, stm.GPIO_IDR]) # The content of GPIOB base address + offset of IDR is loaded in r0, r0 is the result of the function

@micropython.asm_thumb
def readGPIOCIdr():
    movwt(r1, stm.GPIOC)        # r1 contains the base address of GPIOC
    ldr(r0, [r1, stm.GPIO_IDR]) # The content of GPIOC base address + offset of IDR is loaded in r0, r0 is the result of the function


class Controller:
    def __init__(self):
        self._topRadFansTachPinsIndexes = array('B', (TOP_RAD_FAN1_TACH_PIN_IDR_INDEX, TOP_RAD_FAN2_TACH_PIN_IDR_INDEX,
            TOP_RAD_FAN3_TACH_PIN_IDR_INDEX, TOP_RAD_FAN4_TACH_PIN_IDR_INDEX))
        self._bottomRadTopFansTachPinsIndexes = array('B', (BOTTOM_RAD_TOP_FAN1_TACH_PIN_IDR_INDEX, BOTTOM_RAD_TOP_FAN2_TACH_PIN_IDR_INDEX,
            BOTTOM_RAD_TOP_FAN3_TACH_PIN_IDR_INDEX, BOTTOM_RAD_TOP_FAN4_TACH_PIN_IDR_INDEX))
        self._bottomRadBottomFansTachPinsIndexes = array('B', (BOTOM_RAD_BOTTOM_FAN1_TACH_PIN_IDR_INDEX, BOTOM_RAD_BOTTOM_FAN2_TACH_PIN_IDR_INDEX,
            BOTOM_RAD_BOTTOM_FAN3_TACH_PIN_IDR_INDEX, BOTOM_RAD_BOTTOM_FAN4_TACH_PIN_IDR_INDEX))

        self._topRadFansTachPinsLastLevels = array('B', (0, 0, 0, 0))
        self._bottomRadTopFansTachPinsLastLevels = array('B', (0, 0, 0, 0))
        self._bottomRadBottomFansTachPinsLastLevels = array('B', (0, 0, 0, 0))

        self._topRadFansTachPinsLastTimeStamps = array('i', (0, 0, 0, 0))
        self._bottomRadTopFansTachPinsLastTimeStamps = array('i', (0, 0, 0, 0))
        self._bottomRadBottomFansTachPinsLastTimeStamps = array('i', (0, 0, 0, 0))

        self._topRadFansTachPulseCounters = array('i', (0, 0, 0, 0))
        self._bottomRadTopFansTachPulseCounters = array('i', (0, 0, 0, 0))
        self._bottomRadBottomFansTachPulseCounters = array('i', (0, 0, 0, 0))

        self._topRadFansRPMs = array('i', (0, 0, 0, 0))
        self._bottomRadTopFansRPMs = array('i', (0, 0, 0, 0))
        self._bottomRadBottomFansRPMs = array('i', (0, 0, 0, 0))

        self._temperatureReadingCounter = 0
        self._cpuInTemperatureDataReady = False
        self._timerFansPwm = Timer(FANS_PWM_TIMER, freq=25000)
        self._timerTemperatureReadingIsr = Timer(TEMPERATURE_READING_ISR_TIMER, freq = NUMBER_OF_TEMPERATURE_READINGS_PER_SECOND)
        # ISR will trigger every 3.75", this allows rpm = numPulses << 3
        self._timerCalculateFansSpeedIsr = Timer(FANS_SPEED_UPDATE_ISR_TIMER, prescaler=11718, period=30719)

        self._channelTopRadPwm = self._timerFansPwm.channel(TOP_RAD_FANS_PWM_CHANNEL, Timer.PWM, pin=TOP_RAD_FANS_PWM_PIN)
        self._channelBottomRadTopFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_TOP_FANS_PWM_CHANNEL, Timer.PWM, pin=BOTTOM_RAD_TOP_FANS_PWM_PIN)
        self._channelBottomRadBottomFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL, Timer.PWM, pin=BOTTOM_RAD_BOTTOM_FANS_PWM_PIN)

        self._setTopRadFansPwnInPercent(MINIMUM_RPM_DUTY_TIME)
        self._setBottomRadTopFansPwnInPercent(MINIMUM_RPM_DUTY_TIME)
        self._setBottomRadBottomFansPwnInPercent(MINIMUM_RPM_DUTY_TIME)

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
            for i in range(4):
                print("%d " % (50 * round(self._topRadFansRPMs[i] / 50.0)), end='')
            print()
            for i in range(4):
                print("%d " % (50 * round(self._bottomRadTopFansRPMs[i] / 50.0)), end='')
            print()
            for i in range(4):
                print("%d " % (50 * round(self._bottomRadBottomFansRPMs[i] / 50.0)), end='')
            print('\n')

    def _adjustFansSpeeds(self):
        pass


    @micropython.native
    def _pollTachPins(self):
        # utime.ticks_us() wraps after 17.9', max value 0x3fffffff (30 bits), counts up
        nowTimeStamp = utime.ticks_us()
        gpioBLevels = readGPIOBIdr()
        gpioCLevels = readGPIOCIdr()

        for i, gpioIdrIndex in enumerate(self._topRadFansTachPinsIndexes):
            bitNumber = gpioIdrIndex & 0x0f
            isGPIOC = gpioIdrIndex & 0x80
            if isGPIOC:
                newLevel = gpioCLevels & (1 << bitNumber)
            else:
                newLevel = gpioBLevels & (1 << bitNumber)
            lastlevel = self._topRadFansTachPinsLastLevels[i]
            if newLevel != lastlevel:   # newlevel is high, the rising edge of the pulse
                lastTimeStamp = self._topRadFansTachPinsLastTimeStamps[i]
                elapsedTime = utime.ticks_diff(lastTimeStamp, nowTimeStamp)
                if elapsedTime > 1000:      # if it is less than 1 ms, we consider it a bounce and disregard it
                    self._topRadFansTachPinsLastLevels[i] = newLevel
                    self._topRadFansTachPinsLastTimeStamps[i] = nowTimeStamp
                    if newLevel:            # it's a rising edge
                        irqState = pyb.disable_irq()
                        self._topRadFansTachPulseCounters[i] += 1
                        pyb.enable_irq(irqState)

        for i, gpioIdrIndex in enumerate(self._bottomRadTopFansTachPinsIndexes):
            bitNumber = gpioIdrIndex & 0x0f
            isGPIOC = gpioIdrIndex & 0x80
            if isGPIOC:
                newLevel = gpioCLevels & (1 << bitNumber)
            else:
                newLevel = gpioBLevels & (1 << bitNumber)
            lastlevel = self._bottomRadTopFansTachPinsLastLevels[i]
            if newLevel != lastlevel:   # newlevel is high, the rising edge of the pulse
                lastTimeStamp = self._bottomRadTopFansTachPinsLastTimeStamps[i]
                elapsedTime = utime.ticks_diff(lastTimeStamp, nowTimeStamp)
                if elapsedTime > 1000:      # if it is less than 1 ms, we consider it a bounce and disregard it
                    self._bottomRadTopFansTachPinsLastLevels[i] = newLevel
                    self._bottomRadTopFansTachPinsLastTimeStamps[i] = nowTimeStamp
                    if newLevel:            # it's a rising edge
                        irqState = pyb.disable_irq()
                        self._bottomRadTopFansTachPulseCounters[i] += 1
                        pyb.enable_irq(irqState)

        for i, gpioIdrIndex in enumerate(self._bottomRadBottomFansTachPinsIndexes):
            bitNumber = gpioIdrIndex & 0x0f
            isGPIOC = gpioIdrIndex & 0x80
            if isGPIOC:
                newLevel = gpioCLevels & (1 << bitNumber)
            else:
                newLevel = gpioBLevels & (1 << bitNumber)
            lastlevel = self._bottomRadBottomFansTachPinsLastLevels[i]
            if newLevel != lastlevel:   # newlevel is high, the rising edge of the pulse
                lastTimeStamp = self._bottomRadBottomFansTachPinsLastTimeStamps[i]
                elapsedTime = utime.ticks_diff(lastTimeStamp, nowTimeStamp)
                if elapsedTime > 1000:      # if it is less than 1 ms, we consider it a bounce and disregard it
                    self._bottomRadBottomFansTachPinsLastLevels[i] = newLevel
                    self._bottomRadBottomFansTachPinsLastTimeStamps[i] = nowTimeStamp
                    if newLevel:            # it's a rising edge
                        irqState = pyb.disable_irq()
                        self._bottomRadBottomFansTachPulseCounters[i] += 1
                        pyb.enable_irq(irqState)

    #Called by ISR every 3.75"
    @micropython.native
    def _calculateFansSpeed(self):
        arrT = self._topRadFansTachPulseCounters
        arrBT = self._bottomRadTopFansTachPulseCounters
        arrBB = self._bottomRadBottomFansTachPulseCounters
        arrTRPM = self._topRadFansRPMs
        arrBTRPM = self._bottomRadTopFansRPMs
        arrBBRPM = self._bottomRadBottomFansRPMs
        for i in range(4):
            arrTRPM[i] = arrT[i] << 3
            arrT[i] = 0
            arrBTRPM[i] = arrBT[i] << 3
            arrBT[i] = 0
            arrBBRPM[i] = arrBB[i] << 3
            arrBB[i] = 0

    def mainLoop(self):
        while True:
            self._displayIfDisplayTimeElapsed()
            self._pollTachPins()


controller = Controller()   # controller global variable needed by ISR
controller.mainLoop()
