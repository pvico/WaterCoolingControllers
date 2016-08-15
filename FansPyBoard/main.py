import pyb
from pyb import Pin, Timer

TOP_RAD_FANS_PWM_PIN = Pin.board.X2
BOTTOM_RAD_TOP_FANS_PWM_PIN = Pin.board.X3
BOTTOM_RAD_BOTTOM_FANS_PWM_PIN = Pin.board.X4

FANS_PWM_TIMER = 2
TOP_RAD_FANS_PWM_CHANNEL = 4    # PyBoard Lite only !
BOTTOM_RAD_TOP_FANS_PWM_CHANNEL = 1    # PyBoard Lite only !
BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL = 2    # PyBoard Lite only !

MINIMUM_RPM_DUTY_TIME = 10

class Controller:
    def __init__(self):
        self._timerFansPwm = Timer(FANS_PWM_TIMER, freq=25000)

        self._channelTopRadPwm = self._timerFansPwm.channel(TOP_RAD_FANS_PWM_CHANNEL, Timer.PWM)
        self._channelBottomRadTopFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_TOP_FANS_PWM_CHANNEL, Timer.PWM)
        self._channelBottomRadBottomFansPwm = self._timerFansPwm.channel(BOTTOM_RAD_BOTTOM_FANS_PWM_CHANNEL, Timer.PWM)

        self.setAllFansMinSpeed()

        # self._pinPwmTopRadFans = Pin(TOP_RAD_FANS_PWM_PIN, Pin.OUT_PP)
        # self._pinPwmBottomRadTopFans = Pin(BOTTOM_RAD_TOP_FANS_PWM_PIN, Pin.OUT_PP)
        # self._pinPwmBottomRadBottomFans = Pin(BOTTOM_RAD_BOTTOM_FANS_PWM_PIN, Pin.OUT_PP)

    def setAllFansMinSpeed(self):
        self._channelTopRadPwm.pulse_width_percent(MINIMUM_RPM_DUTY_TIME)
        self._channelBottomRadTopFansPwm.pulse_width_percent(MINIMUM_RPM_DUTY_TIME)
        self._channelBottomRadBottomFansPwm.pulse_width_percent(MINIMUM_RPM_DUTY_TIME)




def mainLoop(controller):
    while True:
        pyb.delay(1000)
        print(controller._channelBottomRadBottomFansPwm.pulse_width())


controller = Controller()

mainLoop(controller)
