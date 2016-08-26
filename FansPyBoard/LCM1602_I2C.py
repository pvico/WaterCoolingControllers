from pyb import I2C
from pyb import delay, udelay


I2C_PORT = const(1)
I2C_ADDRESS = const(0x27)
HD44780_BACKLIGHT_PIN = const(3)
HD44780_EN_PIN = const(2)
HD44780_RW_PIN = const(1)
HD44780_RS_PIN = const(0)
HD44780_D4_PIN = const(4)
HD44780_D5_PIN = const(5)
HD44780_D6_PIN = const(6)
HD44780_D7_PIN = const(7)

COMMAND = const(0)
DATA = const(1)
FOUR_BITS = const(2)

LCD_CLEARDISPLAY = const(0x01)
LCD_RETURNHOME = const(0x02)
HOME_CLEAR_EXEC = const(2000)
LCD_DISPLAYCONTROL = const(0x08)
LCD_DISPLAYON = const(0x04)
LCD_DISPLAYOFF = const(0x00)
LCD_CURSORON = const(0x02)
LCD_CURSOROFF = const(0x00)
LCD_BLINKON = const(0x01)
LCD_BLINKOFF = const(0x00)
LCD_SETCGRAMADDR = const(0x40)
LCD_SETDDRAMADDR = const(0x80)


class LCM1602_I2C:
    def __init__(self, cols = 16, rows=2, i2cPort=I2C_PORT):
        self._En = 1 << HD44780_EN_PIN
        self._Rw = 1 << HD44780_RW_PIN
        self._Rs = 1 << HD44780_RS_PIN
        self._data_pins = bytearray((0,0,0,0))
        self._data_pins[0] = 1 << HD44780_D4_PIN
        self._data_pins[1] = 1 << HD44780_D5_PIN
        self._data_pins[2] = 1 << HD44780_D6_PIN
        self._data_pins[3] = 1 << HD44780_D7_PIN
        self._cols = cols
        self._rows = rows
        self._backlightPinMask = 1 << HD44780_BACKLIGHT_PIN
        self._backlightStsMask = 0
        self._displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF
        self._i2c = I2C(I2C_PORT, I2C.MASTER, baudrate=100000)
        self._i2c.send(0, I2C_ADDRESS)
        self.display()
        self.backlight()
        self.clear()
        self.print('  Watercooling')
        self.setCursor(0, 1)
        self.print(' Fan Controller')
        delay(2000)
        self.clear()
        self.print('by Philippe Vico')
        self.setCursor(0, 1)
        self.print('   26/08/2016')
        # self.clear()

    def backlight(self):
        self._backlightStsMask = self._backlightPinMask & 0xff
        self._i2c.send(self._backlightStsMask, I2C_ADDRESS)

    def noBacklight(self):
        self._backlightStsMask = self._backlightPinMask & 0
        self._i2c.send(self._backlightStsMask, I2C_ADDRESS)

    def clear(self):
        self._command(LCD_CLEARDISPLAY)           # clear display, set cursor position to zero
        udelay(HOME_CLEAR_EXEC)                   # this command is time consuming

    def home(self):
        self._command(LCD_RETURNHOME)             # set cursor position to zero
        udelay(HOME_CLEAR_EXEC)                   # this command is time consuming

    def noDisplay(self):
        self._displaycontrol &= ~LCD_DISPLAYON
        self._command(LCD_DISPLAYCONTROL | self._displaycontrol)

    def display(self):
        self._displaycontrol |= LCD_DISPLAYON
        self._command(LCD_DISPLAYCONTROL | self._displaycontrol)

    def noBlink(self):
        pass

    def blink(self):
        pass

    def noCursor(self):
        pass

    def cursor(self):
        pass

    def scrollDisplayLeft(self):
        pass

    def scrollDisplayRight(self):
        pass

    def leftToRight(self):
        pass

    def rightToLeft(self):
        pass

    def moveCursorLeft(self):
        pass

    def moveCursorRight(self):
        pass

    def autoscroll(self):
        pass

    def noAutoscroll(self):
        pass

    def createChar(self, location, charmap):
        pass

    def setCursor(self, col, row):
        row_offsetsDef = bytearray((0x00, 0x40, 0x14, 0x54))        # For regular LCDs
        row_offsetsLarge = bytearray((0x00, 0x40, 0x10, 0x50))      # For 16x4 LCDs

        if row >= self._rows:
            row = self._rows - 1        # rows start at 0

        # 16x4 LCDs have special memory map layout
        if (self._cols == 16) and (self._rows == 4):
            self._command(LCD_SETDDRAMADDR | (col + row_offsetsLarge[row]))
        else:
            self._command(LCD_SETDDRAMADDR | (col + row_offsetsDef[row]))

    def on(self):
        self.display()
        self.backlight()

    def off(self):
        self.noBacklight()
        self.noDisplay()

    def write(self, character):
        self._send(character, DATA)

    def print(self, text):
        for char in bytes(text, 'utf-8'):
            self.write(char)

    def _command(self, value):
        self._send(value, COMMAND)

    def _send(self, value, mode):
        if mode == FOUR_BITS:
            self._write4bits((value & 0x0f), COMMAND)
        else:
            self._write4bits((value >> 4), mode)
            self._write4bits((value & 0x0f), mode)

    @micropython.native
    def _write4bits(self, value, mode):
        pinMapValue = 0
        if (value & 0x1):
            pinMapValue |= self._data_pins[0]
        if (value & 0x2):
            pinMapValue |= self._data_pins[1]
        if (value & 0x4):
            pinMapValue |= self._data_pins[2]
        if (value & 0x8):
            pinMapValue |= self._data_pins[3]

        if mode == DATA:
            mode = self._Rs

        pinMapValue |= (mode | self._backlightStsMask)
        self._pulseEnable(pinMapValue)

    @micropython.native
    def _pulseEnable(self, data):
        self._i2c.send(data | self._En, I2C_ADDRESS)
        self._i2c.send(data & ~(self._En), I2C_ADDRESS)
