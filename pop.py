import os
from bme680 import *
from wiringpi import HIGH, LOW
from threading import Thread, Lock
from smbus import *
import ctypes

import math
import time
import RPi.GPIO as GPIO
import spidev

GPIO.setwarnings(False)

#for Soundmeter & Tone class
import wave
import pyaudio
import audioop
import numpy as np

#for getting callback args
import inspect

# for camera
import __main__
import cv2 as cv
#from pop import Util
#for neopixel
"""
import board
import neopixel

try:
    import _pixelbuf
except ImportError:
    import adafruit_pypixelbuf as _pixelbuf
import digitalio
from neopixel_write import neopixel_write
"""
##################################################################

binder = ctypes.cdll.LoadLibrary('/usr/local/lib/libpop.so')

##################################################################
# Device Define
AIOT_HOME   = 0x10
AIOT_SERVER = 0x11

ES_101      = 0x30

PYC_BASIC   = 0x50

# GPIO Peripheral Define
LED             = 0
SWITCH          = 1
PIEZO_BUZZER    = 2
PIXEL_DISPLAY   = 3
SHIFT_REGISTER  = 4
DC_FAN          = 5
PIR             = 6
FND             = 7

#ADC Peripheral Define
PSD             = 0
CDS             = 1
SOUND           = 2
POTENTIOMETER   = 3
GAS             = 4

# Mapping Table Define
PINMAP = list()
CHANNELMAP = list()
board_name = int()
board_name_str = str()

#---------------------------------------------------------------------------
# PyC Basic
PYC_BASIC_PINMAP = [
    [15, 16, 17, 6, 19, 20, 21, 22],    #LED
    [7, 27],                            #Push Switch
    [13],                               #Piezo
    [18],                               #Led Strip
    [0xFF],                             #ShiftRegister
    [0xFF],                             #DC Fan
    [0xFF]                              #PIR
]

PYC_BASIC_CHANNELMAP = [
    3,                                  #PSD
    1,                                  #CDS
    2,                                  #Sound
    0,                                  #Potentiometer
    0xFF                                #Gas
]

#---------------------------------------------------------------------------
# AIoT Home
AIOT_HOME_PINMAP = [
    [23, 24, 25, 27],                    #LED
    [0xFF],                             #Push Switch
    [12],                               #Piezo
    [0xFF],                             #Led Strip
    [16, 5, 6],                          #Led Bar - ShiftRegister
    [17],                               #DC Fan
    [22]                                #PIR
]

AIOT_HOME_CHANNELMAP = [
    0xFF,                                #PSD
    7,                                   #CDS
    0xFF,                                #Sound
    0xFF,                                #Potentiometer
    6                                    #Gas
]

#---------------------------------------------------------------------------
# ES-101
ES_101_PINMAP = [
    [23, 24, 25, 1],                     #LED
    [4, 17, 27, 22],                     #Push Switch
    [12],                                #Piezo
    [0xFF],                              #Led Strip
    [0, 5, 6],                              #FND - ShiftRegister
    [0xFF],                              #DC Fan
    [7]                                 #PIR
]

ES_101_CHANNELMAP = [
    0,                                   #PSD
    0xFF,                                #CDS
    0xFF,                                #Sound
    0xFF,                                #Potentiometer
    0xFF                                 #Gas
]
#---------------------------------------------------------------------------
def board_config():
    global PINMAP
    global CHANNELMAP
    global board_name
    global board_name_str

    binder.Pop_board_config()
    board_name = binder.Pop_get_board_name()

    if board_name is PYC_BASIC:
        PINMAP = PYC_BASIC_PINMAP
        CHANNELMAP = PYC_BASIC_CHANNELMAP
        board_name_str = "PyC Basic"
    elif board_name is AIOT_HOME:
        PINMAP = AIOT_HOME_PINMAP
        CHANNELMAP = AIOT_HOME_CHANNELMAP
        board_name_str = "AIoT-Home"
    elif board_name is ES_101:
        PINMAP = ES_101_PINMAP
        CHANNELMAP = ES_101_CHANNELMAP
        board_name_str = "ES-101"
    else:
        board_name_str = "None"
    #print("This board is {}".format(board_name_str))
    
def pinMap(peripheral, order=0):
    if peripheral >= len(PINMAP):
        print(len(PINMAP), PINMAP)
        print("Peripheral is out of index\n");
    elif order >= len(PINMAP[peripheral]):
        print("Order is out of index\n")
    else:
        return PINMAP[peripheral][order]

def pinMax(peripheral):
    return len(PINMAP[peripheral])

def channelMap(peripheral):
    if peripheral >= len(CHANNELMAP):
        print("Peripheral is out of index\n")
    else:
        return CHANNELMAP[peripheral]

##################################################################
board_config()
GPIO.setmode(GPIO.BCM)
##################################################################

def map(x, in_min, in_max, out_min, out_max):
    return ((x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

def delay(howLong):
    time.sleep(howLong / 1000)

def delayMicroseconds(howLong):
    time.sleep(howLong / (1000 * 1000))

##################################################################
class PopThread:
    def __init__(self):
        self._state = False    
        self._thread = None

    def __raw_run(self):
        try:
            while (self.isRun()):
                self.run()
                delay(20) #only Python thread
        finally:
            pass

    def start(self):
        self._state = True
        self._thread = Thread(target=self.__raw_run)
        self._thread.daemon = True
        self._thread.start()

    def run(self):
        pass

    def stop(self):
        self._state = False

    def isRun(self):
        return self._state
        
class My(PopThread):
    def __init__(self):
        self.i = 0

    def run(self):
        self.i += 1
        print("run: %d"%(self.i))
        delay(20)

def popmultitask_unittest():
    my = My()
    my.start()
    for i in range(10):
        print("terminate: %d"%(i+1))
        delay(20)
    my.stop()
    my.start()
    for i in range(9, -1, -1):
        print("terminate: %d"%(i+1))
        delay(20)
    my.stop()
##################################################################
"""
#wiringPi
    def __init__(self, n):
        self._gpio = n
        pinMode(self._gpio, OUTPUT)

    def __del__(self):
        self.off()

    def on(self):
        digitalWrite(self._gpio, HIGH)

    def off(self):
        digitalWrite(self._gpio, LOW)

#RPi.GPIO
    def __init__(self, n):
        self._gpio = n
        GPIO.setup(self._gpio, GPIO.OUT)

    def __del__(self):
        self.off()

    def on(self):
        GPIO.output(self._gpio, GPIO.HIGH)

    def off(self):
        GPIO.output(self._gpio, GPIO.LOW)
"""
#cpp binding
class Out(object):
    def __init__(self, n):
        self.bind = binder.Out_new(n)

    def __del__(self):
        self.off()

    def on(self):
        binder.Out_on(self.bind)

    def off(self):
        binder.Out_off(self.bind)

##################################################################
class Led(Out):
    def __init__(self, n):
        super().__init__(n)

    def __del__(self):
        super().__del__()
    
    def blink(self, period, second):
        for _ in range(second):
            self.on()
            delay(period)
            self.off()
            delay(period)

def led_unittest():
    leds = [Led(23), Led(24), Led(25), Led(1)]

    for _ in range(100):
        for i in range(4):
            leds[i].on()
            delay(10)
        
        for i in range(4):
            leds[i].off()
            delay(10)

class Leds(object):
    _leds = list()

    def __init__(self, debug=False):
        self._max = pinMax(LED)
        
        for i in range(self._max):
            pin = pinMap(LED, i)

            if pin is 0xFF:
                _leds = [None]
                break

            else:
                if debug:
                    print("Led Pin: %d" % pin)
                self._leds.append(Led(pin))

    def __getitem__(self, item):
        if(len(self._leds) == 0):
            raise ValueError("leds are empty")
        return self._leds[item]
    
    def allOn(self):
        for led in self._leds:
            led.on()

    def allOff(self):
        for led in self._leds:
            led.off()

##################################################################
class Laser(Out):
    def __init__(self, n):
        super().__init__(n)

    def __del(self):
        super().__del__()

def laser_unittest():
    laser = Laser(2)

    for i in range(10):
        laser.on()
        delay(100)
        laser.off()
        delay(100)
##################################################################

class Buzzer(Out):

    def __init__(self, n):
        super().__init__(n)

    def __del__(self):
        super().__del__()

def buzzer_unittest():
    buzzer = Buzzer(2)

    for i in range(10):
        buzzer.on()
        delay(50)
        buzzer.off()
        delay(50)
##################################################################

class Relay(Out):

    def __init__(self, n):
        super().__init__(n)

    def __del__(self):
        super().__del__()

def relay_unittest1():
    relay = Relay(2)

    for _ in range(10):
        relay.on()
        delay(1000)
        relay.off()
        delay(1000)

def relay_unittest2():
    relay = Relay(2)

    while (True):
        relay.on()
        input("Press <ENTER> key...\n")
        relay.off()
        input("Press <ENTER> key...\n")
##################################################################

#bindings..
class LedEx(object):
    def __init__(self, n):
        self.bind = binder.LedEx_new(n)
    
    def __del__(self):
        self.off()

    def on(self):
        binder.LedEx_on(self.bind)

    def off(self):
        binder.LedEx_off(self.bind)

    def bright(self, n):
        binder.LedEx_bright(self.bind, n)
"""
#wiRingPi
    def __init__(self, n):
        self._gpio = n
        softPwmCreate(self._gpio, 0, 100)

    def __del__(self):
        self.off()

    def on(self):
        softPwmWrite(self._gpio, 100)

    def off(self):
        softPwmWrite(self._gpio, 0)

    def bright(self, n):
        level = [0, 2, 4, 6, 10, 20, 30, 40, 50, 60, 100]

        softPwmWrite(self._gpio, level[n])
"""

def ledex_unittest1():
    led = LedEx(2)

    for i in range(10, -1, -1):
        print("bright %02d" % (i))
        led.bright(i)
        delay(1000)

def ledex_unittest2():
    leds = [LedEx(2), LedEx(3), LedEx(4), LedEx(17)]

    for _ in range(10):
        for i in range(4):
            leds[i].bright(randint(0, 10))

        delay(1000)  
##################################################################

class RGBLed:
    RED_COLOR = 1
    GREEN_COLOR = 2
    BLUE_COLOR = 4

#cpp binding
    def __init__(self, *ns):
        binder.RGBLed_new.argtype = [ctypes.c_void_p]
        array = ctypes.c_int*3
        a = array(ns[0],ns[1],ns[2])
        self.bind = binder.RGBLed_new(a)

    def __del__(self):
        self.off(0)

    def on(self,color):
        binder.RGBLed_on(self.bind,color)

    def off(self,color):
        binder.RGBLed_off(self.bind,color)

    def set(self,*colors):
        l = len(colors)
        if l == 1:
            binder.RGBLed_set_color(self.bind,colors[0])
        elif l == 3:
            binder.RGBLed_set_rgb(self.bind,colors[0],colors[1],colors[2])
"""
#wiRingPi 
    def __init__(self, *ns):
        self._gpios = ns
        for gpio in self._gpios:
            pinMode(gpio, OUTPUT)
            softPwmCreate(gpio, 0, 255)

    def __del__(self):
        for gpio in self._gpios:
            softPwmWrite(gpio, 0)

    def on(self, color):
        for i, gpio in enumerate(self._gpios):
            if ((color >> i) & 0x01):
                softPwmWrite(gpio, 255)

    def off(self, color):
        for i, gpio in enumerate(self._gpios):
            if ((color >> i) & 0x01):
                softPwmWrite(gpio, 0)

    def set(self, *colors):
        l = len(colors)

        if l == 1:
            for i, gpio in enumerate(self._gpios):
                if ((colors[0] >> i) & 0x01):
                    softPwmWrite(gpio, 255)
                else:
                    softPwmWrite(gpio, 0)
        elif l == 3:
            for i, color in enumerate(colors):
                softPwmWrite(self._gpios[i], color)
"""

def rgbled_unittest1():
    rgbled = RGBLed(2, 3, 4)

    rgbled.on(RGBLed.RED_COLOR | RGBLed.GREEN_COLOR | RGBLed.BLUE_COLOR)
    delay(1000)
    rgbled.off(RGBLed.BLUE_COLOR)
    delay(1000)
    rgbled.off(RGBLed.GREEN_COLOR)
    delay(1000)
    rgbled.off(RGBLed.RED_COLOR)
    delay(1000)
    rgbled.set(RGBLed.RED_COLOR)
    delay(1000)
    rgbled.set(RGBLed.GREEN_COLOR)
    delay(1000)
    rgbled.set(RGBLed.BLUE_COLOR)
    delay(1000)

def rgbled_unittest2():
    rgbled = RGBLed(2, 3, 4)

    while (True):
        r = randint(0, 255)
        g = randint(0, 255)
        b = randint(0, 255)
        print("R=%03d, G=%03d, B=%03d"%(r, g, b))
        rgbled.set(r, g, b)
        delay(100)


def rgbled_unittest3():
    rgbled = RGBLed(2, 3, 4)

    while (True):
        r = int(input("Enter Red(0 ~ 255): "))
        g = int(input("Enter Green(0 ~ 255): "))
        b = int(input("Enter Blue(0 ~ 255): "))
        print(">>> set color: %3d:%3d:%3d"%(r, g, b))
        rgbled.set(r, g, b)
        delay(100)

##################################################################

class DCMotor(object):
    SPEED_1 = 3
    SPEED_2 = 5
    SPEED_3 = 10
#binding 
    def __init__(self, *ns):
        binder.DCMotor_new.argtype = [ctypes.c_void_p]
        array = ctypes.c_int*2
        a = array(ns[0],ns[1])
        self.bind = binder.DCMotor_new(a)

    def __del__(self):
        self.stop()

    def forward(self):
        binder.DCMotor_forward(self.bind)
    
    def backward(self):
        binder.DCMotor_backward(self.bind)

    def setSpeed(self,n):
        binder.DCMotor_setSpeed(self.bind,n)

    def stop(self):
        binder.DCMotor_stop(self.bind)

"""
#old 
    def __init__(self, *ns):
        self._gpios = ns
        self._speed = 0

        for gpio in self._gpios:
            pinMode(gpio, OUTPUT)
            softPwmCreate(gpio, 0, 10)

    def __del__(self):
        self.stop(False)

        for gpio in self._gpios:
            digitalWrite(gpio, LOW)

    def forward(self):
        softPwmWrite(self._gpios[0], self._speed)
        softPwmWrite(self._gpios[1], 0)

    def backward(self):
        softPwmWrite(self._gpios[0], 0)
        softPwmWrite(self._gpios[1], self._speed)

    def setSpeed(self, speed):
        self._speed = speed

    def stop(self, isBreak=True):
        for gpio in self._gpios:
            if (isBreak):
                softPwmWrite(gpio, 10)
            else:
                softPwmWrite(gpio, 0)
"""

def dcmotor_unittest1():
    dcmotor = DCMotor(2, 3)
    dcmotor.setSpeed(DCMotor.SPEED_3)
    dcmotor.forward()
    delay(3000)
    dcmotor.backward()
    delay(3000)

def dcmotor_unittest2():
    dcmotor = DCMotor(2, 3)
    speeds = [DCMotor.SPEED_1, DCMotor.SPEED_2, DCMotor.SPEED_3]

    for speed in speeds:
        dcmotor.setSpeed(speed)
        dcmotor.forward()
        input("Press <ENTER> key...\n")

    dcmotor.stop()
    delay(2000)
##################################################################
class StepMotor(object):
# cpp binding
    SPEED_1 = 0
    SPEED_2 = 1
    SPEED_3 = 2

    ONE_PHASE_FULLSTEP = 1
    TWO_PHASE_FULLSTEP = 2
    HALFSTEP = 3

    SPEED_SEQ = [ 100, 40, 15 ]

    ONE_PHASE_FULLSTEP_SEQ = [
        [HIGH, LOW, LOW, LOW],
        [LOW, HIGH, LOW, LOW],
        [LOW, LOW, HIGH, LOW],
        [LOW, LOW, LOW, HIGH],
    ]


    TWO_PHASE_FULLSTEP_SEQ = [
        [HIGH, HIGH, LOW, LOW],
        [LOW, HIGH, HIGH, LOW],
        [LOW, LOW, HIGH, HIGH],
        [HIGH, LOW, LOW, HIGH],
    ]

    HALFSTEP_SEQ = [
        [HIGH, LOW, LOW, LOW], 
        [HIGH, HIGH, LOW, LOW],
        [LOW, HIGH, LOW, LOW],
        [LOW, HIGH, HIGH, LOW],
        [LOW, LOW, HIGH, LOW],
        [LOW, LOW, HIGH, HIGH],
        [LOW, LOW, LOW, HIGH],
        [HIGH, LOW, LOW, HIGH],
    ]

    def __init__(self, *ns):
        binder.StepMotor_new.argtype = [ctypes.c_void_p]
        array = ctypes.c_int*4
        a = array(ns[0],ns[1],ns[2],ns[3])
        self.bind = binder.StepMotor_new(a)

    def __del__(self):
        self.stop()

    def forward(self):
        binder.StepMotor_forward(self.bind)

    def backward(self):
        binder.StepMotor_backward(self.bind)

    def stop(self):
        binder.StepMotor_stop(self.bind)

    def setMode(self,mode):
        binder.StepMotor_setMode(self.bind,mode)

    def setSpeed(self, speed):
        binder.StepMotor_setSpeed(self.bind, speed)

"""
# wiRingPi
class StepMotor(PopThread):
    SPEED_1 = 0
    SPEED_2 = 1
    SPEED_3 = 2

    ONE_PHASE_FULLSTEP = 1
    TWO_PHASE_FULLSTEP = 2
    HALFSTEP = 3

    SPEED_SEQ = [ 100, 40, 15 ]

    ONE_PHASE_FULLSTEP_SEQ = [
        [HIGH, LOW, LOW, LOW],
        [LOW, HIGH, LOW, LOW],
        [LOW, LOW, HIGH, LOW],
        [LOW, LOW, LOW, HIGH],
    ]


    TWO_PHASE_FULLSTEP_SEQ = [
        [HIGH, HIGH, LOW, LOW],
        [LOW, HIGH, HIGH, LOW],
        [LOW, LOW, HIGH, HIGH],
        [HIGH, LOW, LOW, HIGH],
    ]

    HALFSTEP_SEQ = [
        [HIGH, LOW, LOW, LOW], 
        [HIGH, HIGH, LOW, LOW],
        [LOW, HIGH, LOW, LOW],
        [LOW, HIGH, HIGH, LOW],
        [LOW, LOW, HIGH, LOW],
        [LOW, LOW, HIGH, HIGH],
        [LOW, LOW, LOW, HIGH],
        [HIGH, LOW, LOW, HIGH],
    ]
    def __init__(self, *ns):
        self._gpios = ns
        self._speed = 0
        self._mode = None
        self._pos = 0
        self._forward = True
        self._run = False
       
        for gpio in self._gpios:
            pinMode(gpio, OUTPUT)

        self.setMode(StepMotor.HALFSTEP)

    def __del__(self):
        self.stop()

    def forward(self):
        self._forward = True

        if (not self._run):
            self._run = True
            self.start()

    def backward(self):
        self._forward = False

        if (not self._run):
            self._run = True
            self.start()

    def setSpeed(self, n):
        if (self._mode == StepMotor.HALFSTEP and (n == StepMotor.SPEED_2 or n == StepMotor.SPEED_3)):
            self._speed = StepMotor.SPEED_SEQ[n] // 5
        else:
            self._speed = StepMotor.SPEED_SEQ[n]

    def stop(self):
        if (self._run):
            super().stop()
            self._run = False

        for gpio in self._gpios:
            digitalWrite(gpio, LOW)

    def setMode(self, mode):
        self._mode = mode

        if (self._mode == StepMotor.ONE_PHASE_FULLSTEP or self._mode == StepMotor.TWO_PHASE_FULLSTEP):
            self._step = 4
        else:
            self._step = 8

    def run(self):
        for i in range(4):
            if (self._mode == StepMotor.ONE_PHASE_FULLSTEP):
                digitalWrite(self._gpios[i], StepMotor.ONE_PHASE_FULLSTEP_SEQ[self._pos][i])
            elif (self._mode == StepMotor.TWO_PHASE_FULLSTEP):
                digitalWrite(self._gpios[i], StepMotor.TWO_PHASE_FULLSTEP_SEQ[self._pos][i])
            else:
                digitalWrite(self._gpios[i], StepMotor.HALFSTEP_SEQ[self._pos][i])
        delay(self._speed)

        if (self._forward):
            self._pos += 1
            if (self._pos >= self._step):
                self._pos = 0
        else:
            self._pos -= 1
            if (self._pos < 0):
                self._pos = self._step - 1
"""

def stepmotor_unittest1():
    stepmotor = StepMotor(2, 3, 4, 17)
    stepmotor.setSpeed(StepMotor.SPEED_3)
    stepmotor.setMode(StepMotor.HALFSTEP)

    stepmotor.backward()
    delay(3000)
    stepmotor.stop()
    delay(2000)
    stepmotor.forward()
    delay(3000)

def stepmotor_unittest2():
    stepmotor = StepMotor(2, 3, 4, 17)
    for s in [StepMotor.SPEED_1, StepMotor.SPEED_2, StepMotor.SPEED_3]:
        stepmotor.setSpeed(s)
        stepmotor.forward()
        delay(3000)
        stepmotor.backward()
        delay(3000)

def stepmotor_unittest3():
    stepmotor = StepMotor(2, 3, 4, 17)
    
    for m in [StepMotor.ONE_PHASE_FULLSTEP, StepMotor.TWO_PHASE_FULLSTEP, StepMotor.HALFSTEP]:
        for s in [StepMotor.SPEED_1, StepMotor.SPEED_2, StepMotor.SPEED_3]:
            print("speed = %d, mode = %d"%(s, m))
            stepmotor.setSpeed(s)
            stepmotor.setMode(m)
            stepmotor.forward()
            delay(3000)
            stepmotor.backward()
            delay(3000)

class Input:
    FALLING = GPIO.FALLING
    RISING = GPIO.RISING
    BOTH = GPIO.BOTH

    def __init__(self, n, activeHigh=True):
        self._gpio = n
        self._func = None
        self._param = None
        self._activeHigh = activeHigh

        GPIO.setup(self._gpio, GPIO.IN)

    def read(self):
        level = GPIO.input(self._gpio)
        return level if self._activeHigh else not level

    def setCallback(self, func, param=None, interrupt_type=BOTH):
        __func = None
        __params = list(inspect.signature(func).parameters.keys())
        def wrap_callback_function(__gpio_number):
            if len(__params) == 0:
                func()
            elif param is not None:
                func(param)
            else :
                func(self.read())
        __func = wrap_callback_function
        __func.callback_function = func
                
        self._func = __func
        self._param = param
        return GPIO.add_event_detect(self._gpio, interrupt_type, callback=self._func if self._func != None else self._dummy)

    def _wrapper(self):
        self._func(self._param)

    def _dummy(self):
        pass
##################################################################

class Switch(Input):
    def __init__(self, n, activeHigh=False):
        super().__init__(n, activeHigh)

def switch_unittest1():
    sw = Switch(2)

    while (True):
        print("%d"%(sw.read()))
        delay(100)

def switch_unittest2():
    sw = Switch(2)

    old = False

    while (True):
        cur = sw.read()

        if (cur != old):
            if (cur):
                print("press")
            else:
                print("relese")
            old = cur

        delay(100)

def onSwitch(sw):
    if (sw.read()):
        print("press")
    else:
        print("release")

    delay(100)

def switch_unittest3():
    sw = Switch(2)
    sw.setCallback(onSwitch, sw)
    input("Press <ENTER> key...\n")

class Switches(object):
    _switches = list()

    def __init__(self, activeHigh=False, debug=False):
        self._max = pinMax(SWITCH)
        #print(self._max)
        
        for i in range(self._max):
            pin = pinMap(SWITCH, i)

            if pin is 0xFF:
                _switches = [None]
                break;
            
            else:
                if debug:
                    print("Switch Pin: %d" % pin)
                self._switches.append(Switch(pin,activeHigh))

    def __getitem__(self, item):
        if(len(self._switches) == 0):
            raise ValueError("switches are empty")
        return self._switches[item]

##################################################################

class Reed(Input):
    def __init__(self, n):
        super().__init__(n)

def reed_unittest1():
    reed = Reed(2)

    while (True):
        print("%d"%(reed.read()))
        delay(100)

def reed_unittest2():
    reed = Reed(2)
    old = False

    while (True):
        cur = reed.read()

        if (cur != old):
            print("%s"%("press" if (cur) else "relese"))
            old = cur

        delay(100)

def onReed(reed):
    print("%s"%("press" if reed.read() else "relese"))
    delay(100)

def reed_unittest3():
    reed = Reed(2)
    reed.setCallback(onReed, reed)
    input("Press <ENTER> key...\n")
##################################################################

class LimitSwitch(Input):
    def __init__(self, n):
        super().__init__(n)

def limitswitch_unittest1():
    limitswitch = LimitSwitch(2)

    while (True):
        print("%d"%(limitswitch.read()))
        delay(50)

def limitswitch_unittest2():
    limitswitch = LimitSwitch(2)
    old = False
    count = 0

    while (True):
        cur = limitswitch.read()

        if (cur != old):
            if (cur):
                count += 1
                print("count = %d"%(count))
            old = cur

        delay(50)

def onLimitSwitch(unuse):
    onLimitSwitch.flags += 1

    if (onLimitSwitch.flags % 3 == 0):
        onLimitSwitch.count += 1
        print("count = %d"%(onLimitSwitch.count))

    delay(50)

onLimitSwitch.count = 0
onLimitSwitch.flags = 0

def limitswitch_unittest3():
    limitswitch = LimitSwitch(2)
    limitswitch.setCallback(onLimitSwitch, None, LimitSwitch.RISING)
    input("Press <ENTER> key...\n")
##################################################################

class Mercury(Input):
    def __init__(self, n):
        super().__init__(n, False)

def mercury_unittest1():
    mercury = Mercury(2)

    while (True):
        print("%d"%(mercury.read()))
        delay(50)

def mercury_unittest2():
    mercury = Mercury(2)
    old = False

    print("init stat = %s"%("on" if mercury.read() else "off"))

    while (True):
        cur = mercury.read()

        if (cur != old):
            print("%s"%("on" if (cur) else "off"))
            old = cur

        delay(100)

def onMercury(mercury):
    cur = mercury.read()

    if (onMercury.old != cur):
        print("%s"%("on" if cur else "off"))
        onMercury.old = cur

    delay(100)

onMercury.old = False

def mercury_unittest3():
    mercury = Mercury(2)
    
    print("init stat = %s"%("on" if mercury.read() else "off"))

    mercury.setCallback(onMercury, mercury)
    input("Press <ENTER> key...\n")
##################################################################

class Knock(Input):
    def __init__(self, n):
        super().__init__(n, False)

def knock_unittest1():
    knock = Knock(2)

    while (True):
        print("%d"%(knock.read()))
        delay(20)

def knock_unittest2():
    knock = Knock(2)
    count = 0

    while (True):
        for _ in range(100000):
            if (knock.read()):
                count += 1
                print("knock = %d"%(count))
                break

        delay(20)

def onKnock(unuse):
    onKnock.count += 1
    print("knock = %d"%(onKnock.count))
    delay(100)

onKnock.count = 0

def knock_unittest3():
    knock = Knock(2)
    
    knock.setCallback(onKnock, None, Knock.FALLING)
    input("Press <ENTER> key...\n")
##################################################################

class Tilt(Input):
    def __init__(self, n):
        super().__init__(n, False)

def tilt_unittest1():
    tilt = Tilt(2)

    while (True):
        print("%d"%(tilt.read()))
        delay(50)

def tilt_unittest2():
    tilt = Tilt(2)

    while (True):
        print("%s"%("untitled" if (tilt.read()) else "titled"))
        delay(50)

def onTilt(tilt):
    print("%s"%("untitled" if tilt.read() else "titled"))
    delay(50)

def tilt_unittest3():
    tilt = Tilt(2)
    tilt.setCallback(onTilt, tilt)
    input("press any key...\n")
##################################################################
'''
class Shock(SpiAdc):
    def __init__(self, channel=-1, device=0, bus=0, speed=1000000):
        if channel < 0:
            channel = channelMap(5)

        super().__init__(channel, device, bus, speed)
'''
def shock_unittest1():
    shock = Shock(2)

    while (True):
        print("%d"%(shock.read()))
        delay(50)

def shock_unittest2():
    shock = Shock(2)

    while (True):
        if (shock.read()):
            print("shocked")

            cnt = 100
            while (cnt):
                if (not shock.read()):
                    cnt -= 1
                delay(20)

            print("unshocked")

        delay(50)

def onShock(shock):
    cnt = 100

    onShock.flags += 1
    if (onShock.flags % 2):
        return

    print("shocked")

    while (cnt):
        if (not shock.read()):
            cnt -= 1
        delay(20)

    print("unshocked")

    delay(50)

onShock.flags = 1

def shock_unittest3():
    shock = Shock(2)
    shock.setCallback(onShock, shock, Shock.RISING)
    input("press any key...\n")
##################################################################

class Opto(Input):
    def __init__(self, n):
        super().__init__(n)

def opto_unittest1():
    opto = Opto(2)

    while (True):
        print("%d"%(opto.read()))
        delay(50)

def opto_unittest2():
    opto = Opto(2)
    old = False
    count = 0

    while (True):
        cur = opto.read()

        if (cur != old):
            old = cur
            if (cur) :
                count += 1
                print("opto = %d"%(count))

        delay(50)

def onOpto(unuse):
    onOpto.count += 1
    print("opto = %d"%(onOpto.count))

    delay(50)

onOpto.count = 0

def opto_unittest3():
    opto = Opto(2)
    opto.setCallback(onOpto, None, Opto.FALLING)
    input("Press <ENTER> key...\n")
##################################################################

class Pir(Input):
    def __init__(self, n=-1):
        if n < 0:
            n = pinMap(PIR)

        super().__init__(n)

def pir_unittest1():
    pir = Pir(2)

    while (True):
        print("%d"%(pir.read()))
        delay(50)

def pir_unittest2():
    pir = Pir(2)
    count = 0

    while (True):
        if (pir.read()):
            count += 1
            print("pir = %d"%(count))
            delay(1000)
        else:
            delay(50)

def onPir(unuse):
    onPir.count += 1
    print("pir = %d"%(onPir.count))
    delay(1000)

onPir.count = 0

def pir_unittest3():
    pir = Pir(2)
    pir.setCallback(onPir, None, Pir.RISING)
    input("Press <ENTER> key...\n")
##################################################################

class Flame(Input):
    def __init__(self, n):
        super().__init__(n, False)

def flame_unittest1():
    flame = Flame(2)

    while (True):
        print("%d"%(flame.read()))
        delay(50)

def flame_unittest2():
    flame = Flame(2)
    count = 0

    while (True):
        if (flame.read()):
            count += 1
            print("flame = %d"%(count))
            delay(1000)
        else:
            delay(50)

def onFlame(unuse):
    onFlame.count += 1
    print("flame = %d"%(onFlame.count))
    delay(1000)

onFlame.count = 0

def flame_unittest3():
    flame = Flame(2)
    flame.setCallback(onPir, None, Flame.RISING)
    input("Press <ENTER> key...\n")
##################################################################

class LineTrace(Input):
    def __init__(self, n):
        super().__init__(n)

def linetrace_unittest1():
    linetrace = LineTrace(2)

    while (True):
        print("%d"%(linetrace.read()))
        delay(50)

def linetrace_unittest2():
    linetrace = LineTrace(2)

    while (True):
        print("%s"%("line" if linetrace.read() else "ground"))
        delay(50)

def onLineTrace(linetrace):
    print("%s"%("line" if linetrace.read() else "ground"))
    delay(50)

def linetrace_unittest3():
    linetrace = LineTrace(2)
    linetrace.setCallback(onLineTrace, linetrace)
    input("Press <ENTER> key...\n")

##################################################################
#Ultrasonic
def partition(arr,low,high): 
    i = ( low-1 )
    pivot = arr[high]
  
    for j in range(low , high):   
        if   arr[j] <= pivot:
            i = i+1 
            arr[i],arr[j] = arr[j],arr[i] 
  
    arr[i+1],arr[high] = arr[high],arr[i+1] 
    return ( i+1 ) 

def quickSort(arr,low,high): 
    if low < high:   
        pi = partition(arr,low,high) 
  
        quickSort(arr, low, pi-1) 
        quickSort(arr, pi+1, high) 


global epoch_milli
global epoch_micro
def initialize_epoch():
    global epoch_milli
    global epoch_micro
    initialized_time = time.clock_gettime(time.CLOCK_MONOTONIC)
    epoch_micro = int(1000000 * initialized_time)
    epoch_milli = int(1000 * initialized_time)

def micros():
    return time.time()*1e+6

class UltraSonic(PopThread):
    def __init__(self, trig, echo):
        self._gpioTrig = trig
        self._gpioEcho = echo
        self._func = None
        self._param = None
        self._sample = None
        self._weight = None

        # pinMode(self._gpioTrig, OUTPUT)
        # pinMode(self._gpioEcho, INPUT)
        
        # digitalWrite(self._gpioTrig, LOW)

        GPIO.setup(self._gpioTrig, GPIO.OUT)
        GPIO.setup(self._gpioEcho, GPIO.IN)

        GPIO.output(self._gpioTrig, GPIO.LOW)

        delay(500)

    def __del__(self):
        if (self._func != None):
            self.stop()

    def read(self, cal=1, timeout=30000):
        delay(1)
        self._setTrig()

        now = micros()
        # while (digitalRead(self._gpioEcho) == LOW and micros() - now < timeout):
        while (GPIO.input(self._gpioEcho) == GPIO.LOW and micros() - now < timeout):
            pass

        start = micros()
        # while (digitalRead(self._gpioEcho) == HIGH):
        while (GPIO.input(self._gpioEcho) == GPIO.HIGH):
            pass

        return (34029 * (micros() - start) // 2000000) + cal
    
    def readAverage(self, sample=6, weight=0.12):
        dist = 0
        buf = [0 for _ in range(sample)]

        for i in range(sample):
            t = self.read()
            dist += t
            buf[i] = t
            delay(30)

        dist = dist // sample
        
        quickSort(buf, 0, sample-1)
        
        s = 0
        e = sample // 2

        for i in range(e):
            if (buf[i] >= (buf[e] - (buf[e] * weight))):
                s = i
                break

        for i in range(sample):
            if (buf[i] >= (buf[e] + (buf[e] * weight))):
                e = i
                break

        for i in range(s, e):
            dist += buf[i]

        return (dist // (e - s + 1))

    def setCallback(self, func, param=None, sample=20, weight=0.12):
        if (self._func != None and func == None):
            self.stop()
            self._func = None

        if (func != None):
            self._func = func
            _param = param
            _sample = sample
            _weight = weight

            self.start()

    def run(self):
        while (self.isRun()):
            dist = self.read()
            self._func(dist, self._param)

    def _setTrig(self):
        # digitalWrite(self._gpioTrig, HIGH)
        # delayMicroseconds(10)
        # digitalWrite(self._gpioTrig, LOW)
        GPIO.output(self._gpioTrig, GPIO.HIGH)
        delayMicroseconds(10)
        GPIO.output(self._gpioTrig, GPIO.LOW)

def ultrasonic_unittest1():
    usonic = UltraSonic(2, 3)

    while (True):
        dist = usonic.read()
        print("dist = %dcm"%(dist))
        delay(100)

def ultrasonic_unittest2():
    usonic = UltraSonic(2, 3)

    while (True):
        dist = usonic.readAverage()
        print("dist = %dcm"%(dist))
        delay(100)

def onUltraSonic(dist, unuse):
    print("dist = %dcm"%(dist))
    delay(100)

def ultrasonic_unittest3():
    usonic = UltraSonic(2, 3)
    usonic.setCallback(onUltraSonic)
    input("Press <ENTER> key...\n")
    usonic.stop()
##################################################################

class SpiAdc(PopThread):
    MODE_INCLUSIVE = 1
    MODE_EXCLUSIVE = 2
    MODE_FULL = 3

    TYPE_NORMAL = 1
    TYPE_AVERAGE = 2

    REF_VOLTAG = 3.3
    ADC_MAX = 4096 - 1

    def __init__(self, channel, device=0, bus=0, speed=1000000):
        self._channel = channel
        self._device = device
        self._bus = bus
        self._spi = spidev.SpiDev()
        self._spi.open(0,0)
        self._spi.max_speed_hz = speed
        self._func = None
        self._param = None
        self._sample = 1
        self._type = None
        self._mode = None
        self._min = None
        self._max = None

    def setCallback(self, func, param=None, type=TYPE_AVERAGE, mode=MODE_FULL, min=0, max=ADC_MAX):
        self._func = func
        self._param = param
        self._type = type
        self._mode = mode
        self._min = min
        self._max = max

        self.start() if (self._func != None) else self.stop()

    def setSample(self, sample):
        self._sample = sample

    def getSample(self):
        return self._sample

    def read(self):
        """
        #tx0 = 0, 0, 0, 0, 0, start(1), single(1), channel msb D2(bit)
        #tx1 = channel D1(bit), channel lsb D0(bit), 0, 0, 0, 0, 0, 0
        #tx2 = 0, 0, 0, 0, 0, 0, 0, 0
        
        tx1 = (0x06 | ((self._channel & 0x07) >> 2))
        tx2 = ((self._channel & 0x07) << 6) & 0xFF
        tx3 = 0x00
        buf = bytes([tx1, tx2, tx3])

        if (self._cs != None):
            digitalWrite(self._cs, LOW)
        
        _, data = wiringPiSPIDataRW(self._device, buf)
        
        #rx0 = x, x, x, x, x, x, x, x
        #rx1 = x, x, x, 0, D11, D10, D9, D8
        #rx2 = D7, D6, D5, D4, D3, D2, D1, D0
        
        if (self._cs != None):
            digitalWrite(self._cs, HIGH)
       
        return (((data[1] & 0x0F) << 8) | (data[2] & 0xFF)) & 0xFFF
        """
        
        r = self._spi.xfer2([6|(self._channel>>2), (self._channel & 3) << 6, 0])
        adcval = ((r[1] & 15) << 8) + r[2]
        return adcval

    def readAverage(self):
        val = 0.0

        for _ in range(self._sample):
            val += math.pow(self.read(), 2)

        return int(math.sqrt(val / self._sample))

    def readVolt(self, ref=3.3, max=ADC_MAX):
        return ref * (SpiAdc.read(self) / max)

    def readVoltAverage(self, ref=3.3, max=ADC_MAX):
        return ref * (SpiAdc.readAverage(self) / max)

    def run(self):
        val = self.read() if (self._type == SpiAdc.TYPE_NORMAL) else self.readAverage()

        if (self._mode == SpiAdc.MODE_INCLUSIVE):
            if (val >= self._min and val <= self._max):
                if self._param:
                    self._func(self._param)
                else:
                    self._func(val)
        elif (self._mode == SpiAdc.MODE_EXCLUSIVE):
            if (val < self._min or val > self._max):
                if self._param:
                    self._func(self._param)
                else:
                    self._func(val)
        else:
            # self._func(val, self._param)
            if self._param:
                self._func(self._param)
            else:
                self._func(val)

def spiadc_unittest1():
    adc = SpiAdc(6)

    while (True):
        val = adc.read()
        print("%d"%(val))
        delay(20)

def spiadc_unittest2():
    adc = SpiAdc(6)

    adc.setSample(512)

    while (True):
        val = adc.readAverage()
        print("%d"%(val))
        delay(20)

def onAdc(val, unuse):
    print("%d"%(val))

def spiadc_unittest3():
    adc = SpiAdc(6)

    adc.setSample(1024)    
    adc.setCallback(onAdc)
    input("Press <ENTER> key...\n")

def spiadc_unittest4():
    adc = SpiAdc(6)

    adc.setCallback(onAdc, type=SpiAdc.TYPE_NORMAL)
    input("Press <ENTER> key...\n")


def spiadc_unittest5():
    adc = SpiAdc(6)

    adc.setSample(1024)
    adc.setCallback(onAdc, type=SpiAdc.TYPE_AVERAGE, mode=SpiAdc.MODE_EXCLUSIVE, min=100, max=1000)
    input("Press <ENTER> key...\n")
    adc.stop()

    adc.setCallback(onAdc, type=SpiAdc.TYPE_NORMAL, mode=SpiAdc.MODE_INCLUSIVE, min=100, max=1000)
    input("Press <ENTER> key...\n")
##################################################################

class Shock(SpiAdc):
    def __init__(self, channel, device=0, bus=0, speed=1000000):
        super().__init__(channel, device, bus, speed)
        self.setSample(512)



def shock2_unittest1():
    shock2 = Shock2(6)

    while (True):
        print("%d"%(shock2.read()))
        delay(50)

def shock2_unittest2():
    shock2 = Shock2(6)

    while (True):
        print("%d"%(shock2.readAverage()))
        delay(50)

def onShock2(val, unuse):
    print("%d"%(val))
    delay(100)

def shock2_unittest3():
    shock2 = Shock2(6)
    shock2.setCallback(onShock2, type=SpiAdc.TYPE_AVERAGE, mode=SpiAdc.MODE_INCLUSIVE, min=1000)
    input("Press any key...\n")
    shock2.stop()
##################################################################

class Sound(SpiAdc):
    def __init__(self, channel=-1, device=0, bus=0, speed=1000000):
        if channel < 0:
            channel = channelMap(SOUND)
        
        super().__init__(channel, device, bus, speed)
        self.setSample(64)

def sound_unittest1():
    sound = Sound(6)

    while (True):
        print("%d"%(sound.read()))
        delay(20)

def sound_unittest2():
    sound = Sound(6)
    max = 0

    while (True):
        val = sound.readAverage()
        if (val > max):
            max = val
            print("MAX = %d"%(val))
            delay(2000)
            continue
        
        print("%d"%(val))
        delay(20)

def onSound(val, unuse):
    print("%d"%(val))
    delay(100)

def sound_unittest3():
    sound = Sound(6)
    sound.setCallback(onSound, type=SpiAdc.TYPE_AVERAGE, mode=SpiAdc.MODE_EXCLUSIVE, min=550)
    input("Press <ENTER> key...\n")
    sound.stop()

    sound.setCallback(onSound, type=SpiAdc.TYPE_AVERAGE, mode=SpiAdc.MODE_INCLUSIVE, min=550)
    input("Press <ENTER> key...\n")
##################################################################

class Potentiometer(SpiAdc):
    def __init__(self, channel=-1, device=0, bus=0, speed=1000000):
        if channel < 0:
            channel = channelMap(POTENTIOMETER)
        
        super().__init__(channel, device, bus, speed)
        self._len = 10
        self._rangeTable = [0, 0, 48, 300, 700, 1090, 1540, 1945, 2320, 2715, 2980, 3040, SpiAdc.ADC_MAX]

        self.setSample(1024)

    def setRangeTable(self, table):
        self._len = len(table)
        self._rangeTable[2:-1] = table

    def getRangeTable(self):
        return self._rangeTable[2:-1]

    def readAverage(self):
        val = super().readAverage()
        pos = 1

        for i in range(self._len + 1):
            x1 = (self._rangeTable[i+pos] + self._rangeTable[(i-1)+pos]) // 2
            x2 = (self._rangeTable[(i+1)+pos] + self._rangeTable[i+pos]) // 2

            if (val >= x1 and val < x2):
                val = i
                break

        if (i == self._len + 1):
            val = self._len

        return val

def potentiometer_unittest1():
    poten = Potentiometer(6)

    while (True):
        val = poten.read()
        print("%d"%(val))
        delay(100)

def potentiometer_unittest2():
    poten = Potentiometer(6)
    max = 0

    poten.setSample(1024)

    for i in range(10):
        print("set potentiometer to %d"%(i))
        input("Press <ENTER> key...\n")

        val = SpiAdc.readAverage(poten)

        if (val > max):
            max = val
            print("%d"%(val))
                   
def potentiometer_unittest3():
    poten = Potentiometer(6)

    old = 0

    while (True):
        val = poten.readAverage()

        if (val != old):
            old = val
            print("%d"%(val))

def onPotentiometer(val, unuse):
    if (val != onPotentiometer.old):
        onPotentiometer.old = val
        print("%d"%(val))

    delay(100)

onPotentiometer.old = 0

def potentiometer_unittest4():
    poten = Potentiometer(6)

    poten.setCallback(onPotentiometer)
    input("Press <ENTER> key...\n")

def potentiometer_unittest5():
    poten = Potentiometer(6)

    poten.setCallback(onPotentiometer, type=SpiAdc.TYPE_AVERAGE, mode=SpiAdc.MODE_INCLUSIVE, min=3, max=8)
    input("Press <ENTER> key...\n")
##################################################################

class Cds(SpiAdc):
    def __init__(self, channel=-1, device=0, bus=0, speed=1000000):
        if channel < 0:
            channel = channelMap(CDS)
        
        super().__init__(channel, device, bus, speed)
        self._funcPseudoLx = None
        self.setSample(1024)

    def setCalibrationPseudoLx(self, func):
        self._funcPseudoLx = func

    def readAverage(self):
        return self._calcPseudoLx(self.readVoltAverage())

    def _calcPseudoLx(self, volt):
        r = (10 * 33) / volt - 10
        lx = (129.15 * math.pow(r, -10.0 / 9)) * 1000

        if (self._funcPseudoLx != None):
            val = self._funcPseudoLx(volt, r, lx)
        else:
            if (lx >= 100 and lx < 190):
                lx *= 2
            elif (lx >= 190):
                lx *= 2.4

            val = int(math.floor(lx + 0.5))

        return val

def cds_unittest1():
    cds = Cds(6)

    while (True):
        val = cds.read()
        print("%d"%(val))
        delay(20)

def cds_unittest2():
    cds = Cds(6)

    while (True):
        val = SpiAdc.readAverage(cds)
        print("%d"%(val))
        delay(500)
                   
def onCds(val, unuse):
    print("%d"%(val))

def cds_unittest3():
    cds = Cds(6)

    cds.setCallback(onCds, type=SpiAdc.TYPE_AVERAGE, mode=SpiAdc.MODE_EXCLUSIVE, min=100, max=340)
    input("Press <ENTER> key...\n")

def myPseudoLx(mVolt, r, lx):
    print("<mVolt: %.3f, r: %.3f, lx: %.3f"%(mVolt, r, lx))
    if (lx >= 100 and lx < 190):
        lx *= 2
    elif (lx >= 190):
        lx *= 2.4

    delay(500)

    return int(math.floor(lx+0.5))

def cds_unittest4():
    cds = Cds(6)

    cds.setCalibrationPseudoLx(myPseudoLx)
    cds.setCallback(onCds)
    input("Press <ENTER> key...\n")

##################################################################
class Vr(SpiAdc):
    def __init__(self, channel=-1, device=0, bus=0, speed=1000000):
        if channel < 0:
            channel = channelMap(POTENTIOMETER)
        
        super().__init__(channel, device, bus, speed)
        self._funcPseudoLx = None
        self.setSample(1024)

def vr_unittest():
    vr = Vr(0)

    while (True):
        val = vr.read()
        print("%d"%(val))
        delay(20)

##################################################################

class SoilMoisture(SpiAdc):
    def __init__(self, channel, device=0, bus=0, speed=1000000):
        super().__init__(channel, device, bus, speed)
        self._minSM = 1
        self._maxSM = 2453

        self.setSample(1024)

    def setMoistureRange(self, min, max):
        self._minSM = min
        self._maxSM = max

    def readAverage(self):
        val = super().readAverage()

        return map(val, self._minSM, self._maxSM, 0, 100)

def soilmoisture_unittest1():
    sm = SoilMoisture(6)

    print("measre min")
    val = 0
    for i in range(10):
        val += sm.read()
    print("min: %d"%(val//i))
    input("Press <ENTER> key...\n")

    print("measre min")
    val = 0
    for i in range(10):
        val += sm.read()
    print("min: %d"%(val//i))
    
def soilmoisture_unittest2():
    sm = SoilMoisture(6)

    sm.setMoistureRange(1, 2453)

    """
    80% <= : 완전히 메마른 상태
    89% < : 적절한 상태
    93% >= : 물 많음
    96% >= : 침수(물 속)
    """

    while (True):
        val = sm.readAverage()
        print("%d"%(val))
        delay(1000)

def onSoilMoisture(val, unuse):
    print("%d"%(val))
    delay(1000)

def soilmoisture_unittest3():
    sm = SoilMoisture(6)

    sm.setCallback(onSoilMoisture)
    input("Press <ENTER> key...\n")
##################################################################

class Thermistor(SpiAdc):
    def __init__(self, channel, device=0, bus=0, speed=1000000):
        super().__init__(channel, device, bus, speed)
        self._r = None
        self._t0 = None
        self._r0 = None
        self._b = None

        self.setSample(128)

        self.setCoefficient(10000.0, 25.0, 10000.0, 4200.0)

    def setCoefficient(self, r, t0, r0, b):
        self._r = r
        self._t0 = t0
        self._r0 = r0
        self._b = b

    def getCoefficient(self):
        return (self._r, self._t0, self._r0, self._b)

    def calcTempC(self, val, cal=650): 
        rV = self._r * ((SpiAdc.ADC_MAX - cal) / val - 1.0)
        tempK = 1.0 / (1.0 / (self._t0 + 273.15) + (1 / self._b) * math.log((rV / self._r0)))

        return tempK - 273.15

def thermistor_unittest1():
    thermi = Thermistor(6)

    while (True):
        val = thermi.read()
        print("%d"%(val))
        delay(100)

def thermistor_unittest2():
    thermi = Thermistor(6)
    
    thermi.setCoefficient(10000.0, 25.0, 10000.0, 4200.0)

    while (True):
        val = thermi.read()
        temp = thermi.calcTempC(val)
        print("%.2f"%(temp))
        delay(100)    

def thermistor_unittest3():
    thermi = Thermistor(6)

    while (True):
        val = thermi.readAverage()
        temp = thermi.calcTempC(val)
        print("%.2f"%(temp))
        delay(100)

def onThermistor(val, thermi):
    temp = thermi.calcTempC(val)
    print("%.2f"%(temp))
    delay(100)

def thermistor_unittest4():
    thermi = Thermistor(6)
    thermi.setCallback(onThermistor, thermi)
    input("Press <ENTER> key...\n")
##################################################################

class Temperature(SpiAdc):
    def __init__(self, channel, device=0, bus=0, speed=1000000):
        super().__init__(channel, device, bus, speed)
        self.setSample(512)

    def calcTempC(self, val, cal=1390):
        return val * SpiAdc.REF_VOLTAG * 100.0 / (SpiAdc.ADC_MAX + cal)

def temperature_unittest1():
    temp = Temperature(6)

    while (True):
        val = temp.read()
        print("%d"%(val))
        delay(1000)

def temperature_unittest2():
    temp = Temperature(6)

    while (True):
        val = temp.readAverage()
        t = temp.calcTempC(val)

        print("%.2f"%(t))
        delay(1000)

def onTemperature(val, temp):
    t = temp.calcTempC(val)
    print("%.2f"%(t))
    delay(1000)

def temperature_unittest3():
    temp = Temperature(6)

    temp.setCallback(onTemperature, temp)
    input("Press <ENTER> key...\n")    
##################################################################

class Gas(SpiAdc):
    def __init__(self, channel=-1, device=0, bus=0, speed=1000000):
        super().__init__(channel, device, bus, speed)       
        self.setSample(64)

        self._propan = [2.48, -0.64, -0.71]
        self._methan = [2.48, -0.50, -0.56]
        self._ethanol = [2.48, -0.35, -0.49]

        self._r0 = self.calibration()

    def calibration(self, rl=4.7, clean=1):
        val = 0.0

        for _ in range(32):
            val += self.resistanceCalculation(self.read(), rl)
            delay(50)

        val /= 32

        return val / clean

    def setPropanCurve(self, x, y, inclination):
        self._propan = [x, y, inclination]

    def setMethanCurve(self, x, y, inclination):
        self._methan = [x, y, inclination]

    def setEthanolCurve(self, x, y, inclination):
        self._ethanol = [x, y, inclination]

    def calcPropan(self, val):
        ppm = self.resistanceCalculation(val) / self._r0

        return math.pow(10, (((math.log(ppm) - self._propan[1]) / self._propan[2]) + self._propan[0]))

    def calcMethan(self, val):
        ppm = self.resistanceCalculation(val) / self._r0

        return math.pow(10, (((math.log(ppm) - self._methan[1]) / self._methan[2]) + self._methan[0]))

    def calcEthanol(self, val):
        ppm = self.resistanceCalculation(val) / self._r0

        return math.pow(10, (((math.log(ppm) - self._ethanol[1]) / self._ethanol[2]) + self._ethanol[0]))

    def resistanceCalculation(self, val, rl=4.7):
        return (rl * (SpiAdc.ADC_MAX - val)) / val

def gas_unittest1():
    gas = Gas(6)

    while (True):
        val = gas.read()
        print("%d"%(val))
        delay(1000)

def gas_unittest2():
    gas = Gas(6)

    gas.setPropanCurve(2.48, -0.64, -0.71)
    gas.setMethanCurve(2.48, -0.50, -0.56)
    gas.setEthanolCurve(2.48, -0.35, -0.49)

    while (True):
        val = gas.read()
        propan = gas.calcPropan(val)
        methan = gas.calcMethan(val)
        ethanol = gas.calcEthanol(val)

        print("Propan:%.2fppm, Metan:%.2fppm, Ethanol:%.2fppm"%(propan, methan, ethanol))
        delay(1000)

def gas_unittest3():
    gas = Gas(6)

    while (True):
        val = gas.readAverage()
        propan = gas.calcPropan(val)
        methan = gas.calcMethan(val)
        ethanol = gas.calcEthanol(val)

        print("Propan:%.2fppm, Metan:%.2fppm, Ethanol:%.2fppm"%(propan, methan, ethanol))
        delay(1000)

def onGas(val, gas):
    propan = gas.calcPropan(val)
    print("%.2fppm"%(propan))
    delay(1000)

def gas_unittest4():
    gas = Gas(6)
    gas.setCallback(onGas, gas)
    input("Press <ENTER> key...\n")    

##################################################################
#
class Psd(SpiAdc):
    def __init__(self, channel=-1, device=0, bus=0, speed=1000000):
        if channel < 0:
            channel = channelMap(PSD)
            
        super().__init__(channel, device, bus, speed)
        self.setSample(32)

    def calcDist(self, val, calibration = 1.1):
        volt = SpiAdc.REF_VOLTAG * (val / SpiAdc.ADC_MAX)
        return 27.86 * math.pow(volt, -1.15) * calibration

def psd_unittest1():
    psd = Psd()
    while (True):
        val = psd.read()
        print("%d"%(val))
        delay(50)

def psd_unittest2():
    psd = Psd()
    while (True):
        val = psd.readAverage()
        print("%d"%(val))
        delay(50)

def psd_unittest3():
    psd = Psd()
    while (True):
        val = psd.readAverage()
        print("%dcm"%(psd.calcDist(val)))
        delay(50)

def onPsd(val, psd):
    print("%dcm"%(psd.calcDist(val)))
    delay(50)

def psd_unittest4():
    psd = Psd()
    psd.setCallback(onPsd, psd)
    input("Press <ENTER> key...\n")    

##################################################################
# ShiftRegister (IC = 74HC595)
class ShiftRegister(object):
    _fnd_map = [
        [0,1,1,1,1,1,1,0], #0
        [0,0,0,0,1,1,0,0], #1
        [1,0,1,1,0,1,1,0], #2
        [1,0,0,1,1,1,1,0], #3
        [1,1,0,0,1,1,0,0], #4
        [1,1,0,1,1,0,1,0], #5
        [1,1,1,1,1,0,1,0], #6
        [0,1,0,0,1,1,1,0], #7
        [1,1,1,1,1,1,1,0], #8
        [1,1,0,1,1,1,1,0]  #9
    ]
#binding
    def __init__(self,*ns):
        binder.ShiftRegister_new.argtype = [ctypes.c_void_p]
        array = ctypes.c_int*3

        if len(ns) < 3:
            pin0 = pinMap(SHIFT_REGISTER, 0)
            pin1 = pinMap(SHIFT_REGISTER, 1)
            pin2 = pinMap(SHIFT_REGISTER, 2)
            a = array(pin0, pin1, pin2)
        else:
            a = array(ns[0],ns[1],ns[2])
        self.bind = binder.ShiftRegister_new(a)

    def shiftout(self,val):
        binder.ShiftRegister_shiftout(self.bind,val)

    def fnd(self,val):
        binder.ShiftRegister_fnd(self.bind,val)
"""
#wiringPi
    def __init__(self, n):
        self._data = n[0]
        self._clock = n[1]        
        self._latch = n[2]
        pinMode(self._data,OUTPUT)
        pinMode(self._clock,OUTPUT)
        pinMode(self._latch,OUTPUT)

    def __del__(self):
        digitalWrite(self._data,LOW)
        digitalWrite(self._clock,LOW)
        digitalWrite(self._latch,LOW)

    def shiftout(self, value):
        for i in range(8):
            digitalWrite(self._latch,LOW)
            digitalWrite(self._clock,LOW)
            if ((value>>i)&0x1):
                digitalWrite(self._data,HIGH)
            else:
                digitalWrite(self._data,LOW)
            digitalWrite(self._clock,HIGH)
            digitalWrite(self._latch,HIGH)
            
    def fnd(self, value):
        for i in range(8):
            digitalWrite(self._latch,LOW)
            digitalWrite(self._clock,LOW)
            digitalWrite(self._data,self._fnd_map[value][i])
            digitalWrite(self._clock,HIGH)
            digitalWrite(self._latch,HIGH)

#RPi.GPIO
    def __init__(self, n):
        self._data = n[0]
        self._clock = n[1]        
        self._latch = n[2]
        GPIO.setup(self._data,GPIO.OUT)
        GPIO.setup(self._clock,GPIO.OUT)
        GPIO.setup(self._latch,GPIO.OUT)

    def __del__(self):
        GPIO.output(self._data,GPIO.LOW)
        GPIO.output(self._clock,GPIO.LOW)
        GPIO.output(self._latch,GPIO.LOW)

    def shiftout(self, value):
        for i in range(8):
            GPIO.output(self._latch,GPIO.LOW)
            GPIO.output(self._clock,GPIO.LOW)
            if ((value>>i)&0x1):
                GPIO.output(self._data,GPIO.HIGH)
            else:
                GPIO.output(self._data,GPIO.LOW)
            GPIO.output(self._clock,GPIO.HIGH)
            GPIO.output(self._latch,GPIO.HIGH)
            
    def fnd(self, value):
        for i in range(8):
            GPIO.output(self._latch,GPIO.LOW)
            GPIO.output(self._clock,GPIO.LOW)
            GPIO.output(self._data,self._fnd_map[value][i])
            GPIO.output(self._clock,GPIO.HIGH)
            GPIO.output(self._latch,GPIO.HIGH)
"""

def shiftRegister_unittest():
    gpio = [0,6,5]
    shiftregister = ShiftRegister(gpio)
    for i in range(10):
        shiftregister.shiftout(0x7E)
        shiftregister.shiftout(0x30)
        delay(100)
    for i in range(10):
        shiftregister.fnd(i)
        shiftregister.fnd(i)
        delay(100)

##################################################################

class I2c():
# Using WiringPi 
#    def __init__(self, addr):
#        self._sAddr = wiringPiI2CSetup(addr)
#
#    def __del__(self):
#        self._sAddr = 0
#
#    def read(self):
#        return wiringPiI2CRead(self._sAddr)
#
#    def readByte(self, reg):
#        return wiringPiI2CReadReg8(self._sAddr,reg)
#
#    def readWord(self, reg):
#        return wiringPiI2CReadReg16(self._sAddr,reg)
#
#    def write(self, data):
#        return wiringPiI2CWrite(self._sAddr,data)
#
#    def writeByte(self, reg, data):
#        return wiringPiI2CWriteReg8(self._sAddr,reg,data)
#
#    def writeWord(self, reg, data):
#        return wiringPiI2CWriteReg16(self._sAddr,reg,data)

# Using SMBus 
    def __init__(self,addr,bus=1):
        self._sAddr = addr 
        self._bus = SMBus(bus)

    def __del__(self):
        self._bus.close()
        self._sAddr = 0
        self._bus = 0

    def read(self):
        return self._bus.read_byte(self._sAddr)

    def readByte(self, reg):
        return self._bus.read_byte_data(self._sAddr,reg)

    def readWord(self, reg):
        return self._bus.read_word_data(self._sAddr,reg)

    def readBlock(self, reg, length):
        return self._bus.read_i2c_block_data(self._sAddr,reg,length)

    def write(self, data):
        return self._bus.write_byte(self._sAddr,data)

    def writeByte(self, reg, data):
        return self._bus.write_byte_data(self._sAddr,reg,data)

    def writeWord(self, reg, data):
        return self._bus.write_word_data(self._sAddr,reg,data)

    def writeBlock(self, reg, data):
        return self._bus.write_i2c_block_data(self._sAddr,reg,data)

# Unused
#def i2c_unittest():
#    i2c = I2c(0x1b)
#    data = 3
#    while data != 2:
#        data = i2c.readByte(0x03)
#        print("Debug ---",data)
#        delay(100)

##################################################################
# TempHumi sensor class

# DHT11 class
class Dht11(PopThread):

    def __init__(self, gpio_number):
        binder.Dht11_new.argtype = [ctypes.c_int]
        self._gpio_number = gpio_number
        self.bind = binder.Dht11_new(gpio_number)
        self._param = None
        self._func = None
        # print(self.bind)

    def readHumi(self):
        binder.Dht11_readHumi.restype = ctypes.c_double
        val = binder.Dht11_readHumi(self.bind)
        return val

    def readTemp(self):
        binder.Dht11_readTemp.restype = ctypes.c_double
        val = binder.Dht11_readTemp(self.bind)
        return val

    def setCallback(self, func, param=None, type=GPIO.BOTH):
        __func = None
        __params = list(inspect.signature(func).parameters.keys())
        def wrap_callback_function(param1=None, param2=None):
            if len(__params) == 0:
                func()
            elif param is not None:
                func(param)
            else :
                func(self.readTemp(), self.readHumi())
        __func = wrap_callback_function
        if self._func != None and func == None:
            self.stop()
            self._func = None
        if (func != None):
            self._func = func
            self._param = param
            self.start()

    def run(self):
        while(self.isRun()):
            #data = self.read()
            if self._param:
                self._func(self._parm)
            else:
                self._func(self.readTemp(), self.readHumi())

    def __del__(self):
        binder.Dht11_stop(self.bind)

class TempHumiData:
    temp = 0
    humi = 0

class TempHumi():    
    def __init__(self, gpio_number=0):
        #Bme680 is not implemented in c++ yet.
        #self.bind = binder.TempHumi_new() 
        if board_name is PYC_BASIC:
            # self._device = Sht20()
            self._device = Dht11(gpio_number)
        elif board_name is AIOT_HOME:
            self._device = Bme680()
        elif board_name is ES_101:
            self._device = Sht20()
    def getTemperature(self):
        return self.readTemp()
    def getHumidity(self):
        return self.readHumi()
    def readTemp(self):
        if self._device:
            val = self._device.readTemp()
            print(val)
            return val
        else:
            return None
    
    def readHumi(self):
        if self._device:
            val = self._device.readHumi()
            print(val)
            return val
        else:
            return None

    def read(self):
        temphumi_data = TempHumiData()
        temphumi_data.temp = self.readTemp()
        temphumi_data.humi = self.readHumi()
        return temphumi_data
    def stop(self):
        self._device.stop()
    def setCallback(self, func, param=None, type=GPIO.BOTH):
        self._device.setCallback(func, param, type)

##################################################################
# TempHumi sensor for PyC Basic & ES-101
SHT20_ADDR = 0x40

class Sht20():
    def __init__(self, addr=SHT20_ADDR):
        binder.Sht20_new.argtype = [ctypes.c_int]
        self.bind = binder.Sht20_new(addr)

    def softReset(self):
        binder.Sht20_softReset(self.bind)

    def readTemp(self):
        binder.Sht20_readTemp.restype = ctypes.c_double
        return binder.Sht20_readTemp(self.bind)

    def readHumi(self):
        binder.Sht20_readHumi.restype = ctypes.c_double
        return binder.Sht20_readHumi(self.bind)

"""
#wiringPi
    def __init__(self, addr=SHT20_ADDR, debug=False):
        if debug:
            print("Sht20 address: %x" % addr)
        super().__init__(addr)
        self.softReset()

    def __del(self):
        super().__del__()

    def readTemp(self):
        I2c.write(self,0xF3)
        delay(500)
        data0 = I2c.read(self)
        data1 = I2c.read(self)
        temp = data0 * 256 + data1
        temp = -46.85 + ((temp * 175.72) / 65536.0)
        return temp

    def readHumi(self):
        I2c.write(self,0xF5)
        delay(500)
        data0 = I2c.read(self)
        data1 = I2c.read(self)
        humi = data0 * 256 + data1
        humi = -6 + ((humi * 125.0) / 65536.0)
        return humi

    def softReset(self):
        I2c.write(self,0xFE)
        delay(100)
"""
def sht20_unittest():
    sht20 = Sht20(0x40)
    for i in range(50):
        print("Debug Temp ---",sht20.readTemp())
        print("Debug Humi ---",sht20.readHumi())
        print("\n")
        delay(500)

##################################################################
'''
# This class is maybe DHT11
def strbit2dec(string_num):
    return int(string_num, 2)

class TempHumi(PopThread):
    class TempHumiData():
        def __init__(self):
            self.humi = 0
            self.temp = 0

    def __init__(self, n):
        self._gpio = n
        self._data = TempHumi.TempHumiData()
        self._func = None
        self._param = None

    def __del__(self):
        if (self._func != None):
            self.stop()
            
    def read(self, tempCal=-5):
        data = self._read()

        if (data.temp):
            self._data.humi = data.humi
            self._data.temp = data.temp + tempCal

        return self._data

    def setCallback(self, func, param=None):
        if self._func != None and func == None:          
            self.stop()
            self._func = None

        if (func != None):
            self._func = func
            self._param = param

            self.start()

    def run(self):
        while(self.isRun()):
            data = self.read()
            if self._param:
                self._func(self._param)
            else:
                self._func(data.temp, data.humi)
    
    def _read(self):
        seek = 0
        bits_min = 9999
        bits_max = 0 

        data = []
        effectiveData = []
        HumidityBit = ""
        TemperatureBit = ""
        crcBit = ""
        ret = TempHumi.TempHumiData()

        # pinMode(self._gpio, OUTPUT)
        # digitalWrite(self._gpio, LOW)
        # delay(18)
        # digitalWrite(self._gpio, HIGH)
        # delayMicroseconds(40)
        # pinMode(self._gpio, INPUT)
        GPIO.setup(self._gpio, GPIO.OUT)
        GPIO.output(self._gpio, GPIO.LOW)
        delay(18)
        GPIO.output(self._gpio, GPIO.HIGH)
        delayMicroseconds(40)
        GPIO.setup(self._gpio, GPIO.IN)

        for i in range(0, 1500):
            #data.append(digitalRead(self._gpio))
            data.append(GPIO.input(self._gpio))

        #first plus pass
        while(seek < len(data) and data[seek] == 0):
            seek += 1
        while(seek < len(data) and data[seek] == 1):
            seek += 1
 
        for i in range(40):
            index = 0
            buffer = ""
 
            while(seek < len(data) and data[seek] == 0):
                seek += 1
                index += 1
 
            while(seek < len(data) and data[seek] == 1):
                seek += 1
                buffer += "1"
 
            if (len(buffer) < bits_min):
                bits_min = len(buffer)
 
            if (len(buffer) > bits_max):
                bits_max = len(buffer)
 
            effectiveData.append(buffer)
 
        # ((bits_max + bits_min)/2) base, set LOW, HIGH
        for i in range(0, len(effectiveData)):
            if (len(effectiveData[i]) < ((bits_max + bits_min)/2)):
                effectiveData[i] = "0"
            else:
                effectiveData[i] = "1"
 
        for i in range(0, 8):
            HumidityBit += str(effectiveData[i])
 
        for i in range(16, 24):
            TemperatureBit += str(effectiveData[i])
 
        #for i in range(32, 40):
        #    crcBit += str(effectiveData[i])
 
        ret.humi = strbit2dec(HumidityBit)
        ret.temp = strbit2dec(TemperatureBit)
        #crc = strbit2dec(crcBit)

        return ret

def temphumi_unittest1():
    temphumi = TempHumi(4)

    while (True):
        data = temphumi.read()
        print("temp = %d, humi = %d"%(data.temp, data.humi))
        delay(1500)

def onTempHumi(temp, humi, unuse):
    print("temp = %.2f, humi = %.2f"%(temp, humi))
    delay(1500)

def temphumi_unittest2():
    temphumi = TempHumi(4)
    temphumi.setCallback(onTempHumi)
    input("Press <ENTER> key...\n")
    temphumi.stop()

'''
##################################################################
# TempHumi sensor for AIoT Home
BME680_ADDR = 0x77
class Bme680(BME680):
    # Heater control settings
    ENABLE_HEATER = 0x00
    DISABLE_HEATER = 0x08

    # Gas measurement settings
    DISABLE_GAS_MEAS = 0x00
    ENABLE_GAS_MEAS = 0x01
    
    # Over-sampling settings
    OS_NONE = 0
    OS_1X = 1
    OS_2X = 2
    OS_4X = 3
    OS_8X = 4
    OS_16X = 5

    # IIR filter settings
    FILTER_SIZE_0 = 0
    FILTER_SIZE_1 = 1
    FILTER_SIZE_3 = 2
    FILTER_SIZE_7 = 3
    FILTER_SIZE_15 = 4
    FILTER_SIZE_31 = 5
    FILTER_SIZE_63 = 6
    FILTER_SIZE_127 = 7

    # Power mode settings
    SLEEP_MODE = 0
    FORCED_MODE = 1

    # Run gas enable and disable settings
    RUN_GAS_DISABLE = 0
    RUN_GAS_ENABLE = 1
    
    # Settings selector
    OST_SEL = 1
    OSP_SEL = 2
    OSH_SEL = 4
    GAS_MEAS_SEL = 8
    FILTER_SEL = 16
    HCNTRL_SEL = 32
    RUN_GAS_SEL = 64
    NBCONV_SEL = 128
    GAS_SENSOR_SEL = GAS_MEAS_SEL | RUN_GAS_SEL | NBCONV_SEL

    # Number of conversion settings
    NBCONV_MIN = 0
    NBCONV_MAX = 9 # Was 10, but there are only 10 settings: 0 1 2 ... 8 9

    def __init__(self, addr=BME680_ADDR, debug=False):
        BME680.__init__(self, addr)
        
        #initialize for gas
        self.setGasHeaterTemperature(320)
        self.setGasHeaterDuration(150)
        self.selectGasHeaterProfile(0)
        
        self.temperature = 0
        self.pressure = 0
        self.humidity = 0
        self.gas_resistance = 0

        self._last_reading = 0
        self._min_refresh_time = 0.1

    def init(self):
        self.softReset()
        self.setPowerMode(self.SLEEP_MODE)

        self._getCalibrationData()
        
        self.setHumidityOversample(self.OS_2X)
        self.setPressureOversample(self.OS_4X)
        self.setTemperatureOversample(self.OS_8X)
        self.setFilter(self.FILTER_SIZE_3)
        self.setGasStatus(self.ENABLE_GAS_MEAS)
        self.setTempOffset(0)

    def _getCalibrationData(self):
        self._get_calibration_data()

    def softReset(self):
        self.soft_reset()

    def setTempOffset(self, value):
        self.set_temp_offset(value)

    def setHumidityOversample(self, value):
        self.set_humidity_oversample(value)

    def getHumidityOversample(self):
        return self.get_humidity_oversample()

    def setPressureOversample(self, value):
        self.set_pressure_oversample(value)

    def getPressureOversample(self):
        return self.get_pressure_oversample()

    def setTemperatureOversample(self, value):
        self.set_temperature_oversample(value)

    def getTemperatureOversample(self):
        return self.get_temperature_oversample()

    def setFilter(self, value):
        self.set_filter(value)

    def getFilter(self):
        return self.get_filter()

    def selectGasHeaterProfile(self, value):
        self.gas_heater_index = value
        self.select_gas_heater_profile(value)

    def getGasHeaterProfile(self):
        return self.get_gas_heater_profile()

    def setGasStatus(self, value):
        self.set_gas_status(value)

    def getGasStatus(self):
        return self.get_gas_status()

    def setGasHeaterProfile(self, temperature, duration, nb_profile=0):
        self.set_gas_heater_profile(temperature, duration, nb_profile)

    def setGasHeaterTemperature(self, value, nb_profile=0):
        self.gas_heater_temperature = value
        self.set_gas_heater_temperature(value, nb_profile)

    def setGasHeaterDuration(self, value, nb_profile=0):
        self.gas_heater_duration = value
        self.set_gas_heater_duration(value, nb_profile)

    def setPowerMode(self, value, blocking=True):
        self.set_power_mode(value, blocking)

    def getPowerMode(self):
        return self.get_power_mode()

    def getSensorData(self):
        if self.get_sensor_data():
            d = self.data
            
            self.temperature = d.temperature
            self.pressure = d.pressure
            self.humidity = d.humidity
            self.gas_resistance = d.gas_resistance

            self._last_reading = time.monotonic()

            return((self.temperature, self.pressure, self.humidity, self.gas_resistance))
        else:
            return [None] * 4
            

    def isTime(self):
        if time.monotonic() - self._last_reading < self._min_refresh_time:
            return False
        else:
            return True

    def getTemperature(self):
        if self.isTime():
            self.getSensorData()
        return self.temperature

    def getPressure(self):
        if self.isTime():
            self.getSensorData()
        return self.pressure
        
    def getHumidity(self):
        if self.isTime():
            self.getSensorData()
        return self.humidity
        
    def getGas(self):
        if self.isTime():
            self.getSensorData()
        return self.gas_resistance

    def readTemp(self):
        self.getSensorData()
        return self.temperature
    
    def readHumi(self):
        self.getSensorData()
        return self.humidity

##################################################################
# 6-Axis Accel , Gyro 
MPU6050_ADDR = 0x68

class Mpu6050(I2c):
    def __init__(self, addr=MPU6050_ADDR):
        super().__init__(addr)
        self.accelRawX = 0
        self.accelRawy = 0
        self.accelRawz = 0
        self.accelScaledX = 0
        self.accelScaledY = 0
        self.accelScaledZ = 0
        self.gyroRawX = 0
        self.gyroRawY = 0
        self.gyroRawZ = 0
        self.gyroScaledX = 0
        self.gyroScaledY = 0
        self.gyroScaledZ = 0

    def __del(self):
        super().__del__()

    def readAccel(self):
        I2c.writeByte(self,0x6b,0)
        self.accelRawX = I2c.readWord(self,0x3b)
        self.accelRawY = I2c.readWord(self,0x3d)
        self.accelRawZ = I2c.readWord(self,0x3f)
        self.accelScaledX = float(self.accelRawX/16384.0)
        self.accelScaledY = float(self.accelRawY/16384.0)
        self.accelScaledZ = float(self.accelRawZ/16384.0)

    def readGyro(self):
        I2c.writeByte(self,0x6b,0)
        self.gyroRawX = I2c.readWord(self,0x43)
        self.gyroRawY = I2c.readWord(self,0x45)
        self.gyroRawZ = I2c.readWord(self,0x47)
        self.gyroScaledX = float(self.gyroRawX/131)
        self.gyroScaledY = float(self.gyroRawY/131)
        self.gyroScaledZ = float(self.gyroRawZ/131)

def mpu6050_unittest():
    mpu6050 = Mpu6050(0x68)
    for i in range(50):
        mpu6050.readAccel()
        delay(500)
        mpu6050.readGyro()
        delay(500)
        print("\naccel x raw : ",mpu6050.accelRawX,", scaled : ",mpu6050.accelScaledX)
        print("accel y raw : ",mpu6050.accelRawY,", scaled : ",mpu6050.accelScaledY)
        print("accel z raw : ",mpu6050.accelRawZ,", scaled : ",mpu6050.accelScaledZ)
        print("\ngyro x raw : ",mpu6050.gyroRawX,", scaled : ",mpu6050.gyroScaledX)
        print("gyro y raw : ",mpu6050.gyroRawY,", scaled : ",mpu6050.gyroScaledY)
        print("gyro z raw : ",mpu6050.gyroRawZ,", scaled : ",mpu6050.gyroScaledZ)

##################################################################
# DC Fan
class Fan(Out):
    def __init__(self, n=-1):
        if n < 0:
            n = pinMap(DC_FAN)
        super().__init__(n)

    def __del__(self):
        super().__del__()

def fan_unittest():
    fan = Fan(26)
    for i in range(10):
        if i%2 == 0:
            fan.on()
        else:
            fan.off()

##################################################################
# Text LCD
class Textlcd():
#cpp binding
    def __init__(self):
        self.bind = binder.Textlcd_new()
    
    def command(self, command):
        binder.Textlcd_command.argtype = [ctypes.c_uint8]
        binder.Textlcd_command(self.bind, command)
    
    def clear(self):
        binder.Textlcd_clear(self.bind)
    
    def returnHome(self):
        binder.Textlcd_returnHome(self.bind)
    
    def displayOn(self):
        binder.Textlcd_displayOn(self.bind)
    
    def displayOff(self):
        binder.Textlcd_displayOff(self.bind)
    
    def displayShiftR(self):
        binder.Textlcd_displayShiftR(self.bind)
    
    def displayShiftL(self):
        binder.Textlcd_displayShiftL(self.bind)
    
    def cursorOn(self, blink=False):
        binder.Textlcd_cursorOn.argtype = [ctypes.c_uint8]
        binder.Textlcd_cursorOn(self.bind, blink)
    
    def cursorOff(self):
        binder.Textlcd_cursorOff(self.bind)
    
    def cursorShiftR(self):
        binder.Textlcd_cursorShiftR(self.bind)
    
    def cursorShiftL(self):
        binder.Textlcd_cursorShiftL(self.bind)
    
    def entryModeSet(self):
        binder.Textlcd_entryModeSet(self.bind)

    def setCursor(self, x, y):
        binder.Textlcd_setCursor.argtype = [ctypes.c_uint, ctypes.c_uint]
        binder.Textlcd_setCursor(self.bind, x, y)
        
    def data(self, data):
        binder.Textlcd_data.argtype = [ctypes.c_uint8]
        binder.Textlcd_data(self.bind, data)

    def print(self, string):
        binder.Textlcd_print.argtype = [ctypes.c_char_p]
        binder.Textlcd_print(self.bind, string.encode('utf-8'))
        # binder.TextLcd_print(self.bind, string)

def textlcd_unittest():
    gpio = [4,-1,17,13,16,22,27]
    lcd = Textlcd(gpio,Textlcd.CONTROL_MODE_4BIT)
    lcd.lcdInit()
    lcd.cursorOn()
    delay(1000)
    lcd.cursorShiftR()
    lcd.cursorShiftR()
    delay(1000)
    lcd.cursorShiftL()
    delay(1000)
    lcd.returnHome()

'''
# wiRingPi
    CONTROL_MODE_4BIT = 4
    CONTROL_MODE_8BIT = 8
    TEXT_LINE_1 = 1
    TEXT_LINE_2 = 2

    dataPin = [0,0,0,0,0,0,0,0]

    def __init__(self, gpio, control_mode):
        self.mode = control_mode
        self._rs = gpio[0]
        self._rw = gpio[1]        
        self._en = gpio[2]
        for i in range(self.mode):
            self.dataPin[i] = gpio[i+3]

        pinMode(self._rs,OUTPUT)
        if self._rw != -1:
            pinMode(self._rw,OUTPUT)
        pinMode(self._en,OUTPUT)
        for i in range(self.mode):
            pinMode(self.dataPin[i],OUTPUT)

    def __del__(self):
        digitalWrite(self._rs,LOW)
        digitalWrite(self._rw,LOW)
        digitalWrite(self._en,LOW)
        for i in range(self.mode):
            digitalWrite(self.dataPin[i],LOW)

    def setCommand(self, cmd):
        digitalWrite(self._rs,LOW)
        digitalWrite(self._en,LOW)
        if self.mode == self.CONTROL_MODE_4BIT:
            msb = (cmd&0xf0)>>4
            lsb = cmd&0x0f
            for i in range(self.mode):
                if msb&0x01:
                    digitalWrite(self.dataPin[i],HIGH)
                else:
                    digitalWrite(self.dataPin[i],LOW)
                msb = msb>>1
            digitalWrite(self._en,HIGH)
            delay(50)
            digitalWrite(self._en,LOW)
            delay(50)
            for i in range(self.mode):
                if lsb&0x01:
                    digitalWrite(self.dataPin[i],HIGH)
                else:
                    digitalWrite(self.dataPin[i],LOW)
                lsb = lsb>>1
        elif self.mode == self.CONTROL_MODE_8BIT:
            for i in range(self.mode):
                if cmd&0x01:
                    digitalWrite(self.dataPin[i],HIGH)
                else:
                    digitalWrite(self.dataPin[i],LOW)
                cmd = cmd>>1
        digitalWrite(self._en,HIGH)
        delay(50)
        digitalWrite(self._en,LOW)
        delay(100)
        
    def writeByte(self,data):
        digitalWrite(self._rs,HIGH)
        digitalWrite(self._en,LOW)
        delay(1)
        if self.mode == self.CONTROL_MODE_4BIT:
            msb = (data&0xf0)>>4
            lsb = data&0x0f
            for i in range(self.mode):
                if msb&0x01:
                    digitalWrite(self.dataPin[i],HIGH)
                else:
                    digitalWrite(self.dataPin[i],LOW)
                msb = msb >> 1
            digitalWrite(self._en,HIGH)
            delay(1)
            digitalWrite(self._en,LOW)
            delay(1)
            for i in range(self.mode):
                if lsb&0x01:
                    digitalWrite(self.dataPin[i],HIGH)
                else:
                    digitalWrite(self.dataPin[i],LOW)
                lsb = lsb >> 1
        elif self.mode == self.CONTROL_MODE_8BIT:
            for i in range(self.mode):
                if data&0x01:
                    digitalWrite(self.dataPin[i],HIGH)
                else:
                    digitalWrite(self.dataPin[i],LOW)
                data = data >> 1
        digitalWrite(self._en,HIGH)
        delay(1)
        digitalWrite(self._en,LOW)
        delay(10)

    def lcdInit(self):
        if self.mode == self.CONTROL_MODE_4BIT:
            self.setCommand(0x30)
            delay(500)
            self.setCommand(0x30)
            delay(100)
            self.setCommand(0x32)
        self.functionSet()
        delay(500)
        self.displayOff()
        self.clearDisplay()
        self.entryModeSet()

    def functionSet(self):
        if self.mode == self.CONTROL_MODE_4BIT:
            self.setCommand(0x28)
        elif self.mode == self.CONTROL_MODE_8BIT:
            self.setCommand(0x38)
    
    def displayOn(self):
        self.setCommand(0x08)

    def displayOff(self):
        self.setCommand(0x0c)

    def displayShiftR(self):
        self.setCommand(0x1c)

    def displayShiftL(self):
        self.setCommand(0x18)

    def cursorOn(self):
        self.setCommand(0x0f)

    def cursorOff(self):
        self.setCommand(0x0e)

    def cursorShiftL(self):
        self.setCommand(0x10)

    def cursorShiftR(self):
        self.setCommand(0x14)

    def entryModeSet(self):
        self.setCommand(0x06)

    def returnHome(self):
        self.setCommand(0x02)

    def clearDisplay(self):
        self.setCommand(0x01)

    def setLine1(self):
        self.setCommand(0x80)

    def setLine2(self):
        self.setCommand(0xc0)

    def writeString(self,str,line):
        if line == self.TEXT_LINE_1:
            self.setLine1()
        elif line == self.TEXT_LINE_2:
            self.setLine2()
        for char in str:
            self.writeByte(ord(char))

#RPi.GPIO
    def __init__(self, addr=LCD_1602_ADDR):
        super().__init__(addr)
        self.command(0x33)
        self.command(0x32)

        self.command(0x28)
        self.command(0x0F)
        self.command(0x06)
        self.command(0x01)
        delay(100)

        self.display_status = 0x0F

        self.returnHome()
        self.print("Text LCD Init")

    def __del__(self):
        self.display_status = 0x00
        #self.clear()

    def _byte(self, byte, mode):
        high_bit = mode | (byte & 0xF0) | self.LCD_1602_BACKLIGHT
        low_bit = mode | ((byte << 4) & 0xF0) | self.LCD_1602_BACKLIGHT
        self._enable(high_bit)
        self._enable(low_bit)

    def _enable(self, byte):
        delay(5)
        I2c.write(self, byte | self.ENABLE)
        delay(5)
        I2c.write(self, byte & ~self.ENABLE)
        delay(5)
        
    def command(self, command):
        self._byte(command,self.LCD_1602_CMD)

    def clear(self):
        self.command(self.LCD_1602_CLEAR)

    def returnHome(self):
        self.command(self.LCD_1602_HOME)

    def displayOn(self):
        self.display_status = self.display_status | self.LCD_1602_DISPLAY
        self.command(self.display_status)

    def displayOff(self):
        self.display_status = self.display_status & ~self.LCD_1602_DISPLAY
        self.command(self.display_status)

    def displayShiftR(self):
        self.command(self.LCD_1602_DISPLAY_SHIFT_R)

    def displayShiftL(self):
        self.command(self.LCD_1602_DISPLAY_SHIFT_L)

    def cursorOn(self, blinking):
        self.display_status = self.display_status | self.LCD_1602_CURSOR
        if blinking == 1:
            self.display_status = self.display_status | self.LCD_1602_BLINKING
        else:
            self.display_status = self.display_status & ~self.LCD_1602_BLINKING

        self.command(self.display_status)

    def cursorOff(self):
        self.display_status = self.display_status & ~self.LCD_1602_CURSOR
        self.display_status = self.display_status & ~self.LCD_1602_BLINKING
        self.command(self.display_status)

    def cursorShiftR(self):
        self.command(self.LCD_1602_CURSOR_SHIFT_R)

    def cursorShiftL(self):
        self.command(self.LCD_1602_CURSOR_SHIFT_L)

    def entryModeSet(self):
        self.command(self.LCD_1602_ENTRY_MODE_SET)

    def setCursor(self, x, y):
        if x > 15:
            x = 15
        if y >= 1:
            y = self.LCD_1602_LINE2
        else:
            y = self.LCD_1602_LINE1
        self.command(0x80 | (x+y))

    def data(self, data):
        self._byte(data, self.LCD_1602_CHR)

    def print(self, str):
        for i in str:
            self.data(ord(i))
'''
##################################################################
# Touch sensor class
class Touch():
    def __init__(self,n=0):
        if board_name is PYC_BASIC:
            self._device = Input(n)
        elif board_name is AIOT_HOME:
            self._device = Mpr121()
        elif board_name is ES_101:
            self._device = At42qt1070()

    def read(self):
        if self._device:
            return self._device.read()
        else:
            return None

    def readChannel(self, n):
        return self._device.readChannel(n)

    def setCallback(self, func, param=None, type=GPIO.BOTH):
        self._device.setCallback(func, param, type)

def touch_unittest1():
    touch = Touch(2)

    while (True):
        print("%d"%(touch.read()))
        delay(100)

def touch_unittest2():
    touch = Touch(2)
    old = False

    while (True):
        cur = touch.read()

        if (cur != old):
            print("%s"%("press" if (cur) else "relese"))
            old = cur

        delay(100)

def onTouch(touch):
    print("%s"%("press" if touch.read() else "relese"))
    delay(100)

def touch_unittest3():
    touch = Touch(2)
    touch.setCallback(onTouch, touch)
    input("Press <ENTER> key...\n")

##################################################################
# Touch Sensor for AIoT Connected Home 
class Mpr121(I2c):
    MPR121_ADDR            = 0x5A
    MPR121_TOUCHSTATUS_L   = 0x00
    MPR121_TOUCHSTATUS_H   = 0x01
    MPR121_FILTDATA_0L     = 0x04
    MPR121_FILTDATA_0H     = 0x05
    MPR121_BASELINE_0      = 0x1E
    MPR121_MHDR            = 0x2B
    MPR121_NHDR            = 0x2C
    MPR121_NCLR            = 0x2D
    MPR121_FDLR            = 0x2E
    MPR121_MHDF            = 0x2F
    MPR121_NHDF            = 0x30
    MPR121_NCLF            = 0x31
    MPR121_FDLF            = 0x32
    MPR121_NHDT            = 0x33
    MPR121_NCLT            = 0x34
    MPR121_FDLT            = 0x35
    MPR121_TOUCHTH_0       = 0x41
    MPR121_RELEASETH_0     = 0x42
    MPR121_DEBOUNCE        = 0x5B
    MPR121_CONFIG1         = 0x5C
    MPR121_CONFIG2         = 0x5D
    MPR121_ECR             = 0x5E
    MPR121_SOFTRESET       = 0x80

    def __init__(self, addr=MPR121_ADDR):
        super().__init__(addr)
        self._channels = [None]*12
        self.reset()

    def __del__(self):
        super().__del__()

    def reset(self):
        I2c.writeByte(self,self.MPR121_SOFTRESET,0x63)
        delay(100)
        I2c.writeByte(self,self.MPR121_ECR,0x00)
        data = I2c.readByte(self,self.MPR121_CONFIG2)
        if data != 0x24:
            print("MPR121 config ERROR MPR121_CONFIG2\n")
        for i in range(12):
            I2c.writeByte(self,self.MPR121_TOUCHTH_0+2*i,12)
            I2c.writeByte(self,self.MPR121_RELEASETH_0+2*i,6)
        I2c.writeByte(self,self.MPR121_MHDR,0x01)
        I2c.writeByte(self,self.MPR121_NHDR, 0x01)
        I2c.writeByte(self,self.MPR121_NCLR, 0x0E)
        I2c.writeByte(self,self.MPR121_FDLR, 0x00)
        I2c.writeByte(self,self.MPR121_MHDF, 0x01)
        I2c.writeByte(self,self.MPR121_NHDF, 0x05)
        I2c.writeByte(self,self.MPR121_NCLF, 0x01)
        I2c.writeByte(self,self.MPR121_FDLF, 0x00)
        I2c.writeByte(self,self.MPR121_NHDT, 0x00)
        I2c.writeByte(self,self.MPR121_NCLT, 0x00)
        I2c.writeByte(self,self.MPR121_FDLT, 0x00)
        I2c.writeByte(self,self.MPR121_DEBOUNCE, 0)
        I2c.writeByte(self,self.MPR121_CONFIG1, 0x10) 
        I2c.writeByte(self,self.MPR121_CONFIG2, 0x20) 
        I2c.writeByte(self,self.MPR121_ECR, 0x8F) 

    def keyStatus(self):
        value = I2c.readByte(self, 0x00)
        for i in range(8):
            self._channels[i] = value & 0x01
            value = value >> 1
        
    def readChannel(self,ch):
        value = 0 
        if ch >= 0 and ch < 8:
            value = I2c.readByte(self,0x00)
            value = (value >> ch) & 0x1 
        elif ch >= 8 and ch <12:
            value = I2c.readByte(self,0x01)
            value = (value >> (ch-8)) & 0x1
        self._channels[ch] = value
        return self._channels[ch]

    def readAll(self):
        value = I2c.readByte(self,0x00)
        for i in range(8):
            self._channels[i] = value & 0x01
            value = value >> 1
        value = I2c.readByte(self,0x01)
        for i in range(4):
            self._channels[i+8] = value & 0x01
            value = value >> 1
        return self._channels

    def read(self):
        return self.readAll()

##################################################################
# Touch Sensor for ES-101 
AT42QT1070_ADDR = 0x1B

class At42qt1070(I2c):
    def __init__(self, addr=AT42QT1070_ADDR):
        binder.At42qt1070_new.argtype = [ctypes.c_int]
        self.bind = binder.At42qt1070_new(addr)

    def keyStatus(self):
        return binder.At42qt1070_keyStatus()

    def read(self):
        return binder.At42qt1070_keyStatus()

def At42qt1070_unittest():
    at42qt1070 = At42qt1070()
    data = 3
    while data != 2:
        data = at42qt1070.keyStatus()
        print("Debug ---",data)
        delay(100)

##################################################################
# I2C Dust Sensor for AIoT Connected Home 
class Dust():
    def __init__(self,n=0, LED=6):
        if board_name is PYC_BASIC:
            self._device = SpiAdc(channel=n,device=0, bus=0, speed=1000000)
            self._device.setSample(64)
            self._LED = LED
            GPIO.setup(self._LED, GPIO.OUT)
        elif board_name is AIOT_HOME:
            self._device = Pm2008m(0x28)
            self.pm_1p0_grimm = 0
            self.pm_2p5_grimm = 0
            self.pm_10_grimm = 0
            self.pm_1p0_tsi = 0
            self.pm_2p5_tsi = 0
            self.pm_10_tsi = 0
            self.num_0p3 = 0
            self.num_0p5 = 0
            self.num_1 = 0
            self.num_2p5 = 0
            self.num_5 = 0
            self.num_10 = 0
        elif board_name is ES_101:
            self._device = None

    def read(self):
        if self._device:
            if board_name is PYC_BASIC:
                GPIO.output(self._LED, GPIO.LOW)
                delayMicroseconds(280)
            val = self._device.read()
            if board_name is AIOT_HOME:
                self.pm_1p0_grimm = self._device.pm_1p0_grimm               
                self.pm_2p5_grimm = self._device.pm_2p5_grimm               
                self.pm_10_grimm = self._device.pm_10_grimm                
                self.pm_1p0_tsi = self._device.pm_1p0_tsi                 
                self.pm_2p5_tsi = self._device.pm_2p5_tsi                 
                self.pm_10_tsi = self._device.pm_10_tsi
                self.num_0p3 = self._device.num_0p3                    
                self.num_0p5 = self._device.num_0p5
                self.num_1 = self._device.num_1
                self.num_2p5 = self._device.num_2p5
                self.num_5 = self._device.num_5
                self.num_10 = self._device.num_10
            if board_name is PYC_BASIC:
                delayMicroseconds(40)
                GPIO.output(self._LED, GPIO.HIGH)
                delayMicroseconds(9680)
            return val
        else:
            return None
    
    def readAverage(self):
        if self._device:
            return self._device.readAverage()
        else:
            return None

    # def setCallback(self, func, param=None, type=TYPE_AVERAGE, mode=MODE_FULL, min=0, max=ADC_MAX):
    #     if self._device:
    #         self._device.setCallback(func, param, type, mode, min, max)

##################################################################
# ADC Dust Sensor for ...
class Gp2y1010au0f():
    def __init__(n, channel, device=0, bus=0, speed=100000):
        self.bind = binder.Gp2y1010au0f_new(n, channel, device, bus, speed)
    
    def read(self):
        return binder.Gp2y1010au0f_read(self.bind)

    def readAverage(self):
        binder.Gp2y1010au0f_readAverage(self.bind)

    def calcDensity(self, val=None, calibration=None):
        if val is None:
            val = self.readAverage()
        if calibration is None:
            calibration = 70

        binder.Gp2y1010au0f_calcDensity.argtype = [ctypes.c_int, ctypes.c_int]
        binder.Gp2y1010au0f_calcDensity.restype = ctypes.c_double
        return binder.Gp2y1010au0f_calcDensity(val, calibration)

'''
#wiringPi
    SAMPLING_TIME = 280
    DELTA_TIME = 40
    SLEEP_TIME = 9680

    def __init__(self, n, channel, device=0, bus=0, speed=1000000):
        super().__init__(channel, device, bus, speed)
        self._gpio = n
        self.setSample(64)

        pinMode(self._gpio, OUTPUT)

    def read(self):
        digitalWrite(self._gpio, LOW)
        delayMicroseconds(Dust.SAMPLING_TIME)
blass Dust():
        val = SpiAdc.read(self)
        delayMicroseconds(Dust.DELTA_TIME)
        digitalWrite(self._gpio, HIGH)
        delayMicroseconds(Dust.SLEEP_TIME)
        
        return val

    def calcDensity(self, val, calibration=70):
        volt = SpiAdc.REF_VOLTAG * (val / SpiAdc.ADC_MAX)
        density = (0.17 * volt - 0.1) * 1000

        return density + calibration

    def readAverage(self):
        val = 0.0
        sample = self.getSample()

        for _ in range(sample):
            val += math.pow(self.read(), 2)

        return math.sqrt(val /sample)

def dust_unittest1():
    dust = Dust(2, 6)

    while (True):
        val = dust.read()
        print("%d"%(val))
        delay(1000)

def dust_unittest2():
    dust = Dust(2, 6)

    """
#    < 35mg/m^3 : 좋음
#    < 75mg/m^3 : 나쁨
#    > 75mg/m^3 : 매우 나쁨
    """

    while (True):
        val = dust.readAverage()
        density = dust.calcDensity(val)

        print("%.2fmg/^3"%(density))
        delay(1000)

def onDust(val, dust):
    density = dust.calcDensity(val)
    print("%.2fmg/^3"%(density))
    delay(1000)

def dust_unittest3():
    dust = Dust(2, 6)
    dust.setCallback(onDust, dust)
    input("Press <ENTER> key...\n")
'''

##################################################################
# I2C Dust Sensor for AIoT Connected Home 
PM2008M_ADDR = 0x28

class Pm2008m(I2c):
    def __init__(self, addr=PM2008M_ADDR):
        super().__init__(addr)
        self._rBuf = [None]*32
        self.sensor_status = 0
        self.measuring_mode = 0
        self.calibration_factor = 0
        self.pm_1p0_grimm = 0
        self.pm_2p5_grimm = 0
        self.pm_10_grimm = 0
        self.pm_1p0_tsi = 0
        self.pm_2p5_tsi = 0
        self.pm_10_tsi = 0
        self.num_0p3 = 0
        self.num_0p5 = 0
        self.num_1 = 0
        self.num_2p5 = 0
        self.num_5 = 0
        self.num_10 = 0
        self.reset()

    def __del__(self):
        super().__del__()

    def reset(self):
        data = [0x16,0x7,0x03,0xff,0xff,0x00,0x16]
        for i in range(5):
            data[6] = data[i+1]
        I2c.writeBlock(self,0x50,data)

    def _read(self):
        self._rBuf = I2c.readBlock(self,0x50,32)
        # print("length: %d" % len(self._rBuf))
        a = ""
        for i in range(len(self._rBuf)):
            a = a + hex(self._rBuf[i]) + " "
        # print(a)
        if self._rBuf[0] != 0x16:
            print("PM2008M Frame Heater is wrong!\n")
        else:
            if self._rBuf[1] != 32:
                print("PM2008M Frame Length is wrong!\n")
            else:
                check = self._rBuf[0]
                for i in range(30):
                    check = check ^ self._rBuf[i+1]
                # print("check: %d" % check)
                # print("self._rBuf[31]: %d" % self._rBuf[31])
                if check != self._rBuf[31]:
                    print("PM2008M Check Code is wrong!\n")
                else:
                    self.sensor_status = self._rBuf[2]
                    self.measuring_mode = (self._rBuf[3]<<8)|self._rBuf[4]
                    self.calibration_factor = (self._rBuf[5]<<8)|self._rBuf[6]
                    self.pm_1p0_grimm = (self._rBuf[7]<<8)|self._rBuf[8]
                    self.pm_2p5_grimm = (self._rBuf[9]<<8)|self._rBuf[10]
                    self.pm_10_grimm = (self._rBuf[11]<<8)|self._rBuf[12]
                    self.pm_1p0_tsi = (self._rBuf[13]<<8)|self._rBuf[14]
                    self.pm_2p5_tsi = (self._rBuf[15]<<8)|self._rBuf[16]
                    self.pm_10_tsi = (self._rBuf[17]<<8)|self._rBuf[18]
                    self.num_0p3 = (self._rBuf[19]<<8)|self._rBuf[20]
                    self.num_0p5 = (self._rBuf[21]<<8)|self._rBuf[22]
                    self.num_1 = (self._rBuf[23]<<8)|self._rBuf[24]
                    self.num_2p5 = (self._rBuf[25]<<8)|self._rBuf[26]
                    self.num_5 = (self._rBuf[27]<<8)|self._rBuf[28]
                    self.num_10 = (self._rBuf[29]<<8)|self._rBuf[30]

    def read(self):
        self._read()

        ret = dict()
        ret["pm_1p0_grimm"] = self.pm_1p0_grimm
        ret["pm_2p5_grimm"] = self.pm_2p5_grimm
        ret["pm_10_grimm"] = self.pm_10_grimm
        ret["pm_1p0_tsi"] = self.pm_1p0_tsi
        ret["pm_2p5_tsi"] = self.pm_2p5_tsi
        ret["pm_10_tsi"] = self.pm_10_tsi
        ret["num_0p3"] = self.num_0p3
        ret["num_0p5"] = self.num_0p5
        ret["num_1"] = self.num_1
        ret["num_2p5"] = self.num_2p5
        ret["num_5"] = self.num_5
        ret["num_10"] = self.num_10

        return ret

def pm2008m_unittest():
    pm2008m = Pm2008m()
    while True:
        data = pm2008m.read()
        print("\n")
        print("PM 1.0 GRIM  : %u ㎍/㎥"%pm2008m.pm_1p0_grimm)
        print("PM 2.5 GRIM  : %u ㎍/㎥"%pm2008m.pm_2p5_grimm)
        print("PM 10  GRIM  : %u ㎍/㎥"%pm2008m.pm_10_grimm)
        print("PM 1.0 TSI   : %u ㎍/㎥"%pm2008m.pm_1p0_tsi)
        print("PM 2.5 TSI   : %u ㎍/㎥"%pm2008m.pm_2p5_tsi)
        print("PM 10  TSI   : %u ㎍/㎥"%pm2008m.pm_10_tsi)
        print("Number of 0.3 ㎛ : %u pcs/0.1L"%pm2008m.num_0p3)
        print("Number of 0.5 ㎛ : %u pcs/0.1L"%pm2008m.num_0p5)
        print("Number of 1 ㎛ : %u pcs/0.1L"%pm2008m.num_1)
        print("Number of 2.5 ㎛ : %u pcs/0.1L"%pm2008m.num_2p5)
        print("Number of 5 ㎛ : %u pcs/0.1L"%pm2008m.num_5)
        print("Number of 10 ㎛ : %u pcs/0.1L"%pm2008m.num_10)
        delay(1000)

##################################################################
# PWM Controller IC
class PwmController():
    def __init__(self):
        self.bind = binder.PwmController_new()

    def init(self):
        if self.bind is None:
            self.bind = binder.PwmController_new()

    def setChannel(self, channel):
        binder.PwmController_setChannel.argtype = [ctypes.c_int]
        binder.PwmController_setChannel(self.bind, channel)
    
    def setDuty(self, percent):
        binder.PwmController_setDuty.argtype = [ctypes.c_int]
        binder.PwmController_setDuty(self.bind, percent)

    def setFreq(self, freq):
        binder.PwmController_setFreq.argtype = [ctypes.c_int]
        binder.PwmController_setFreq(self.bind, freq)

    def setInvertPulse(self):
        binder.PwmController_setInvertPulse()

##################################################################
# PWM Controller IC 
class Pca9685():
    PCA9685_ADDR = 0x5E
    PCA9685_REG_MODE1 = 0x00
    PCA9685_REG_MODE2 = 0x01
    PCA9685_REG_PRESCALE = 0xFE
    PCA9685_REG_LED0_ON_L = 0x06
    PCA9685_REG_LED0_ON_H = 0x07
    PCA9685_REG_LED0_OFF_L = 0x08
    PCA9685_REG_LED0_OFF_H = 0x09
    PCA9685_REG_ALL_ON_L = 0xFA
    PCA9685_REG_ALL_ON_H = 0xFB
    PCA9685_REG_ALL_OFF_L = 0xFC
    PCA9685_REG_ALL_OFF_H = 0xFD

    RESTART = 1<<7
    AI = 1<<5
    SLEEP = 1<<4
    ALLCALL = 1<<0
    OCH = 1<<3
    OUTDRV = 1<<2
    INVRT = 1<<4
#cpp binding
    def __init__(self, addr=PCA9685_ADDR):
        binder.Pca9685_new.argtype = [ctypes.c_int]
        self.bind = binder.Pca9685_new(addr)

    def init(self):
        binder.Pca9685_init(self.bind)

    def setChannel(self, channel):
        binder.Pca9685_setChannel.argtype = [ctypes.c_int]
        binder._setChannel(self.bind, channel)
    
    def setDuty(self, percent):
        binder.PwmController_setDuty.argtype = [ctypes.c_int]
        binder.PwmController_setDuty(self.bind, percent)

    def setFreq(self, freq):
        binder.PwmController_setFreq.argtype = [ctypes.c_int]
        binder.PwmController_setFreq(self.bind, freq)

    def setDuty(self):
        binder.PwmController_setInvertPulse(self.bind)
"""
#wiringPi
    def __init__(self, addr=PCA9685_ADDR):
        super().__init__(addr)
        self._curChannel = -1

    def __del(self):
        super().__del__()

    def init(self):
        buf = self.AI | self.ALLCALL
        I2c.writeByte(self,self.PCA9685_REG_MODE1,buf)
        buf = self.OCH | self.OUTDRV
        I2c.writeByte(self,self.PCA9685_REG_MODE2,buf)
        delay(50)
        recv = I2c.readByte(self,self.PCA9685_REG_MODE1)
        buf = recv & (~self.SLEEP)
        I2c.writeByte(self,self.PCA9685_REG_MODE1,buf)

    def setChannel(self, ch):
        self._curChannel = ch

    def setDuty(self, percent):
        step = int(round(percent * (4096.0 / 100.0)))
        if step < 0:
            on = 0
            off = 4096
        elif step > 4095:
            on = 4096
            off = 0
        else:
            on = 0
            off = step

        on_l = on&0xff
        on_h = on>>8
        off_l = off&0xff
        off_h = off>>8
        if self._curChannel >= 0:
            I2c.writeByte(self,self.PCA9685_REG_LED0_ON_L+4*self._curChannel,on_l)
            I2c.writeByte(self,self.PCA9685_REG_LED0_ON_H+4*self._curChannel,on_h)
            I2c.writeByte(self,self.PCA9685_REG_LED0_OFF_L+4*self._curChannel,off_l)
            I2c.writeByte(self,self.PCA9685_REG_LED0_OFF_H+4*self._curChannel,off_h)
        elif self._curChannel == -1:
            I2c.writeByte(self,self.PCA9685_REG_ALL_ON_L,on_l)
            I2c.writeByte(self,self.PCA9685_REG_ALL_ON_H,on_h)
            I2c.writeByte(self,self.PCA9685_REG_ALL_OFF_L,off_l)
            I2c.writeByte(self,self.PCA9685_REG_ALL_OFF_H,off_h)

    def setFerq(self, freq):
        prescale = int(round(25000000/(4096*freq))-1)
        if prescale < 3:
            prescale = 3;
        elif prescale > 255:
            prescale = 255

        recv = I2c.readByte(self,self.PCA9685_REG_MODE1)
        buf = (recv &(~self.SLEEP))|self.SLEEP;
        I2c.writeByte(self,self.PCA9685_REG_MODE1,buf)
        I2c.writeByte(self,self.PCA9685_REG_PRESCALE,prescale)
        I2c.writeByte(self,self.PCA9685_REG_MODE1,recv)
        delay(50)
        buf = recv | self.RESTART
        I2c.writeByte(self,self.PCA9685_REG_MODE1,buf)

    def setInvertPulse(self):
        recv = I2c.readByte(self,self.PCA9685_REG_MODE2)
        buf = recv | self.INVRT
        I2c.writeByte(self,self.PCA9685_REG_MODE2,buf)
        delay(50)
"""

def pca9685_unittest():
    pca9685 = Pca9685(0x5e)
    pca9685.init()
    pca9685.setChannel(0)
    pca9685.setDuty(0)
    pca9685.setFerq(200)
    for i in range(100):
        pca9685.setDuty(i)
        delay(100)
    pca9685.setDuty(0)
    pca9685.setChannel(1)
    for i in range(100):
        pca9685.setDuty(i)
        delay(100)
    pca9685.setDuty(0)
    pca9685.setChannel(2)
    for i in range(100):
        pca9685.setDuty(i)
        delay(100)
    pca9685.setDuty(0)
    pca9685.setChannel(-1)
    for i in range(100):
        pca9685.setDuty(i)
        delay(100)
    pca9685.setDuty(0)

##################################################################
#HW PWM 
class HwPwm(object):
    def __init__(self):
        self.h = binder.HwPwm_new(18)

##################################################################
#PiezoBuzzer 

class PiezoBuzzer(PopThread):
    REST = 0
    DO = 1
    DO_ = 2
    RE = 3
    RE_ = 4
    MI = 5
    FA = 6
    FA_ = 7
    SOL = 8
    SOL_ = 9
    RA = 10
    RA_ = 11
    SI = 12

    def __init__(self, gpio=-1):
        if gpio < 0:
            gpio = pinMap(PIEZO_BUZZER)

        binder.PiezoBuzzer_new.argtype = [ctypes.c_uint]
        self.bind = binder.PiezoBuzzer_new(gpio)
    
    def __del__(self):
        binder.PiezoBuzzer_delete(self.bind)
        
    def setTempo(self,n):
        binder.PiezoBuzzer_setTempo(self.bind,n)

    def getTempo(self):
        return binder.PiezoBuzzer_getTempo(self.bind)

    def tone(self,scale,pitch,duration):
        binder.PiezoBuzzer_tone(self.bind,scale,pitch,duration)

    def rest(self,duration):
        binder.PiezoBuzzer_rest(self.bind,duration)

    def play(self, sheet):
        self._Sheet = sheet
        self.start()
    
    def isPlay(self):
        return self.isRun()
    
    def run(self):
        for s, p, d in zip(self._Sheet[0], self._Sheet[1], self._Sheet[2]):
            self.tone(s, p, d)
            if not self.isRun():
                break
        self.stop()
'''
# RPi.GPIO
class PiezoBuzzer():
    def __init__(self, n=-1, freq=261, duty=50):
        if n < 0:
            n = pinMap(PIEZO_BUZZER)

        GPIO.setup(n, GPIO.OUT)
        self.piezo = GPIO.PWM(n, freq)
        self.piezo.start(duty)

    def __del__(self):
        self.piezo.stop()

    def setFreq(self,freq):
        self.piezo.ChangeFrequency(freq)
    
    def setDuty(self,duty):
        self.piezo.ChangeDutyCycle(duty)

    def tone(self,scale,pitch,duration):
        freq = int(math.pow(2,scale-1)*55*pow(2,(pitch -10)/12))
        self.setFreq(freq)
        self.piezo.start(50)
        time.sleep(duration*0.1)
        self.piezo.stop()
'''
def piezo_buzzer_unittest():
    p = PiezoBuzzer(12)

    p.setTempo(120)
    p.tone(4, 8, 4)
    p.tone(4, 8, 4)
    p.tone(4, 10, 4)
    p.tone(4, 10, 4)
    p.tone(4, 8, 4)
    p.tone(4, 8, 4)
    p.tone(4, 5, 2)

##################################################################
# NeoPixel -> Only PWM CH0 Pin (ex -> D18, D12 Pin in Raspberry)
#           -> Using ws281x with PWM CH1, error occured... 

"""
class LedStrip(_pixelbuf.PixelBuf):
    bpp = None
    n = 0

    #Pixel Color 
    RGB = 'RGB'     # Red Green Blue
    GRB = 'GRB'     # Green Red Blue
    RGBW = 'RGBW'   # Red Green Blue White
    GRBW = 'GRBW'   # Green Red Blue White

    def __init__(self, n, *, bpp=3, brightness=1.0,auto_write=True,pixel_order=None):
        self.bpp = bpp
        self.n = n
        if not pixel_order:
            pixel_order = self.GRB if bpp == 3 else self.GRBW
        else:
            self.bpp = bpp = len(pixel_order)
            if isinstance(pixel_order, tuple):
                order_chars = self.RGBW
                order = []
                for char_no, order in enumerate(pixel_order):
                    order[pixel_order] = order_chars[char_no]
                pixel_order = ''.join(order)

        super().__init__(n, bytearray(self.n * bpp),
                         brightness=brightness,
                         rawbuf=bytearray(self.n * bpp),
                         byteorder=pixel_order,
                         auto_write=auto_write)

        self.pin = digitalio.DigitalInOut(board.D18)
        self.pin.direction = digitalio.Direction.OUTPUT

    def __del__(self):
        self.fill(0)
        self.show()


    def show(self):
        neopixel_write(self.pin, self._buf)

    def fill(self,color):
        _pixelbuf.fill(self,color)
"""

class PixelDisplay(object):
    
    #WS2811 Color type
    RGB = 0x00100800
    RBG = 0x00100008
    GRB = 0x00081000
    GBR = 0x00080010
    BRG = 0x00001008
    BGR = 0x00000810

    #SK6812 Color type 
    WRGB = 0x18100800
    WRBG = 0x18100008
    WGRB = 0x18081000
    WGBR = 0x18080010
    WBRG = 0x18001008
    WBGR = 0x18000810
    
    #Integer Color Value
    RED     = 0x00200000
    ORANGE  = 0x00201000
    YELLOW  = 0x00202000
    GREEN   = 0x00002000
    SKYBLUE = 0x00002020
    BLUE    = 0x00000020
    PURPLE  = 0x00100010
    PINK    = 0x0046000E

    def __init__(self, width=8, height=8, gpio=-1, type=GRB, dma=10, automode=True, debug=False):
        if gpio < 0:
            gpio = pinMap(PIXEL_DISPLAY)

        if debug:
            print("Pixel Display Pin: %d" % gpio)
            
        binder.PixelDisplay_new.argtype = [ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int, ctypes.c_uint8]
        
        self._buf = [[0] * height for i in range(width)]
        self.bind = binder.PixelDisplay_new(width, height, gpio, type, dma, automode)
        self.width = width
        self.height = height
        self.automode = automode
        self.clear()

    def __del__(self):
        self.clear()
        self.display()

    def fill(self, color_arr):        
        for y in range(self.height):
            for x in range(self.width):
                self.setColor(x,y,color_arr)
        
        if(self.automode):
            self.display()

    def clear(self):
        binder.PixelDisplay_clear(self.bind)

    def setColor(self, x, y, color):
        color = self.RGBtoHEX(color)
        self._buf[x][y] = color

        binder.PixelDisplay_setColor.argtype = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
        binder.PixelDisplay_setColor(self.bind, x, y, color)

    def getColor(self, x, y):
        color = self._buf[x][y]
        return self.HEXtoRGB(color)

    def setAutomode(self, automode):
        if type(automode) is not bool:
            raise TypeError("automode must be bool not {%s}".format(type(automode)))
        
        binder.PixelDisplay_setAutomode.argtype = [ctypes.c_uint8]
        binder.PixelDisplay_setAutomode(self.bind, automode)
        self.automode = automode

    def rainbow(self):
        binder.PixelDisplay_rainbow(self.bind)
    
    def setColorInvert(self, invert):
        binder.PixelDisplay_setColorInvert.argtype = [ctypes.c_int]
        binder.PixelDisplay_setColorInvert(self.bind, invert)

    def setBrightness(self, brightness):
        binder.PixelDisplay_setBrightness.argtype = [ctypes.c_int]
        binder.PixelDisplay_setBrightness(self.bind, brightness)

    def display(self):
        binder.PixelDisplay_display(self.bind)
        
    def getRGBType(self):
        return binder.PixelDisplay_getRGBType(self.bind)

    def RGBtoHEX(self, color_arr):
        color = 0
        _type = type(color_arr)

        if _type is list or _type is tuple:
            color_arr = list(color_arr)

            for i in range(3):
                color_arr[i] = 0 if color_arr[i] < 0 else color_arr[i]
                color_arr[i] = 255 if color_arr[i] > 255 else color_arr[i]

                color = color + (color_arr[i] << (16 - i * 8))

        elif _type is int:
            color = color_arr

        else:
            raise TypeError("color must be int or array type not {%s}".format(_type))

        return color

    def HEXtoRGB(self, color):
        color_arr = [0] * 3
        _type = type(color)

        if _type is int:
            color_arr = list(color_arr)
            for i in range(3):
                color_arr[i] = (color >> (16 - i * 8)) & 0xFF
            
        else:
            raise TypeError("color must be int not {}".format(_type))

        return color_arr


if os.path.exists("/dev/video0"):
    class Camera(PopThread):
        def __init__(self, width=cv.VideoCapture(0).get(cv.CAP_PROP_FRAME_WIDTH), height=cv.VideoCapture(0).get(cv.CAP_PROP_FRAME_HEIGHT)):
            super().__init__()
            Util.enable_imshow()
            self._video_capture = cv.VideoCapture(0)
            self._video_capture.set(cv.CAP_PROP_FRAME_WIDTH, width)
            self._video_capture.set(cv.CAP_PROP_FRAME_HEIGHT, height)
            self._width = width
            self._height = height
            self.value = None
            self._show = False
            self.start()

        def run(self):
            while self._state:
                # print(self._state, self._video_capture.isOpened())
                if self._video_capture.isOpened():
                    ret, frame = self._video_capture.read()
                    if ret:
                        self.value = frame
                        if self._show:
                            Util.imshow("image", self.value)

        def show(self):
            self._show = True
            if self.value is not None:
                Util.imshow("image", self.value)

'''
def ledstrip_unittest():
    ledstrip = PixelDisplay()
    for i in range(100):
        if i%4 == 0:
            ledstrip.fill([255,0,0])
        elif i%4 == 1:
            ledstrip.fill([0,255,0])
        elif i%4 == 2:
            ledstrip.fill([0,0,255])
        elif i%4 == 3:
            ledstrip.fill([0,0,0])
        time.sleep(1)
'''
##################################################################
# I2C Guesture Sensor
"""
class ADPS9960InvalidDevId(ValueError):
    def __init__(self, id, valid_ids):
        Exception.__init__(self, "Device id 0x{} is not a valied one (valid: {})!".format(format(id, '02x'), ', '.join(["0x{}".format(format(i, '02x')) for i in valid_ids])))

class ADPS9960InvalidMode(ValueError):
    def __init__(self, mode):
        Exception.__init__(self, "Feature mode {} is invalid!".format(mode))

class Apds9960(I2c):
    APDS9960_ADDR = 0x39
    APDS9960_GESTURE_THRESHOLD_OUT = 10
    APDS9960_GESTURE_SENSITIVITY_1 = 50
    APDS9960_GESTURE_SENSITIVITY_2 = 20
    APDS9960_DEV_ID = [0xab, 0x9c, 0xa8, -0x55]
    APDS9960_TIME_FIFO_PAUSE = 0.03
    APDS9960_REG_ENABLE = 0x80
    APDS9960_REG_ATIME = 0x81
    APDS9960_REG_WTIME = 0x83
    APDS9960_REG_AILTL = 0x84
    APDS9960_REG_AILTH = 0x85
    APDS9960_REG_AIHTL = 0x86
    APDS9960_REG_AIHTH = 0x87
    APDS9960_REG_PILT = 0x89
    APDS9960_REG_PIHT = 0x8b
    APDS9960_REG_PERS = 0x8c
    APDS9960_REG_CONFIG1 = 0x8d
    APDS9960_REG_PPULSE = 0x8e
    APDS9960_REG_CONTROL = 0x8f
    APDS9960_REG_CONFIG2 = 0x90
    APDS9960_REG_ID = 0x92
    APDS9960_REG_STATUS = 0x93
    APDS9960_REG_CDATAL = 0x94
    APDS9960_REG_CDATAH = 0x95
    APDS9960_REG_RDATAL = 0x96
    APDS9960_REG_RDATAH = 0x97
    APDS9960_REG_GDATAL = 0x98
    APDS9960_REG_GDATAH = 0x99
    APDS9960_REG_BDATAL = 0x9a
    APDS9960_REG_BDATAH = 0x9b
    APDS9960_REG_PDATA = 0x9c
    APDS9960_REG_POFFSET_UR = 0x9d
    APDS9960_REG_POFFSET_DL = 0x9e
    APDS9960_REG_CONFIG3 = 0x9f
    APDS9960_REG_GPENTH = 0xa0
    APDS9960_REG_GEXTH = 0xa1
    APDS9960_REG_GCONF1 = 0xa2
    APDS9960_REG_GCONF2 = 0xa3
    APDS9960_REG_GOFFSET_U = 0xa4
    APDS9960_REG_GOFFSET_D = 0xa5
    APDS9960_REG_GOFFSET_L = 0xa7
    APDS9960_REG_GOFFSET_R = 0xa9
    APDS9960_REG_GPULSE = 0xa6
    APDS9960_REG_GCONF3 = 0xaA
    APDS9960_REG_GCONF4 = 0xaB
    APDS9960_REG_GFLVL = 0xae
    APDS9960_REG_GSTATUS = 0xaf
    APDS9960_REG_IFORCE = 0xe4
    APDS9960_REG_PICLEAR = 0xe5
    APDS9960_REG_CICLEAR = 0xe6
    APDS9960_REG_AICLEAR = 0xe7
    APDS9960_REG_GFIFO_U = 0xfc
    APDS9960_REG_GFIFO_D = 0xfd
    APDS9960_REG_GFIFO_L = 0xfe
    APDS9960_REG_GFIFO_R = 0xff
    APDS9960_BIT_PON = 0b00000001
    APDS9960_BIT_AEN = 0b00000010
    APDS9960_BIT_PEN = 0b00000100
    APDS9960_BIT_WEN = 0b00001000
    APDS9960_BIT_AIEN =0b00010000
    APDS9960_BIT_PIEN = 0b00100000
    APDS9960_BIT_GEN = 0b01000000
    APDS9960_BIT_GVALID = 0b00000001
    APDS9960_MODE_POWER = 0
    APDS9960_MODE_AMBIENT_LIGHT = 1
    APDS9960_MODE_PROXIMITY = 2
    APDS9960_MODE_WAIT = 3
    APDS9960_MODE_AMBIENT_LIGHT_INT = 4
    APDS9960_MODE_PROXIMITY_INT = 5
    APDS9960_MODE_GESTURE = 6
    APDS9960_MODE_ALL = 7
    APDS9960_LED_DRIVE_100MA = 0
    APDS9960_LED_DRIVE_50MA = 1
    APDS9960_LED_DRIVE_25MA = 2
    APDS9960_LED_DRIVE_12_5MA = 3
    APDS9960_PGAIN_1X = 0
    APDS9960_PGAIN_2X = 1
    APDS9960_PGAIN_4X = 2
    APDS9960_PGAIN_8X = 3
    APDS9960_AGAIN_1X = 0
    APDS9960_AGAIN_4X = 1
    APDS9960_AGAIN_16X = 2
    APDS9960_AGAIN_64X = 3
    APDS9960_GGAIN_1X = 0
    APDS9960_GGAIN_2X = 1
    APDS9960_GGAIN_4X = 2
    APDS9960_GGAIN_8X = 3
    APDS9960_LED_BOOST_100 = 0
    APDS9960_LED_BOOST_150 = 1
    APDS9960_LED_BOOST_200 = 2
    APDS9960_LED_BOOST_300 = 3
    APDS9960_GWTIME_0MS = 0
    APDS9960_GWTIME_2_8MS = 1
    APDS9960_GWTIME_5_6MS = 2
    APDS9960_GWTIME_8_4MS = 3
    APDS9960_GWTIME_14_0MS = 4
    APDS9960_GWTIME_22_4MS = 5
    APDS9960_GWTIME_30_8MS = 6
    APDS9960_GWTIME_39_2MS = 7
    APDS9960_DEFAULT_ATIME = 219                            
    APDS9960_DEFAULT_WTIME = 246                            
    APDS9960_DEFAULT_PROX_PPULSE = 0x87                     
    APDS9960_DEFAULT_GESTURE_PPULSE = 0x89                  
    APDS9960_DEFAULT_POFFSET_UR = 0                         
    APDS9960_DEFAULT_POFFSET_DL = 0                         
    APDS9960_DEFAULT_CONFIG1 = 0x60                         
    APDS9960_DEFAULT_LDRIVE = APDS9960_LED_DRIVE_100MA
    APDS9960_DEFAULT_PGAIN = APDS9960_PGAIN_4X
    APDS9960_DEFAULT_AGAIN = APDS9960_AGAIN_4X
    APDS9960_DEFAULT_PILT = 0                               
    APDS9960_DEFAULT_PIHT = 50                              
    APDS9960_DEFAULT_AILT = 0xffff                          
    APDS9960_DEFAULT_AIHT = 0
    APDS9960_DEFAULT_PERS = 0x11                            
    APDS9960_DEFAULT_CONFIG2 = 0x01                           
    APDS9960_DEFAULT_CONFIG3 = 0                            
    APDS9960_DEFAULT_GPENTH = 40                            
    APDS9960_DEFAULT_GEXTH = 30                                 
    APDS9960_DEFAULT_GCONF1 = 0x40                          
    APDS9960_DEFAULT_GGAIN = APDS9960_GGAIN_4X
    APDS9960_DEFAULT_GLDRIVE = APDS9960_LED_DRIVE_100MA
    APDS9960_DEFAULT_GWTIME = APDS9960_GWTIME_2_8MS
    APDS9960_DEFAULT_GOFFSET = 0                            
    APDS9960_DEFAULT_GPULSE = 0xc9                          
    APDS9960_DEFAULT_GCONF3 = 0                             
    APDS9960_DEFAULT_GIEN = 0
    APDS9960_STATE_NA = 0
    APDS9960_STATE_NEAR = 1
    APDS9960_STATE_FAR = 2
    APDS9960_STATE_ALL = 3
    DIR_NONE = 0
    DIR_LEFT = 1
    DIR_RIGHT = 2
    DIR_UP = 3
    DIR_DOWN = 4
    DIR_NEAR = 5
    DIR_FAR = 6
    DIR_ALL = 7

    class GestureData(object):
        def __init__(self):
            self.u_data = [0] * 32
            self.d_data = [0] * 32
            self.l_data = [0] * 32
            self.r_data = [0] * 32
            self.index = 0
            self.total_gestures = 0
            self.in_threshold = 0
            self.out_threshold = 0

    def __init__(self, addr=APDS9960_ADDR):
        super().__init__(addr)
        
        self.gesture_ud_delta_ = 0
        self.gesture_lr_delta_ = 0
        self.gesture_ud_count_ = 0
        self.gesture_lr_count_ = 0
        self.gesture_near_count_ = 0
        self.gesture_far_count_ = 0
        self.gesture_state_ = 0
        self.gesture_motion_ = self.DIR_NONE

        self.gesture_data_ = self.GestureData()

        self.dev_id = I2c.readByte(self,self.APDS9960_REG_ID)
        if not self.dev_id in self.APDS9960_DEV_ID:
            raise ADPS9960InvalidDevId(self.dev_id, self.APDS9960_DEV_ID)

        self.setMode(self.APDS9960_MODE_ALL, False)

        I2c.writeByte(self,self.APDS9960_REG_ATIME, self.APDS9960_DEFAULT_ATIME)
        I2c.writeByte(self,self.APDS9960_REG_WTIME, self.APDS9960_DEFAULT_WTIME)
        I2c.writeByte(self,self.APDS9960_REG_PPULSE, self.APDS9960_DEFAULT_PROX_PPULSE)
        I2c.writeByte(self,self.APDS9960_REG_POFFSET_UR, self.APDS9960_DEFAULT_POFFSET_UR)
        I2c.writeByte(self,self.APDS9960_REG_POFFSET_DL, self.APDS9960_DEFAULT_POFFSET_DL)
        I2c.writeByte(self,self.APDS9960_REG_CONFIG1, self.APDS9960_DEFAULT_CONFIG1)
        self.setLEDDrive(self.APDS9960_DEFAULT_LDRIVE)
        self.setProximityGain(self.APDS9960_DEFAULT_PGAIN)
        self.setAmbientLightGain(self.APDS9960_DEFAULT_AGAIN)
        self.setProxIntLowThresh(self.APDS9960_DEFAULT_PILT)
        self.setProxIntHighThresh(self.APDS9960_DEFAULT_PIHT)
        self.setLightIntLowThreshold(self.APDS9960_DEFAULT_AILT)
        self.setLightIntHighThreshold(self.APDS9960_DEFAULT_AIHT)

        I2c.writeByte(self,self.APDS9960_REG_PERS, self.APDS9960_DEFAULT_PERS)
        I2c.writeByte(self,self.APDS9960_REG_CONFIG2, self.APDS9960_DEFAULT_CONFIG2)
        I2c.writeByte(self,self.APDS9960_REG_CONFIG3, self.APDS9960_DEFAULT_CONFIG3)
        
        self.setGestureEnterThresh(self.APDS9960_DEFAULT_GPENTH)
        self.setGestureExitThresh(self.APDS9960_DEFAULT_GEXTH)
        I2c.writeByte(self,self.APDS9960_REG_GCONF1, self.APDS9960_DEFAULT_GCONF1)

        self.setGestureGain(self.APDS9960_DEFAULT_GGAIN)
        self.setGestureLEDDrive(self.APDS9960_DEFAULT_GLDRIVE)
        self.setGestureWaitTime(self.APDS9960_DEFAULT_GWTIME)
        I2c.writeByte(self,self.APDS9960_REG_GOFFSET_U, self.APDS9960_DEFAULT_GOFFSET)
        I2c.writeByte(self,self.APDS9960_REG_GOFFSET_D, self.APDS9960_DEFAULT_GOFFSET)
        I2c.writeByte(self,self.APDS9960_REG_GOFFSET_L, self.APDS9960_DEFAULT_GOFFSET)
        I2c.writeByte(self,self.APDS9960_REG_GOFFSET_R, self.APDS9960_DEFAULT_GOFFSET)
        I2c.writeByte(self,self.APDS9960_REG_GPULSE, self.APDS9960_DEFAULT_GPULSE)
        I2c.writeByte(self,self.APDS9960_REG_GCONF3, self.APDS9960_DEFAULT_GCONF3)
        self.setGestureIntEnable(self.APDS9960_DEFAULT_GIEN)

    def __del__(self):
        super().__del__()
    
    def getMode(self):
        return I2c.readByte(self,self.APDS9960_REG_ENABLE)

    def setMode(self, mode, enable=True):
        reg_val = self.getMode()
        if mode < 0 or mode > self.APDS9960_MODE_ALL:
            raise ADPS9960InvalidMode(mode)
        if mode == self.APDS9960_MODE_ALL:
            if enable:
                reg_val = 0x7f
            else:
                reg_val = 0x00
        else:
            if enable:
                reg_val |= (1 << mode)
            else:
                reg_val &= ~(1 << mode)
        I2c.writeByte(self,self.APDS9960_REG_ENABLE, reg_val)

    def enableLightSensor(self, interrupts=True):       
        self.setAmbientLightGain(self.APDS9960_DEFAULT_AGAIN)
        self.setAmbientLightIntEnable(interrupts)
        self.enablePower()
        self.setMode(self.APDS9960_MODE_AMBIENT_LIGHT, True)

    def disableLightSensor(self):
        self.setAmbientLightIntEnable(False)
        self.setMode(self.APDS9960_MODE_AMBIENT_LIGHT, False)

    def enableProximitySensor(self, interrupts=True):
        self.setProximityGain(self.APDS9960_DEFAULT_PGAIN)
        self.setLEDDrive(self.APDS9960_DEFAULT_LDRIVE)
        self.setProximityIntEnable(interrupts)
        self.enablePower()
        self.setMode(self.APDS9960_MODE_PROXIMITY, True)

    def disableProximitySensor(self):
        self.setProximityIntEnable(False)
        self.setMode(self.APDS9960_MODE_PROXIMITY, False)

    def enableGestureSensor(self, interrupts=True):
        self.resetGestureParameters()
        I2c.writeByte(self,self.APDS9960_REG_WTIME, 0xff)
        I2c.writeByte(self,self.APDS9960_REG_PPULSE, self.APDS9960_DEFAULT_GESTURE_PPULSE)
        self.setLEDBoost(self.APDS9960_LED_BOOST_300)
        self.setGestureIntEnable(interrupts)
        self.setGestureMode(True)
        self.enablePower()
        self.setMode(self.APDS9960_MODE_WAIT, True)
        self.setMode(self.APDS9960_MODE_PROXIMITY, True)
        self.setMode(self.APDS9960_MODE_GESTURE, True)

    def disableGestureSensor(self):
        self.resetGestureParameters()
        self.setGestureIntEnable(False)
        self.setGestureMode(False)
        self.setMode(self.APDS9960_MODE_GESTURE, False)

    def isGestureAvailable(self):
        val = I2c.readByte(self,self.APDS9960_REG_GSTATUS)
        val &= self.APDS9960_BIT_GVALID
        return (val == self.APDS9960_BIT_GVALID)

    def readGesture(self):
        fifo_level = 0
        fifo_data = []
        if not (self.getMode() & 0b01000001) or not self.isGestureAvailable():
            return self.DIR_NONE
        while(self.isGestureAvailable()):
            fifo_level = I2c.readByte(self,self.APDS9960_REG_GFLVL)
            if fifo_level > 0:
                fifo_data = []
                for i in range(0, fifo_level):
                    fifo_data += I2c.readBlock(self,self.APDS9960_REG_GFIFO_U, 4)

                if len(fifo_data) >= 4:
                    for i in range(0, len(fifo_data), 4):
                        self.gesture_data_.u_data[self.gesture_data_.index] = fifo_data[i + 0]
                        self.gesture_data_.d_data[self.gesture_data_.index] = fifo_data[i + 1]
                        self.gesture_data_.l_data[self.gesture_data_.index] = fifo_data[i + 2]
                        self.gesture_data_.r_data[self.gesture_data_.index] = fifo_data[i + 3]
                        self.gesture_data_.index += 1
                        self.gesture_data_.total_gestures += 1
                    if self.processGestureData():
                        if self.decodeGesture():
                            pass
                    self.gesture_data_.index = 0
                    self.gesture_data_.total_gestures = 0
            time.sleep(self.APDS9960_TIME_FIFO_PAUSE)
        time.sleep(self.APDS9960_TIME_FIFO_PAUSE)
        self.decodeGesture()
        motion = self.gesture_motion_
        self.resetGestureParameters()
        return motion

    def enablePower(self):
        self.setMode(self.APDS9960_MODE_POWER, True)

    def disablePower(self):
        self.setMode(self.APDS9960_MODE_POWER, False)

    def readAmbientLight(self):
        l = I2c.readByte(self,self.APDS9960_REG_CDATAL)
        h = I2c.readByte(self,self.APDS9960_REG_CDATAH)
        return l + (h << 8)

    def readRedLight(self):
        l = I2c.readByte(self,self.APDS9960_REG_RDATAL)
        h = I2c.readByte(self,self.APDS9960_REG_RDATAH)
        return l + (h << 8)

    def readGreenLight(self):
        l = I2c.readByte(self,self.APDS9960_REG_GDATAL)
        h = I2c.readByte(self,self.APDS9960_REG_GDATAH)
        return l + (h << 8)

    def readBlueLight(self):
        l = I2c.readByte(self,self.APDS9960_REG_BDATAL)
        h = I2c.readByte(self,self.APDS9960_REG_BDATAH)
        return l + (h << 8)

    def readProximity(self):
        return I2c.readByte(self,self.APDS9960_REG_PDATA)

    def resetGestureParameters(self):
        self.gesture_data_.index = 0
        self.gesture_data_.total_gestures = 0

        self.gesture_ud_delta_ = 0
        self.gesture_lr_delta_ = 0

        self.gesture_ud_count_ = 0
        self.gesture_lr_count_ = 0

        self.gesture_near_count_ = 0
        self.gesture_far_count_ = 0

        self.gesture_state_ = 0
        self.gesture_motion_ = self.DIR_NONE

    def processGestureData(self):
        u_first = 0
        d_first = 0
        l_first = 0
        r_first = 0
        u_last = 0
        d_last = 0
        l_last = 0
        r_last = 0

        if self.gesture_data_.total_gestures <= 4:
            return False

        if self.gesture_data_.total_gestures <= 32 and self.gesture_data_.total_gestures > 0:
            for i in range(0, self.gesture_data_.total_gestures):
                if self.gesture_data_.u_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT and \
                    self.gesture_data_.d_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT and \
                    self.gesture_data_.l_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT and \
                    self.gesture_data_.r_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT:

                    u_first = self.gesture_data_.u_data[i]
                    d_first = self.gesture_data_.d_data[i]
                    l_first = self.gesture_data_.l_data[i]
                    r_first = self.gesture_data_.r_data[i]
                    break

            if u_first == 0 or  d_first == 0 or l_first == 0 or r_first == 0:
                return False

            for i in reversed(range(0, self.gesture_data_.total_gestures)):
                if self.gesture_data_.u_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT and \
                    self.gesture_data_.d_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT and \
                    self.gesture_data_.l_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT and \
                    self.gesture_data_.r_data[i] > self.APDS9960_GESTURE_THRESHOLD_OUT:

                    u_last = self.gesture_data_.u_data[i]
                    d_last = self.gesture_data_.d_data[i]
                    l_last = self.gesture_data_.l_data[i]
                    r_last = self.gesture_data_.r_data[i]
                    break

            ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first)
            lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first)
            ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last)
            lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last)

            ud_delta = ud_ratio_last - ud_ratio_first
            lr_delta = lr_ratio_last - lr_ratio_first

            self.gesture_ud_delta_ += ud_delta
            self.gesture_lr_delta_ += lr_delta

            if self.gesture_ud_delta_ >= self.APDS9960_GESTURE_SENSITIVITY_1:
                self.gesture_ud_count_ = 1
            elif self.gesture_ud_delta_ <= -self.APDS9960_GESTURE_SENSITIVITY_1:
                self.gesture_ud_count_ = -1
            else:
                self.gesture_ud_count_ = 0

            if self.gesture_lr_delta_ >= self.APDS9960_GESTURE_SENSITIVITY_1:
                self.gesture_lr_count_ = 1
            elif self.gesture_lr_delta_ <= -self.APDS9960_GESTURE_SENSITIVITY_1:
                self.gesture_lr_count_ = -1
            else:
                self.gesture_lr_count_ = 0

            if self.gesture_ud_count_ == 0 and self.gesture_lr_count_ == 0:
                if abs(ud_delta) < self.APDS9960_GESTURE_SENSITIVITY_2 and \
                    abs(lr_delta) < self.APDS9960_GESTURE_SENSITIVITY_2:

                    if ud_delta == 0 and lr_delta == 0:
                        self.gesture_near_count_ += 1
                    elif ud_delta != 0 or lr_delta != 0:
                        self.gesture_far_count_ += 1

                    if self.gesture_near_count_ >= 10 and self.gesture_far_count_ >= 2:
                        if ud_delta == 0 and lr_delta == 0:
                            self.gesture_state_ = self.APDS9960_STATE_NEAR
                        elif ud_delta != 0 and lr_delta != 0:
                            self.gesture_state_ = self.APDS9960_STATE_FAR
                        return True
            else:
                if abs(ud_delta) < self.APDS9960_GESTURE_SENSITIVITY_2 and \
                    abs(lr_delta) < self.APDS9960_GESTURE_SENSITIVITY_2:
                        if ud_delta == 0 and lr_delta == 0:
                            self.gesture_near_count_ += 1
                        if self.gesture_near_count_ >= 10:
                            self.gesture_ud_count_ = 0
                            self.gesture_lr_count_ = 0
                            self.gesture_ud_delta_ = 0
                            self.gesture_lr_delta_ = 0
        return False

    def decodeGesture(self):
        if self.gesture_state_ == self.APDS9960_STATE_NEAR:
            self.gesture_motion_ = self.DIR_NEAR
            return True

        if self.gesture_state_ == self.APDS9960_STATE_FAR:
            self.gesture_motion_ = self.DIR_FAR
            return True

        if self.gesture_ud_count_ == -1 and self.gesture_lr_count_ == 0:
            self.gesture_motion_ = self.DIR_UP
        elif self.gesture_ud_count_ == 1 and self.gesture_lr_count_ == 0:
            self.gesture_motion_ = self.DIR_DOWN
        elif self.gesture_ud_count_ == 0 and self.gesture_lr_count_ == 1:
            self.gesture_motion_ = self.DIR_RIGHT
        elif self.gesture_ud_count_ == 0 and self.gesture_lr_count_ == -1:
            self.gesture_motion_ = self.DIR_LEFT
        elif self.gesture_ud_count_ == -1 and self.gesture_lr_count_ == 1:
            if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
                self.gesture_motion_ = self.DIR_UP
            else:
                self.gesture_motion_ = self.DIR_DOWN
        elif self.gesture_ud_count_ == 1 and self.gesture_lr_count_ == -1:
            if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
                self.gesture_motion_ = self.DIR_DOWN
            else:
                self.gesture_motion_ = self.DIR_LEFT
        elif self.gesture_ud_count_ == -1 and self.gesture_lr_count_ == -1:
            if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
                self.gesture_motion_ = self.DIR_UP
            else:
                self.gesture_motion_ = self.DIR_LEFT
        elif self.gesture_ud_count_ == 1 and self.gesture_lr_count_ == 1:
            if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
                self.gesture_motion_ = self.DIR_DOWN
            else:
                self.gesture_motion_ = self.DIR_RIGHT
        else:
            return False
        return True

    def getProxIntLowThresh(self):
        return I2c.readByte(self,self.APDS9960_REG_PILT)

    def setProxIntLowThresh(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_PILT, threshold)

    def getProxIntHighThresh(self):
        return I2c.readByte(self,self.APDS9960_REG_PIHT)

    def setProxIntHighThresh(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_PIHT, threshold)

    def getLEDDrive(self):
        val = I2c.readByte(self,self.APDS9960_REG_CONTROL)
        return (val >> 6) & 0b00000011

    def setLEDDrive(self, drive):
        val = I2c.readByte(self,self.APDS9960_REG_CONTROL)
        drive &= 0b00000011
        drive = drive << 6
        val &= 0b00111111
        val |= drive
        I2c.writeByte(self,self.APDS9960_REG_CONTROL, val)

    def getProximityGain(self):
        val = I2c.readByte(self,self.APDS9960_REG_CONTROL)
        return (val >> 2) & 0b00000011

    def setProximityGain(self, drive):
        val = I2c.readByte(self,self.APDS9960_REG_CONTROL)
        drive &= 0b00000011
        drive = drive << 2
        val &= 0b11110011
        val |= drive
        I2c.writeByte(self,self.APDS9960_REG_CONTROL, val)

    def getAmbientLightGain(self):
        val = I2c.readByte(self,self.APDS9960_REG_CONTROL)
        return (val & 0b00000011)

    def setAmbientLightGain(self, drive):
        val = I2c.readByte(self,self.APDS9960_REG_CONTROL)
        drive &= 0b00000011
        val &= 0b11111100
        val |= drive
        I2c.writeByte(self,self.APDS9960_REG_CONTROL, val)

    def getLEDBoost(self):
        val = I2c.readByte(self,self.APDS9960_REG_CONFIG2)
        return (val >> 4) & 0b00000011

    def setLEDBoost(self, boost):
        val = I2c.readByte(self,self.APDS9960_REG_CONFIG2)
        boost &= 0b00000011
        boost = boost << 4
        val &= 0b11001111
        val |= boost
        I2c.writeByte(self,self.APDS9960_REG_CONFIG2, val)

    def getProxGainCompEnable(self):
        val = I2c.readByte(self,self.APDS9960_REG_CONFIG3)
        val = (val >> 5) & 0b00000001
        return val == 1

    def setProxGainCompEnable(self, enable):
        val = I2c.readByte(self,self.APDS9960_REG_CONFIG3)
        val &= 0b11011111
        if enable:
            val |= 0b00100000
        I2c.writeByte(self,self.APDS9960_REG_CONFIG3, val)

    def getProxPhotoMask(self):
        val = I2c.readByte(self,self.APDS9960_REG_CONFIG3)
        return val & 0b00001111

    def setProxPhotoMask(self, mask):
        val = I2c.readByte(self,self.APDS9960_REG_CONFIG3)
        mask &= 0b00001111
        val &= 0b11110000
        val |= mask
        I2c.writeByte(self,self.APDS9960_REG_CONFIG3, val)

    def getGestureEnterThresh(self):
        return I2c.readByte(self,self.APDS9960_REG_GPENTH)

    def setGestureEnterThresh(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_GPENTH, threshold)

    def getGestureExitThresh(self):
        return I2c.readByte(self,self.APDS9960_REG_GEXTH)

    def setGestureExitThresh(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_GEXTH, threshold)

    def getGestureGain(self):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF2)
        return (val >> 5) & 0b00000011

    def setGestureGain(self, gain):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF2)
        gain &= 0b00000011
        gain = gain << 5
        val &= 0b10011111
        val |= gain
        I2c.writeByte(self,self.APDS9960_REG_GCONF2, val)

    def getGestureLEDDrive(self):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF2)
        return (val >> 3) & 0b00000011

    def setGestureLEDDrive(self, drive):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF2)
        drive &= 0b00000011
        drive = drive << 3
        val &= 0b11100111
        val |= drive
        I2c.writeByte(self,self.APDS9960_REG_GCONF2, val)

    def getGestureWaitTime(self):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF2)
        return val & 0b00000111

    def setGestureWaitTime(self, time):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF2)
        time &= 0b00000111
        val &= 0b11111000
        val |= time
        I2c.writeByte(self,self.APDS9960_REG_GCONF2, val)

    def getLightIntLowThreshold(self):
        return I2c.readByte(self,self.APDS9960_REG_AILTL) | (I2c.readByte(self,self.APDS9960_REG_AILTH) << 8)

    def setLightIntLowThreshold(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_AILTL, threshold & 0x00ff)
        I2c.writeByte(self,self.APDS9960_REG_AILTH, (threshold & 0xff00) >> 8)

    def getLightIntHighThreshold(self):
        return I2c.readByte(self,self.APDS9960_REG_AIHTL) | (I2c.readByte(self,self.APDS9960_REG_AIHTH) << 8)

    def setLightIntHighThreshold(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_AIHTL, threshold & 0x00ff)
        I2c.writeByte(self,self.APDS9960_REG_AIHTH, (threshold & 0xff00) >> 8)

    def getProximityIntLowThreshold(self):
        return I2c.readByte(self,self.APDS9960_REG_PILT)

    def setProximityIntLowThreshold(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_PILT, threshold)

    def getProximityIntHighThreshold(self):
        return I2c.readByte(self,self.APDS9960_REG_PIHT)

    def setProximityIntHighThreshold(self, threshold):
        I2c.writeByte(self,self.APDS9960_REG_PIHT, threshold)

    def getAmbientLightIntEnable(self):
        val = I2c.readByte(self,self.APDS9960_REG_ENABLE)
        return (val >> 4) & 0b00000001 == 1

    def setAmbientLightIntEnable(self, enable):
        val = I2c.readByte(self,self.APDS9960_REG_ENABLE)
        val &= 0b11101111
        if enable:
            val |= 0b00010000
        I2c.writeByte(self,self.APDS9960_REG_ENABLE, val)

    def getProximityIntEnable(self):
        val = I2c.readByte(self,self.APDS9960_REG_ENABLE)
        return (val >> 5) & 0b00000001 == 1

    def setProximityIntEnable(self, enable):
        val = I2c.readByte(self,self.APDS9960_REG_ENABLE)
        val &= 0b11011111
        if enable:
            val |= 0b00100000
        I2c.writeByte(self,self.APDS9960_REG_ENABLE, val)

    def getGestureIntEnable(self):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF4)
        return (val >> 1) & 0b00000001 == 1

    def setGestureIntEnable(self, enable):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF4)
        val &= 0b11111101
        if enable:
            val |= 0b00000010
        I2c.writeByte(self,self.APDS9960_REG_GCONF4, val)

    def clearAmbientLightInt(self):
        I2c.readByte(self,self.APDS9960_REG_AICLEAR)

    def clearProximityInt(self):
        I2c.readByte(self,self.APDS9960_REG_PICLEAR)

    def getGestureMode(self):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF4)
        return val & 0b00000001 == 1

    def setGestureMode(self, enable):
        val = I2c.readByte(self,self.APDS9960_REG_GCONF4)
        val &= 0b11111110
        if enable:
            val |= 0b00000001
        I2c.writeByte(self,self.APDS9960_REG_GCONF4, val)  

def gesture_unittest():
    apds = Apds9960()
    dirs = {
        apds.DIR_NONE: "none",
        apds.DIR_LEFT: "left",
        apds.DIR_RIGHT: "right",
        apds.DIR_UP: "up",
        apds.DIR_DOWN: "down",
        apds.DIR_NEAR: "near",
        apds.DIR_FAR: "far",
    }
    apds.enableGestureSensor()
    while True:
        time.sleep(0.5)
        if apds.isGestureAvailable():
            motion = apds.readGesture()
            print("Gesture={}".format(dirs.get(motion, "unknown"))) 

def proximity_unittest():
    apds = Apds9960()
    apds.setProximityIntLowThreshold(50)
    apds.enableProximitySensor()
    oval = -1
    while True:
        time.sleep(0.25)
        val = apds.readProximity()
        if val != oval:
            print("proximity={}".format(val))
            oval = val

def light_unittest():
    apds = Apds9960(bus)
    print("Light Sensor Test")
    print("=================")
    apds.enableLightSensor()
    oval = -1
    while True:
        sleep(0.25)
        val = apds.readAmbientLight()
        if val != oval:
            print("AmbientLight={}".format(val))
            oval = val
"""

##################################################################
# WiringPi I2C Gesture from binder
APDS9960_ADDR = 0x39

class Apds9960(object):
    DIR_NONE = 0
    DIR_LEFT = 1
    DIR_RIGHT = 2
    DIR_UP = 3
    DIR_DOWN = 4
    DIR_NEAR = 5
    DIR_FAR = 6
    DIR_ALL = 7
    
    def __init__(self, addr=APDS9960_ADDR, debug=False):
        if debug:
            print("Apds9960 Addr: %x" % addr)
        self.bind = binder.Apds9960_new(addr)
    
    def getMode(self):
        return binder.Apds9960_getMode(self.bind)
    def setMode(self, mode, enable=True):
        binder.Apds9960_setMode.argtype = [ctypes.c_int, ctypes.c_int]
        binder.Apds9960_setMode(self.bind, mode, enable)
   
    def enablePower(self):
        binder.Apds9960_enablePower(self.bind)
    def disablePower(self):
        binder.Apds9960_enablePower(self.bind)
    
    def getLEDDrive(self):
        return binder.Apds9960_getLEDDrive(self.bind)
    def setLEDDrive(self, drive):
        binder.Apds9960_setLEDDrive.argtype = [ctypes.c_int]
        binder.Apds9960_setLEDDrive(self.bind, drive)
    def getLEDBoost(self):
        return binder.Apds9960_getLEDBoost(self.bind)
    def setLEDBoost(self, boost):
        binder.Apds9960_setLEDBoost.argtype = [ctypes.c_int]
        binder.Apds9960_setLEDBoost(self.bind, boost)
    
    #Gesture
    def enableGestureSensor(self, interrupts=True):
        binder.Apds9960_enableGestureSensor.argtype = [ctypes.c_int]
        binder.Apds9960_enableGestureSensor(self.bind, interrupts)
    def disableGestureSensor(self):
        binder.Apds9960_disableGestureSensor(self.bind)
    def isGestureAvailable(self):
        return binder.Apds9960_isGestureAvailable(self.bind)
    def readGesture(self):
        return binder.Apds9960_readGesture(self.bind)
    
    def resetGestureParameters(self):
        binder.Apds9960_resetGestureParameters(self.bind)
    def processGestureData(self):
        return binder.Apds9960_processGestureData(self.bind)
    def decodeGesture(self):
        return binder.Apds9960_decodeGesture(self.bind)

    def getGestureEnterThresh(self):
        return binder.Apds9960_getGestureEnterThresh(self.bind)
    def setGestureEnterThresh(self, threshold):
        binder.Apds9960_setGestureEnterThresh.argtype = [ctypes.c_int]
        binder.Apds9960_setGestureEnterThresh(self.bind, threshold)
    def getGestureExitThresh(self):
        return binder.Apds9960_getGestureExitThresh(self.bind)
    def setGestureExitThresh(self, threshold):
        binder.Apds9960_setGestureExitThresh.argtype = [ctypes.c_int]
        binder.Apds9960_setGestureExitThresh(self.bind, threshold)
    def getGestureGain(self):
        return binder.Apds9960_getGestureGain(self.bind)
    def setGestureGain(self, gain):
        binder.Apds9960_setGestureGain.argtype = [ctypes.c_int]
        binder.Apds9960_setGestureGain(self.bind, gain)
    def getGestureLEDDrive(self):
        return binder.Apds9960_getGestureLEDDrive(self.bind)
    def setGestureLEDDrive(self, drive):
        binder.Apds9960_setGestureLEDDrive.argtype = [ctypes.c_int]
        binder.Apds9960_setGestureLEDDrive(self.bind, drive)
    def getGestureWaitTime(self):
        return binder.Apds9960_getGestureWaitTime(self.bind)
    def setGestureWaitTime(self, time):
        binder.Apds9960_setGestureWaitTime.argtype = [ctypes.c_int]
        binder.Apds9960_setGestureWaitTime(self.bind, time)
    
    def getGestureMode(self):
        return binder.Apds9960_getGestureMode(self.bind)
    def setGestureMode(self, enable):
        binder.Apds9960_setGestureMode.argtype = [ctypes.c_int]
        binder.Apds9960_setGestureMode(self.bind, enable)
    def getGestureIntEnable(self):
        return binder.Apds9960_getGestureIntEnable(self.bind)
    def setGestureIntEnable(self, enable):
        binder.Apds9960_setGestureIntEnable.argtype = [ctypes.c_int]
        binder.Apds9960_setGestureIntEnable(self.bind, enable)

    #Proximity
    def enableProximitySensor(self, interrupts=True):
        binder.Apds9960_enableProximitySensor.argtype = [ctypes.c_int]
        binder.Apds9960_enableProximitySensor(self.bind, interrupts)
    def disableProximitySensor(self):
        binder.Apds9960_disableProximitySensor(self.bind)
    def readProximity(self):
        return binder.Apds9960_readProximity(self.bind)
    
    def getProxIntLowThresh(self):
        return binder.Apds9960_getProxIntLowThresh(self.bind)
    def setProxIntLowThresh(self, threshold):
        binder.Apds9960_setProxIntLowThresh.argtype = [ctypes.c_int]
        binder.Apds9960_setProxIntLowThresh(self.bind, threshold)
    def getProxIntHighThresh(self):
        return binder.Apds9960_getProxIntHighThresh(self.bind)
    def setProxIntHighThresh(self, threshold):
        binder.Apds9960_setProxIntHighThresh.argtype = [ctypes.c_int]
        binder.Apds9960_setProxIntHighThresh(self.bind, threshold)
    def getProximityGain(self):
        return binder.Apds9960_getProximityGain(self.bind)
    def setProximityGain(self, drive):
        binder.Apds9960_setProximityGain.argtype = [ctypes.c_int]
        binder.Apds9960_setProximityGain(self.bind, drive)
    def getProxGainCompEnable(self):
        return binder.Apds9960_getProxGainCompEnable(self.bind)
    def setProxGainCompEnable(self, enable):
        binder.Apds9960_setProxGainCompEnable.argtype = [ctypes.c_int]
        binder.Apds9960_setProxGainCompEnable(self.bind, enable)
    def getProxPhotoMask(self):
        return binder.Apds9960_getProxPhotoMask(self.bind)
    def setProxPhotoMask(self, mask):
        binder.Apds9960_setProxPhotoMask.argtype = [ctypes.c_int]
        binder.Apds9960_setProxPhotoMask(self.bind, mask)

    def getProximityIntEnable(self):
        return binder.Apds9960_getProximityIntEnable(self.bind)
    def setProximityIntEnable(self, enable):
        binder.Apds9960_setProximityIntEnable.argtype = [ctypes.c_int]
        binder.Apds9960_setProximityIntEnable(self.bind, enable)
    def getProximityIntLowThreshold(self):
        return binder.Apds9960_getProximityIntLowThreshold(self.bind)
    def setProximityIntLowThreshold(self, threshold):
        binder.Apds9960_setProximityIntLowThreshold.argtype = [ctypes.c_int]
        binder.Apds9960_setProximityIntLowThreshold(self.bind, threshold)
    def getProximityIntHighThreshold(self):
        return binder.Apds9960_getProximityIntHighThreshold(self.bind)
    def setProximityIntHighThreshold(self, threshold):
        binder.Apds9960_setProximityIntHighThreshold.argtype = [ctypes.c_int]
        binder.Apds9960_setProximityIntHighThreshold(self.bind, threshold)
    def clearProximityInt(self):
        binder.Apds9960_clearProximityInt(self.bind)

    #Ambient Light
    def enableLightSensor(self, interrupts=True):
        binder.Apds9960_enableLightSensor.argtype = [ctypes.c_int]
        binder.Apds9960_enableLightSensor(self.bind, interrupts)
    def disableLightSensor(self):
        binder.Apds9960_disableLightSensor(self.bind)
    def readAmbientLight(self):
        return binder.Apds9960_readAmbientLight(self.bind)
    def readRedLight(self):
        return binder.Apds9960_readRedLight(self.bind)
    def readGreenLight(self):
        return binder.Apds9960_readGreenLight(self.bind)
    def readBlueLight(self):
        return binder.Apds9960_readBlueLight(self.bind)

    def getAmbientLightIntEnable(self):
        return binder.Apds9960_getAmbientLightIntEnable(self.bind)
    def setAmbientLightIntEnable(self, enable):
        binder.Apds9960_setAmbientLightIntEnable.argtype = [ctypes.c_int]
        binder.Apds9960_setAmbientLightIntEnable(self.bind, enable)
    def getLightIntLowThreshold(self):
        return binder.Apds9960_getLightIntLowThreshold(self.bind)
    def setLightIntLowThreshold(self, threshold):
        binder.Apds9960_setLightIntLowThreshold.argtype = [ctypes.c_int]
        binder.Apds9960_setLightIntLowThreshold(self.bind, threshold)
    def getLightIntHighThreshold(self):
        return binder.Apds9960_getLightIntHighThreshold(self.bind)
    def setLightIntHighThreshold(self, threshold):
        binder.Apds9960_setLightIntHighThreshold.argtype = [ctypes.c_int]
        binder.Apds9960_setLightIntHighThreshold(self.bind, threshold)
    def clearAmbientLightInt(self):
        binder.Apds9960_clearAmbientLightInt(self.bind)

#Gesture Class for user convenience
GESTURE_FLAG      = 0
GESTURE_GESTURE   = 1
GESTURE_LIGHT     = 2
GESTURE_COLOR     = 2
GESTURE_PROXIMITY = 3

class Gesture(object):

    class Light(object):
        def __init__(self):
            self.gesture = Gesture()
            self.bind = binder.Light_new()
        
        def read(self):
            if GESTURE_FLAG is not GESTURE_LIGHT:
                binder.Light_enable(self.bind)
            return binder.Light_read(self.bind)

    class Color(object):
        def __init__(self):
            self.gesture = Gesture()
            self.bind = binder.Color_new()
        
        def readRed(self):
            if GESTURE_FLAG is not GESTURE_COLOR:
                binder.Color_enable(self.bind)
            return binder.Color_readRed(self.bind)
            
        def readGreen(self):
            if GESTURE_FLAG is not GESTURE_COLOR:
                binder.Color_enable(self.bind)
            return binder.Color_readGreen(self.bind)
            
        def readBlue(self):
            if GESTURE_FLAG is not GESTURE_COLOR:
                binder.Color_enable(self.bind)
            return binder.Color_readBlue(self.bind)

    class Proximity(object):
        def __init__(self):
            self.gesture = Gesture()
            self.bind = binder.Proximity_new()
        
        def read(self):
            if GESTURE_FLAG is not GESTURE_PROXIMITY:
                binder.Proximity_enable(self.bind)
            return binder.Proximity_read(self.bind)

    def __init__(self, addr=APDS9960_ADDR, debug=False):
        if debug:
            print("Gesture Address is %d" % addr)
        
        binder.Gesture_new.argtype = [ctypes.c_int]
        self.bind = binder.Gesture_new(addr)
    
    def isAvailable(self):
        if GESTURE_FLAG is not GESTURE_GESTURE:
            binder.Gesture_enable(self.bind)
        return binder.Gesture_isAvailable(self.bind)

    def read(self):
        return binder.Gesture_read(self.bind)

    def readStr(self):
        dirs = {0 : "None" , 1 : "Left", 2 : "Right", 3 : "Up", 4 : "Down", 5: "Near", 6 : "Far"}
        return dirs[self.read()]

##################################################################
# I2C Oled (SSD1306 -> 128x32 , SSH1106 -> 128x64)
class Oled(object):
    OLED_ADDR = 0x3C
    OLED_NONE_TYPE = 0xFF
    OLED_SSD1306_I2C_128x32 = 0
    OLED_SH1106_I2C_128x64 = 1

    BLACK = 0
    WHITE = 1

    def __init__(self, addr=OLED_ADDR, type=OLED_NONE_TYPE, automode=True):
        if type is 0xFF:
            if board_name is PYC_BASIC:
                type = Oled.OLED_SH1106_I2C_128x64
            elif board_name is AIOT_HOME:
                pass
            elif board_name is ES_101:
                type = Oled.OLED_SSD1306_I2C_128x32

        binder.Oled_new.argtype = [ctypes.c_int, ctypes.c_int8, ctypes.c_int]
        self.bind = binder.Oled_new(addr, type, automode)

        self.type = type
        self.automode = automode
        self.init()
        self.clearDisplay()

    def __del__(self):
        if self.type != Oled.OLED_NONE_TYPE:
            self.clearDisplay()
            self.display()
            binder.Oled_close(self.bind)

    def init(self,type=None):
        if type:
            self.type = type

        binder.Oled_new.argtype = [ctypes.c_int8]
        binder.Oled_init(self.bind, self.type)
    
    def print(self,string):
        binder.Oled_print.argtype = [ctypes.c_char_p]
        string = str(string).encode('utf-8')
        binder.Oled_print(self.bind,string)

    def drawCircle(self, x0, y0, r, color):
        binder.Oled_drawCircle.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_drawCircle(self.bind,x0,y0,r,color)

    def fillCircle(self, x0, y0, r, color):
        binder.Oled_fillCircle.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_fillCircle(self.bind,x0,y0,r,color)

    def drawLine(self, x0, y0, x1, y1, color):
        binder.Oled_drawLine.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_drawLine(self.bind,x0,y0,x1,y1,color)

    def drawRect(self, x, y, w, h, color):
        binder.Oled_drawRect.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_drawRect(self.bind,x,y,w,h,color)

    def fillRect(self,x, y, w, h, color):
        binder.Oled_fillRect.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_fillRect(self.bind,x,y,w,h,color)
        
    def drawVerticalBargraph(self, x, y, w, h, color, percent):
        binder.Oled_drawVerticalBargraph.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16,ctypes.c_uint16]
        binder.Oled_drawVerticalBargraph(self.bind,x,y,w,h,color,percent)

    def drawHorizontalBargraph(self, x, y, w, h, color, percent):
        binder.Oled_drawHorizontalBargraph.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16,ctypes.c_uint16]
        binder.Oled_drawHorizontalBargraph(self.bind,x,y,w,h,color,percent)

    def drawRoundRect(self, x, y, w, h, r, color):
        binder.Oled_drawRoundRect.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_drawRoundRect(self.bind,x,y,w,h,r,color)

    def fillRoundRect(self,x, y, w, h, r, color):
        binder.Oled_fillRoundRect.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_fillRoundRect(self.bind,x,y,w,h,r,color)

    def drawTriangle(self,x0, y0, x1, y1, x2, y2, color):
        binder.Oled_drawTriangle.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_drawTriangle(self.bind,x0,y0,x1,y1,x2,y2,color)

    def fillTriangle(self, x0, y0, x1, y1, x2, y2, color):
        binder.Oled_fillTriangle.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_fillTriangle(self.bind,x0,y0,x1,y1,x2,y2,color)

    def drawChar(self, x, y, c, color, bg, size):
        binder.Oled_drawChar.argtype = [ctypes.c_int16,ctypes.c_int16,c_ubyte,ctypes.c_uint16,ctypes.c_uint16,ctypes.c_uint8]
        binder.Oled_drawChar(self.bind,x,y,c,color,bg,size)

    def drawBitmap(self,x, y, bitmap, w, h, color):
        binder.Oled_drawBitmap.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_char_p,ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        #bitmap = bitmap.encode('utf-8')
        arr = (ctypes.c_uint8*len(bitmap))(*bitmap)
        binder.Oled_drawBitmap(self.bind,x,y,arr,w,h,color)

    def setCursor(self, x, y):
        binder.Oled_setCursor.argtype = [ctypes.c_int16,ctypes.c_int16]
        binder.Oled_setCursor(self.bind,x,y)

    def setTextSize(self, s):
        binder.Oled_setTextSize.argtype = [ctypes.c_int8]
        binder.Oled_setTextSize(self.bind,s)

    def setTextColor(self, c):
        binder.Oled_setTextColor.argtype = [ctypes.c_uint16]
        binder.Oled_setTextColor(self.bind,c)

    def setTextColorWithBg(self, c, b):
        binder.Oled_setTextColorWithBg.argtype = [ctypes.c_uint16,ctypes.c_uint16]
        binder.Oled_setTextColorWithBg(self.bind,c,b)

    def width(self):
        return binder.Oled_width(self.bind)
    
    def height(self):
        return binder.Oled_height(self.bind)

    def drawPixel(self, x, y, color):
        binder.Oled_drawPixel.argtype = [ctypes.c_int16,ctypes.c_int16,ctypes.c_uint16]
        binder.Oled_drawPixel(self.bind,x,y,color)

    def setBrightness(self, Brightness):
        binder.Oled_setBrightness.argtype = [ctypes.c_uint8]
        binder.Oled_setBrightness(self.bind,Brightness)

    def invertDisplay(self, i):
        binder.Oled_invertDisplay.argtype = [ctypes.c_uint8]
        binder.Oled_invertDisplay(self.bind,i)

    def startscrollright(self, start, stop):
        binder.Oled_startscrollright.argtype = [ctypes.c_uint8,ctypes.c_uint8]
        binder.Oled_startscrollright(self.bind,start,stop)

    def startscrollleft(self, start, stop):
        binder.Oled_startscrollleft.argtype = [ctypes.c_uint8,ctypes.c_uint8]
        binder.Oled_startscrollleft(self.bind,start,stop)

    def startscrolldiagright(self, start, stop):
        binder.Oled_startscrolldiagright.argtype = [ctypes.c_uint8,ctypes.c_uint8]
        binder.Oled_startscrolldiagright(self.bind,start,stop)

    def startscrolldiagleft(self, start, stop):
        binder.Oled_startscrolldiagleft.argtype = [ctypes.c_uint8,ctypes.c_uint8]
        binder.Oled_startscrolldiagleft(self.bind,start,stop)

    def stopscroll(self):
        binder.Oled_stopscroll(self.bind)

    def display(self):
        binder.Oled_display(self.bind)

    def clearDisplay(self):
        binder.Oled_clearDisplay(self.bind)
    
    def write(self,c):
        binder.Oled_write.argtype = [ctypes.c_uint8]
        binder.Oled_write(self.bind, c)
    
    def setAutomode(self, automode):
        self.automode = automode
        binder.Oled_setAutomode.argtype = [ctypes.c_uint8]
        binder.Oled_setAutomode(self.bind, self.automode)

def oled_unittest():
    display = Oled()
    display.init(display.OLED_SH1106_I2C_128x64)
    display.clearDisplay()
    display.display()    
    display.clearDisplay()

    for i in range(0,display.width(),4):
        display.drawLine(0, 0, i, display.height()-1, display.WHITE)
        display.display()

    for i in range(0,display.height(),4):
        display.drawLine(0, 0, display.width()-1, i, display.WHITE)
        display.display()
    display.display()

##################################################################
# Audio classes

class Audio:
    def __init__(self, blocking=True, cont=False):
        self.W = None
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.blocking = blocking
        self.cont = cont
        self.isStop = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def stop(self):
        self.isStop = True

    def close(self):
        self.stop()
        time.sleep(.1)

        if self.W != None:
            if type(self.W) is list:
                for w in self.W:
                    w.close()
            else:
                self.W.close()

        if self.stream != None:
            self.stream.stop_stream()
            self.stream.close()
        
        self.p.terminate()


def audio_play(file):
    w = wave.open(file, 'rb')
    data = w.readframes(w.getnframes())
    w.close()

    p = pyaudio.PyAudio()
    s = p.open(format=p.get_format_from_width(w.getsampwidth()), channels=w.getnchannels(), 
                      rate=w.getframerate(), output=True)
    s.write(data)
    s.stop_stream()
    s.close()
    p.terminate()


class AudioPlay(Audio):
    def __init__(self, file, blocking=True, cont=False):
        super().__init__(blocking, cont)

        self.W = wave.open(file, "rb")
        self.stream = self.p.open(format=self.p.get_format_from_width(self.W.getsampwidth()),
            channels=self.W.getnchannels(), rate=self.W.getframerate(), output=True, 
            stream_callback=None if blocking else self._callback)

        if blocking:
            self.data = self.W.readframes(self.W.getnframes())

    def __del__(self):
        super().__del__()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def _callback(self, in_data, frame_count, time_info, status):
        data = self.w.readframes(frame_count)
        if self.cont:
            mod = frame_count - len(data) // self.W.getsampwidth()
            if mod != 0:
                self.W.rewind()
                data += self.W.readframes(mod)

        return (data, pyaudio.paContinue if not self.isStop else pyaudio.paAbort)   

    def run(self):
        self.isStop = False
        if self.blocking:
            self.stream.write(self.data)
        else:
            self.stream.start_stream()

    def isPlay(self):
        return self.stream.is_active()


class AudioPlayList(Audio):
    def __init__(self, files, blocking=True, cont=False):
        super().__init__(blocking, cont)

        self.W = []
        for file in files:
            self.W.append(wave.open(file, "rb"))

        self.stream = self.p.open(format=self.p.get_format_from_width(self.W[0].getsampwidth()),
            channels=self.W[0].getnchannels(), rate=self.W[0].getframerate(), output=True, 
            stream_callback=None if blocking else self._callback)

        self.data = []
        if blocking:
            for w in self.W:
                self.data.append(w.readframes(w.getnframes()))

    def _callback(self, in_data, frame_count, time_info, status):
        data = self.W[self.pos].readframes(frame_count)
        if self.cont:
            mod = frame_count - len(data) // self.W[self.pos].getsampwidth()
            if mod != 0:
                self.pos += 1
                if self.pos >= len(self.W):
                    self.pos = 0

                self.W[self.pos].rewind()
                data += self.W[self.pos].readframes(mod)

        return (data, pyaudio.paContinue if not self.isStop else pyaudio.paAbort)   

    def run(self, pos=0):
        self.isStop = False
        self.pos = pos

        if self.blocking:
            self.stream.write(self.data[pos])            
        else:
            self.stream.start_stream()
    
    def isPlay(self):
        return self.stream.is_active()


class AudioRecord(Audio):
    def __init__(self, file, sFormat=8, sChannel=1, sRate=48000, sFramePerBuffer=1024):
        super().__init__(False)

        self.w = wave.open(file, "wb")
        self.w.setsampwidth(self.p.get_sample_size(sFormat))
        self.w.setnchannels(sChannel)
        self.w.setframerate(sRate)

        self.stream = self.p.open(format=sFormat, channels=sChannel, rate=sRate, input=True, 
            frames_per_buffer=sFramePerBuffer, stream_callback=self._callback)

    def __del__(self):
        super().__del__()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def _callback(self, in_data, frame_count, time_info, status):
        self.w.writeframes(in_data)
        data = chr(0) * len(in_data)

        return (data, pyaudio.paContinue if not self.isStop else pyaudio.paAbort)   

    def run(self):
        self.stream.start_stream()


class Tone:
    def __init__(self, tempo=100, volume=.5, rate=48000, channels=1): 
        self.tempo = tempo
        self.volume = volume
        self.rate = rate
        self.channels = channels
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paFloat32, channels=self.channels, rate=self.rate, output=True)

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.stop()

    def close(self):
        self.stream.stop_stream()
        self.stream.close() 
        self.p.terminate()

    def setTempo(self, tempo):
        self.tempo = tempo

    def rest(self, duration):
        self.play(0, "REST", 1/4)

    def play(self, octave, pitch, duration):
        """
        octave = 1 ~ 8
        note = DO, RE, MI, FA, SOL, RA, SI
        dulation = 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, ...
        """
        string_to_pitch = {"REST":0, "DO":1, "DO#":2, "RE":3, "RE#":4, "MI":5, "FA":6, "FA#":7, "SOL":8, "SOL#":9, "RA":10, "RA#":11, "SI":12}

        p = string_to_pitch[pitch]
        f = 2**(octave) * 55 * 2**((p - 10) / 12)
        
        if p == 0:
            time.sleep((60.0 / self.tempo) * (duration * 4))
        else:
            sample = (np.sin(2 * np.pi * np.arange(self.rate * (60.0 / self.tempo * 4) * (duration*4)) * f / self.rate)).astype(np.float32)
            self.stream.write(self.volume * sample)

            delay(20)


class SoundMeter:
    def __init__(self, sampleFormat=pyaudio.paInt16, channelNums=1, framesPerBuffer=1024, sampleRate=48000):
        self.func = None
        self.args = None
        self.isStop = False

        self.p = pyaudio.PyAudio()

        self.stream = self.p.open(format=sampleFormat, channels=channelNums, rate=sampleRate, input=True, 
                                  frames_per_buffer=framesPerBuffer, stream_callback=self._callback)
    
    def __del__(self):
        self.stop()

    def setCallback(self, func, *args):
        self.func = func
        self.args = args
        self.stream.start_stream()

    def stop(self):
        self.isStop = True
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

    def _callback(self, inData, frameCount, timeInfo, status):
        rms = audioop.rms(inData, 2)
        self.func(rms, inData, *self.args)

        data = chr(0) * len(inData)
        return (data, pyaudio.paContinue if not self.isStop else pyaudio.paAbort)

##################################################################
from random import *

if __name__ == "__main__":
    pass
    #popmultitask_unittest()
    #led_unittest()
    #laser_unittest()
    #buzzer_unittest()
    #relay_unittest1()
    #relay_unittest2()
    #ledex_unittest1()
    #ledex_unittest2()
    #rgbled_unittest1()
    #rgbled_unittest2()
    #rgbled_unittest3()
    #dcmotor_unittest1()
    #dcmotor_unittest2()
    #stepmotor_unittest1()
    #stepmotor_unittest2()
    #stepmotor_unittest3()
    #switch_unittest1()
    #switch_unittest2()
    #switch_unittest3()
    #touch_unittest1()
    #touch_unittest2()
    #touch_unittest3()
    #reed_unittest1()
    #reed_unittest2()
    #reed_unittest3()
    #limitswitch_unittest1()
    #limitswitch_unittest2()
    #limitswitch_unittest1()
    #limitswitch_unittest2()
    #limitswitch_unittest3()
    #mercury_unittest1()
    #mercury_unittest2()
    #mercury_unittest3()
    #knock_unittest1()
    #knock_unittest2()
    #knock_unittest3()
    #tilt_unittest1()
    #tilt_unittest2()
    #tilt_unittest3()
    #shock_unittest1()
    #shock_unittest2()
    #shock_unittest3()
    #opto_unittest1()
    #opto_unittest2()
    #opto_unittest3()
    #pir_unittest1()
    #pir_unittest2()
    #pir_unittest3()
    #flame_unittest1()
    #flame_unittest2()
    #flame_unittest3()
    #linetrace_unittest1()
    #linetrace_unittest2()
    #linetrace_unittest3()
    #temphumi_unittest1()
    #temphumi_unittest2()
    #ultrasonic_unittest1()
    #ultrasonic_unittest2()
    #ultrasonic_unittest3()
    #spiadc_unittest1()
    #spiadc_unittest2()
    #spiadc_unittest3()
    #spiadc_unittest4()
    #spiadc_unittest5()
    #shock2_unittest1()
    #shock2_unittest2()
    #shock2_unittest3()
    #sound_unittest1()
    #sound_unittest2()
    #sound_unittest3()
    #potentiometer_unittest1()
    #potentiometer_unittest2()
    #potentiometer_unittest3()
    #potentiometer_unittest4()
    #potentiometer_unittest5()
    #cds_unittest1()
    #cds_unittest2()
    #cds_unittest3()
    #cds_unittest4()
    #soilmoisture_unittest1()
    #soilmoisture_unittest2()
    #soilmoisture_unittest3()
    #thermistor_unittest1()
    #thermistor_unittest2()
    #thermistor_unittest3()
    #thermistor_unittest4()
    #temperature_unittest1()
    #temperature_unittest2()
    #temperature_unittest3()
    #gas_unittest1()
    #gas_unittest2()
    #gas_unittest3()
    #gas_unittest4()
    #dust_unittest1()
    #dust_unittest2()
    #dust_unittest3()
    #psd_unittest1()
    #psd_unittest2()
    #psd_unittest3()
    #psd_unittest4()
    #shiftRegister_unittest()
    #i2c_unittest()
    #At42qt1070_unittest()
    #sht20_unittest()
    #pca9685_unittest()
    #mpu6050_unittest()
    #fan_unittest()
    #textlcd_unittest()
    #ledstrip_unittest()
