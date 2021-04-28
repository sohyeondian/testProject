import pop
import time

def caller(func):
    def wrapper(*args, **kwargs):
        date = time.strftime('%Y-%m-%d %X',time.localtime(time.time()))
        ret = func(*args)
        data = "[%s]%s(%s)->%r\n"%(date, func.__name__,",".join(repr(arg) for arg in args), ret)
        with open("status_logs.bin", "ab") as f:
            f.write(data.encode())
        return ret
    return wrapper


class Out(pop.Out):    
    @caller
    def on(self):
        super().on()
    
    @caller
    def off(self):
        super().off()


class Led(Out, pop.Led):
    def blink(self, period, second):
        super().blink(period, second)


class Leds(pop.Leds):
    # _leds = list()

    # def __init__(self, led=None, debug=False):
    #     self._max = pop.pinMax(pop.LED)
        
    #     for i in range(self._max):
    #         pin = pop.pinMap(pop.LED, i)

    #         if pin is 0xFF:
    #             _leds = [None]
    #             break

    #         else:
    #             #if debug:
    #                 #print("Led Pin: %d" % pin)
    #             self._leds.append(Led(pin))

    @caller
    def allOn(self):
        super().allOn()
    
    @caller
    def allOff(self):
        super().allOff()


class PiezoBuzzer(pop.PiezoBuzzer):
    @caller
    def setTempo(self, n):
        super().setTempo(n)
    
    @caller
    def tone(self, scale, pitch, duration):
        super().tone(scale,pitch,duration)


class Gesture(pop.Gesture):
    @caller
    def readStr(self):
        return super().readStr()

    @caller
    def read(self):
        return super().read()


class Oled(pop.Oled):
    @caller
    def print(self, string):
        super().print(string)


class Sht20(pop.Sht20):
    @caller
    def readTemp(self):
        return super().readTemp()
    
    @caller
    def readHumi(self):
        return super().readHumi()