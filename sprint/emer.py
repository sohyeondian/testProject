from pop import Leds, PiezoBuzzer, Oled, PopThread
from send import Send
import time

class Emergency(PopThread):
    def __init__(self, company, location, addr="192.168.101.101"):
        super().__init__()
        self.__send = Send(company, location, "EMERGENCY", addr)
        self.__send.set_message("EMERGENCY, HELP")
        self.__leds = Leds()
        self.__oled = Oled()
        self.__pb = PiezoBuzzer()

    def start(self):
        super().start()
        self.__send.send()
    
    def run(self):
        self.__oled.clearDisplay()
        self.__oled.setCursor(40,25)
        self.__leds.allOff()
        time.sleep(.5)
        self.__leds.allOn()
        self.__oled.print("WARNING")
        self.__pb.tone(3,2,1)
        time.sleep(.5)
    
    def stop(self):
        self.__leds.allOff()
        self.__oled.clearDisplay()
        super().stop()