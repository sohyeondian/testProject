import BlynkLib
from pop import Cds, Leds, PiezoBuzzer, Sht20, Potentiometer, Oled, Gesture, PixelDisplay, Sound
import time

blynk = BlynkLib.Blynk("S1CjE5juP0ZofIId7ctHDnjDkXx0rJSi", server="127.0.0.1", port="8080")

@blynk.VIRTUAL_READ(0)
def on_cds_average():
    cds = Cds()
    val = cds.readAverage()
    blynk.virtual_write(0, val)


@blynk.VIRTUAL_WRITE(1)
def on_sht(on):
    sht = Sht20()
    tmp = sht.readTemp()
    hum = sht.readHumi()
    blynk.virtual_write(2, tmp)
    blynk.virtual_write(3, hum)

    
@blynk.VIRTUAL_READ(4)
def on_potentiometer():
    poten = Potentiometer()
    poten.setRangeTable([144, 629, 1112, 1621, 2085, 2642, 3158, 3590, 3992, 4094])
    val = poten.readAverage()
    blynk.virtual_write(4, val)


@blynk.VIRTUAL_WRITE(5)
def on_leds_ctl(on):
    leds = Leds()
    # list 문자열로 나옴 ['0']
    if int(on[0]):
        leds.allOn()
    else:
        leds.allOff()


@blynk.VIRTUAL_WRITE(6)
def on_oled_display(msg):
    oled.print(str(msg[0]))


@blynk.VIRTUAL_WRITE(7)
def on_all_clear(on):
    oled.clearDisplay()
    pixel.clear()


@blynk.VIRTUAL_READ(8)
def on_gesture():
    if ges.isAvailable():
        msg = ges.readStr()
        blynk.virtual_write(8, msg)


class MyPixels(PixelDisplay):
    def __init__(self):
        super().__init__()
        self.x = 0
        self.y = 0
        self.rgb = [0, 0, 0]
        super().setBrightness(20)
    
    def set_pix_color(self, rgb):
        self.rgb = [int(rgb[0]), int(rgb[1]), int(rgb[2])]

    def set_pix_x(self, index):
        self.x = index
    
    def set_pix_y(self, index):
        self.y = index

    def set_pixcel_color(self):
        super().setColor(self.x, self.y, self.rgb)
    
    def set_all_pix_on(self):
        super().fill(self.rgb)

    def set_all_pix_off(self):
        super().clear()
    

@blynk.VIRTUAL_WRITE(9)
def on_pix_color(rgb):
    rgb = [int(rgb[0]), int(rgb[1]), int(rgb[2])]
    pixel.set_pix_color(rgb)

@blynk.VIRTUAL_WRITE(10)
def on_pix_y(index):
    pixel.set_pix_x(int(index[0]))

@blynk.VIRTUAL_WRITE(11)
def on_pix_y(index):
    pixel.set_pix_y(int(index[0]))    

@blynk.VIRTUAL_WRITE(12)
def on_pixcel(on):
    pixel.set_pixcel_color()

@blynk.VIRTUAL_WRITE(13)
def on_pixcel_all(on):
    if int(on[0]):
        pixel.set_all_pix_on()
    else:
        pixel.set_all_pix_off()


@blynk.VIRTUAL_READ(14)
def on_sound():
    sound = Sound()
    val = sound.readAverage()
    ret = abs(val - (4096//2 + 12))
    if ret > 10:
        blynk.virtual_write(14, ret)


@blynk.VIRTUAL_WRITE(15)
def on_buzzer(on):
    buz = PiezoBuzzer()
    buz.setTempo(120)
    buz.tone(4, 8, 4)


@blynk.VIRTUAL_WRITE(16)
def on_leds_ctl(on):
    for x in range(8):
        leds = Leds()
        leds[x].on()
        time.sleep(.5)
        leds[x].off()


@blynk.VIRTUAL_READ(17)
def on_switches():
    pass


if __name__ == '__main__':
    
    ges = Gesture()
    oled = Oled()
    pixel = MyPixels()
    
    
    while True:
        blynk.run()
    