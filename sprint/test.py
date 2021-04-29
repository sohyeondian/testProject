from pop import Cds, Sht20, PixelDisplay, Sound
from emer import Emergency
from factory import Factory
import BlynkLib
import time

addr = "192.168.101.101"
blynk = BlynkLib.Blynk("B7xckwtUCyE9ga7ZlUnNUl1az2sK93l9", server="127.0.0.1", port="8080")

def emergency():
    f.stop()
    e.start()

@blynk.VIRTUAL_WRITE(0)
def on_emergency(on):
    if int(on[0]):
        emergency()
    else:
        e.stop()
        f.start()


def check_tmp_humi():
    tmp = sht.readTemp()
    hum = sht.readHumi()
    if tmp > 40:
        emergency()
        return
    return (tmp, hum)

def set_lcd_msg():
    th = check_tmp_humi()
    if e.isRun():
        msg = ['EMERGENCY', "HELP"]
    else:
        msg = [f"TEMP : {th[0]:.2f}°C" , f"HUMI : {th[1]:.2f}%"]
    return msg

@blynk.VIRTUAL_READ(1)
def on_sht():
    msg = set_lcd_msg()
    blynk.virtual_write(1, msg[0])
    blynk.virtual_write(2, msg[1])


def check_cds():
    return cds.readAverage()

@blynk.VIRTUAL_READ(3)
def on_cds_average():
    cd_val = check_cds()
    blynk.virtual_write(3, cd_val)


def check_sound():
    val = sound.readAverage()
    return abs(val - (4096//2 + 12))

@blynk.VIRTUAL_READ(4)
def on_sound():
    sd_ret = check_sound()
    if sd_ret > 10:
        blynk.virtual_write(4, sd_ret)


if __name__ == '__main__':

    sht = Sht20()
    sound = Sound()
    cds = Cds()
    e = Emergency("SODA", "Gongju")
    f = Factory()
    f.start()

    while True:
        blynk.run()

            