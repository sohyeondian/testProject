import paho.mqtt.client as mqtt
from pop import Leds, PiezoBuzzer, Gesture, Oled, Sht20, Psd, Cds
from pop import Switch, Switches, PixelDisplay, Potentiometer, Sound
from pop import PopThread
import json
import time

def do_connect(client, usrdata, flags, rc):
    if rc == 0:
        print("ok connect")
        client.subscribe("SODA/hello/test") 
    else:
        print("not connect")


def do_message(client, usrdata, message):
    global buf
    info = json.loads(message.payload)
    if buf != info:
        try:
            ret = eval(info['command'])
            if ret != None:
                info['result'] = ret
                print(info['result'])
        except:
            info['result'] = "error"
        client.publish("SODA/hello/test", json.dumps(info))
        buf = info


def on_psd(psd):
    def wrapper(val):
        global result
        res = str(psd.calcDist(val))+"cm"
        print(res)
        result.append(res)
        time.sleep(.1)
    return wrapper

def psd_test():
    global result
    psd = Psd()
    result = []

    psd.setCallback(on_psd(psd))
    time.sleep(.5)  
    psd.stop()
    return result


class MyGesture(PopThread):
    def __init__(self):
        self.__gesture = Gesture()
        
    def run(self):
        if self.__gesture.isAvailable():
            global result
            res = self.__gesture.readStr()
            result.append(res)
            print(res)

def gesture_test():
    global result
    result = []
    gesture = MyGesture()
    
    gesture.start()
    time.sleep(3)   
    gesture.stop()
    return result


def pixel_display_test():
    pixel = PixelDisplay()
    
    pixel.setBrightness(20)
    pixel.fill([255,0,0])
    time.sleep(1)
    
    pixel.setBrightness(50)
    for y in range(8):
        for x in range(8):
            pixel.setColor(x, y, [255,255,0])
            time.sleep(.1)
    
    pixel.rainbow()
    pixel.clear()


# 이거 콜백 종료하는 방법을 모르겠다. 
def on_switch(sw):
    global result
    msg = "sw2 press" if sw.read() else "sw2 unpress"
    print(msg)
    result.append(msg)

def switchs_test():
    global result
    result = []
    sw1 = Switch(7, True) # Bug (disable interrupted)
    sw2 = Switch(27, True)
    sw2.setCallback(on_switch, sw2)
    
    sw1_old = False
    while True:
        try:
            ret = sw1.read()
            if ret and sw2.read():  # 동시에 누르면 탈출
                break
            if sw1_old != ret:
                sw1_old = ret
                msg = "sw1 press" if ret else "sw1 unpress"
                print(msg)
                result.append(msg)
            time.sleep(.1)
            
        except KeyboardInterrupt:
            break
    return result


# 이거 콜백 종료하는 방법을 모르겠다. 멈추지 않아서 주석처리 함
# def on_potentiometer(val):
#     global result
#     res ="%d"%(val)
#     print(res)
#     result.append(res)
#     time.sleep(.5)

def potentiometer_test():
    global result
    result = []
    poten = Potentiometer()
    poten.setRangeTable([144, 629, 1112, 1621, 2085, 2642, 3158, 3590, 3992, 4094])

    # poten.setCallback(on_potentiometer)
    # input("Press <ENTER> key...\n")  
    # poten.stop()
    return poten.readAverage()


def on_sound(val):
    global result
    ret = abs(val - (4096//2 + 12))
    if ret > 10:
        print(ret)
        result.append(ret)
    
def sound_test():
    global result
    result = []
    sound = Sound()
    
    sound.setCallback(on_sound, type=Sound.TYPE_AVERAGE)
    time.sleep(3)
    sound.stop()
    return result


def on_cds(val):
    global result
    result.append(val)

def cds_test():
    global result
    result = []
    cds = Cds()
    
    cds.setCallback(on_cds, type=Cds.TYPE_AVERAGE)
    time.sleep(1)
    cds.stop()
    return result


leds = Leds()
pb = PiezoBuzzer()
oled = Oled()
sht = Sht20()
buf = None


def main():
    client = mqtt.Client()
    client.on_connect = do_connect
    client.on_message = do_message
    client.connect("192.168.101.101") 
    client.loop_forever()

if __name__ == "__main__":
    main()