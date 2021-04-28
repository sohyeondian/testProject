from xpop import Leds, PiezoBuzzer, Gesture, Oled, Sht20
import time

def menu():
    print('1. leds - all leds ON & OFF')
    print('2. buzzer - beep-sound')
    print('3. Gesture - check up down left right')
    print('4. Oled - print HELLO')
    print('5. Temp & Humid')
    print('6. Print status log')
    print('0. QUIT')
    print('==========================')

leds = Leds()
pb = PiezoBuzzer()
ges = Gesture()
oled = Oled()
sht = Sht20()

menu()

while True:
    num = input('입력 : ')
    if num in ['0', 'quit', 'q']:
        break
    elif num == '1':
        leds.allOn()
        time.sleep(3)
        leds.allOff()
        for i in range(8):
            leds[i].on()
            time.sleep(.5)
            leds[i].off()

    elif num == '2':
        pb.setTempo(120)
        pb.tone(4, 8, 4)
    elif num == '3':
        for _ in range(100):
            if ges.isAvailable():
                m = ges.readStr()
                print(m)
    elif num == '4':
        oled.setCursor(10,30)
        oled.print("Hello :D")
        time.sleep(5)
        oled.clearDisplay()
    elif num == '5':
        t = sht.readTemp()
        h = sht.readHumi()
        print(f"Temp: {t:.2f}, Humi : {h:.2f}%")
    elif num == '6':
        with open("status_logs.bin", "rb") as f:
            for line in f.readlines():
                print(line.decode(), end = "")
    else:
        print('잘못입력하셨습니다.')