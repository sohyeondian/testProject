from pop import Leds, PiezoBuzzer, Gesture, Oled, Sht20
import paho.mqtt.client as mqtt
import time

def do_connect(client, userdata, flags, rc):
    print("connected")
    client.subscribe("SODA/hello/test")
    

def do_message(client, userdata, message):
    data = message.payload.decode()
    print(f"[{message.topic}] {data}")
    msg = ''
    try:
        ret = eval(data)
        if ret != None:
            msg = ret
    except:
        msg = "Error : "+ data
        
    print(msg)
        

leds = Leds()
pb = PiezoBuzzer()
ges = Gesture()
oled = Oled()
sht = Sht20()

client = mqtt.Client()

client.on_connect = do_connect
client.on_message = do_message
client.connect("192.168.101.101")

client.loop_forever()