import paho.mqtt.client as mqtt

def publish_message():
    data = input("Enter Msg : ")
    client.publish("SODA/hello/test", data)

def do_connect(client, userdata, flags, rc):
    print("connected")
    publish_message()
    
    
def do_publish(client, userdata, mid):
    print("ok publish")
    publish_message()

    

print("<Instans Names>")
print('''leds = Leds()
pb = PiezoBuzzer()
ges = Gesture()
oled = Oled()
sht = Sht20()''')

client = mqtt.Client()

client.on_connect = do_connect
client.on_publish = do_publish
client.connect("192.168.101.101")
client.loop_forever()
