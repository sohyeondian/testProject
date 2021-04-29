import paho.mqtt.client as mqtt
import time

def do_connect(client, userdata, flags, rc):
    print("connected")
    client.subscribe("SODA/+/EMERGENCY")

def do_message(client, userdata, message):
    date = time.strftime('%Y-%m-%d %X',time.localtime(time.time()))
    topic = message.topic.split("/")
    # Topic [회사이름]/[공장ID]/EMERGENCY
    print(f"[{date}] {topic[1]} : {message.payload.decode()}")

client = mqtt.Client()

client.on_connect = do_connect
client.on_message = do_message

client.connect("192.168.101.101")

client.loop_forever()