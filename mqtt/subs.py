import paho.mqtt.client as mqtt

def do_connect(client, userdata, flags, rc):
    print("connected")
    client.subscribe("SODA/hello/test")


def do_message(client, userdata, message):
    print(f"[{message.topic}] {message.payload.decode()}")

client = mqtt.Client()

client.on_connect = do_connect
client.on_message = do_message

client.connect("192.168.101.101")

client.loop_forever()