import paho.mqtt.client as mqtt
import threading
import json

def publish_message(client):
    info = {'sensor': None, 'action': None, 'result': None}
    
    info['sensor'] = input("Sensor: ")
    info['action'] = input("Action: ")
    client.publish("SODA/hello/test", json.dumps(info))

def do_connect(client, usrdata, flags, rc):
    if rc == 0:
        print("ok connect")
        client.subscribe("SODA/hello/test") 
        publish_message(client)
        #threading.Thread(target=publish_message, args=(client,)).start()
    else:
        print("not connect")

def do_publish(client, userdata, mid):
    threading.Thread(target=publish_message, args=(client,)).start()
    
def do_message(client, usrdata, message):
    global buf
    info = json.loads(message.payload)
    if buf != info:
        print("\n[Result]", info['result'])
        buf = info

buf = []

def main():
    client = mqtt.Client() 
    client.on_connect = do_connect
    client.on_publish = do_publish
    client.on_message = do_message
    client.connect("192.168.101.101")  #실제 브로커 주소 사용
    client.loop_forever()

if __name__ == "__main__":
    main()