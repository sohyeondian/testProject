import paho.mqtt.client as mqtt

class Send():
    def __init__(self, company, location, topic, addr):
        self.client = mqtt.Client()
        self.client.connect(addr)
        self.company = company
        self.location = location
        self.topic = topic
    
    def set_topic(self, topic):
        self.topic = topic
    
    def set_message(self, msg):
        self.msg = msg
    
    def send(self):
        self.client.publish(self.company+"/"+self.location+"/"+self.topic, self.msg)