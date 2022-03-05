#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import random
from paho.mqtt import client as mqtt_client


broker = 'damoa.io'
port = 1883
topic = "s2m/#"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
# username = 'emqx'
# password = 'public'

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        if not msg.topic.endswith('data'):
            print(f"`{msg.topic}`  `{msg.payload.decode()}`")

    client.subscribe(topic)
    client.on_message = on_message
    print("Subscribed to {}.".format(topic))

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker.")
            subscribe(client)
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    #client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def run():
    client = connect_mqtt()
    client.loop_forever()

if __name__ == '__main__':
    run()


# In[2]:


get_ipython().system('pip install paho-mqtt')


# In[ ]:




