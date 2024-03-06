## connector v1 just catch json data of mqtt

import paho.mqtt.client as mqtt # pip3 install "paho-mqtt<2.0.0"
import ssl

# MQTT Credentials and Connection Parameters
broker_address = "eu1.cloud.thethings.network"
port = 8883
username = "dnam-ttn-01@ttn"
password = "NNSXS.CKZDJWMSSA7BLUG6IGSIEZ2DQBN54ASO22JFYYI.5HMGHOOPV5WZLN6RCAB7FA7CWWEBAG76YUSZAYOJCFELJRJH6M4A"
application_id = "dnam-ttn-01"
device_id = "eui-70b3d57ed00633b8"
topic = f"v3/{application_id}@ttn/devices/{device_id}/up"

# Callback when connecting to the MQTT server
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(topic)

# Callback when receiving a message from the subscribed topic
def on_message(client, userdata, msg):
    payload_hex = msg.payload.decode()
    print(f"Received payload: {payload_hex}")
    # Assuming the payload is already in Hex and writing it to a file
    with open("device_payloads1.txt", "a") as file:
        file.write(payload_hex + "\n")

client = mqtt.Client()
client.username_pw_set(username, password)
client.tls_set(tls_version=ssl.PROTOCOL_TLS)
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(broker_address, port, 60)
    # Blocking call that processes network traffic, dispatches callbacks and handles reconnecting.
    client.loop_forever()
except Exception as e:
    print(f"An error occurred: {e}")