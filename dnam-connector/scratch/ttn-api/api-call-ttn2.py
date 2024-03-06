## Connector v2 with csv parsed data

import paho.mqtt.client as mqtt
import ssl
import json
import base64
import os

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
    payload = json.loads(msg.payload.decode())  # Decode JSON payload
    
    # Extract required information
    time = payload["received_at"]
    device_id = payload["end_device_ids"]["device_id"]
    rssi = payload["uplink_message"]["rx_metadata"][0]["rssi"]
    snr = payload["uplink_message"]["rx_metadata"][0]["snr"]
    frm_payload_encoded = payload["uplink_message"]["frm_payload"]
    frm_payload = base64.b64decode(frm_payload_encoded).decode('utf-8')  # Decode frm_payload from Base64
    
    # Prepare the data string to write to the file
    data_string = f"{time};{device_id};{rssi};{snr};{frm_payload}\n"
    
    # Check if file exists and write header if necessary
    file_exists = os.path.isfile("device_payloads.csv")
    with open("device_payloads.csv", "a") as file:
        if not file_exists:
            file.write("date;device-id;rssi;snr;payload\n")
        file.write(data_string)

# Initialize the MQTT Client
client = mqtt.Client()
client.username_pw_set(username, password)
client.tls_set(tls_version=ssl.PROTOCOL_TLS)
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT Broker and start the loop
try:
    client.connect(broker_address, port, 60)
    client.loop_forever()
except Exception as e:
    print(f"An error occurred: {e}")
