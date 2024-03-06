## TTN Connector one-way with formatting of data received

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
    try:
        payload = json.loads(msg.payload.decode())  # JSON-Payload dekodieren

        # Überprüfen, ob die Nachricht die Mindestanforderungen erfüllt
        if "uplink_message" not in payload or "frm_payload" not in payload["uplink_message"]:
            print("Nachricht ignoriert: Nicht die erforderliche Struktur.")
            return  # Frühzeitige Rückkehr, um die Verarbeitung zu stoppen

        # Extrahiere die notwendigen Informationen aus dem Payload
        time = payload.get("received_at", "Unbekanntes Datum")
        device_info = payload.get("end_device_ids", {})
        device_id = device_info.get("device_id", "Unbekannte Geräte-ID")
        
        rx_metadata = payload["uplink_message"].get("rx_metadata", [])
        if rx_metadata:  # Prüfe, ob rx_metadata vorhanden und nicht leer ist
            rssi = rx_metadata[0].get("rssi", "Unbekannter RSSI")
            snr = rx_metadata[0].get("snr", "Unbekannter SNR")
        else:
            rssi = "Unbekannter RSSI"
            snr = "Unbekannter SNR"

        frm_payload_encoded = payload["uplink_message"].get("frm_payload", "")
        frm_payload = base64.b64decode(frm_payload_encoded).decode('utf-8') if frm_payload_encoded else "Leere Nutzdaten"

        # Vorbereite den Datenstring für die Dateischreibung
        data_string = f"{time};{device_id};{rssi};{snr};{frm_payload}\n"

        # Überprüfe, ob die Datei existiert, und schreibe ggf. einen Header
        file_exists = os.path.isfile("device_payloads.csv")
        with open("device_payloads.csv", "a") as file:
            if not file_exists:
                file.write("date;device-id;rssi;snr;payload\n")
            file.write(data_string)
    except Exception as e:
        print(f"Ein Fehler ist aufgetreten, die Nachricht wird ignoriert: {e}")



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
