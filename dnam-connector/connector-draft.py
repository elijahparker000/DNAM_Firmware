import mysql.connector
import requests
import logging
import schedule
import time
from datetime import datetime

# Datenbank-Konfiguration
db_config = {
    "host": "localhost",
    "user": "your_username",
    "password": "your_password",
    "database": "your_database"
}

# Zeitintervall für Abfragen
INTERVAL_MINUTES = 1

# Logging-Konfiguration
logging.basicConfig(filename='sensor_update.log', level=logging.DEBUG, format='%(asctime)s %(message)s')

def db_connect():
    return mysql.connector.connect(**db_config)

def get_api_config(sensor_id):
    conn = db_connect()
    cursor = conn.cursor()
    query = """
    SELECT ae.endpoint_url, ae.app_id, ae.api_key
    FROM ApiEndpoint ae
    JOIN UserSensor us ON ae.user_id = us.user_id
    WHERE us.sensor_id = %s
    """
    cursor.execute(query, (sensor_id,))
    result = cursor.fetchone()
    cursor.close()
    conn.close()
    return result if result else (None, None, None)

def send_sensor_data_to_api(sensor_id, data):
    endpoint_url, app_id, api_key = get_api_config(sensor_id)
    if endpoint_url and app_id and api_key:
        headers = {"Authorization": f"Bearer {api_key}"}
        response = requests.post(endpoint_url, headers=headers, json=data)
        logging.info(f"Senden der Daten für Sensor {sensor_id} an {endpoint_url}, Antwort: {response.status_code}")

def fetch_sensor_data_from_api(sensor_id):
    endpoint_url, app_id, api_key = get_api_config(sensor_id)
    if endpoint_url and app_id and api_key:
        headers = {"Authorization": f"Bearer {api_key}"}
        response = requests.get(endpoint_url, headers=headers)
        if response.status_code == 200:
            return response.json()
        else:
            logging.error(f"Fehler beim Abrufen der Daten für Sensor {sensor_id}: {response.status_code}")
    return None

def save_sensor_data(sensor_id, data):
    conn = db_connect()
    cursor = conn.cursor()
    query = """
    INSERT INTO SensorData (sensor_id, timestamp, value, status_code)
    VALUES (%s, %s, %s, %s)
    """
    timestamp = datetime.now()
    value = data.get("value")  # Wert muss entsprechend der API-Antwort angepasst werden
    status_code = data.get("status_code", 0)
    cursor.execute(query, (sensor_id, timestamp, value, status_code))
    conn.commit()
    cursor.close()
    conn.close()
    logging.info(f"Sensordaten für {sensor_id} gespeichert.")

def check_for_updates():
    conn = db_connect()
    cursor = conn.cursor()
    query = "SELECT sensor_id FROM Sensor WHERE update_flag = TRUE"
    cursor.execute(query)
    sensors_to_update = cursor.fetchall()
    cursor.close()
    conn.close()
    return [sensor[0] for sensor in sensors_to_update]

def main():
    logging.info("Starte den Update-Prozess")
    sensors_to_update = check_for_updates()
    for sensor_id in sensors_to_update:
        sensor_data = fetch_sensor_data_from_api(sensor_id)
        if sensor_data:
            save_sensor_data(sensor_id, sensor_data)
            # Hier könnten Daten aktualisiert werden
            send_sensor_data_to_api(sensor_id, sensor_data)

    logging.info("Warte auf Sensordaten von TheThingsNetwork")

# Zeitplan einrichten
schedule.every(INTERVAL_MINUTES).minutes.do(main)

# Endlosschleife, um die geplanten Aufgaben auszuführen
while True:
    schedule.run_pending()
    time.sleep(1)

