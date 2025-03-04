# python 3.6
import random
import time
import ssl
from paho.mqtt import client as mqtt_client
broker = "spatialintegration.japaneast-1.ts.eventgrid.azure.net"
port = 8883
topic = 'hic/dwh/location-adapter'
client_id = f"kobe_kosen_01"
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    client=mqtt_client.Client(client_id='kobe_kosen_01')
    client.username_pw_set(username='kobe_kosen_01')
    #client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    client.tls_set(certfile='./kobe_kosen_01.cert', keyfile='./kobe_kosen_01.key', tls_version=ssl.PROTOCOL_TLSv1_2)
    # client.tls_set(
    #     ca_certs='kobe_kosen_01.csr',
    #     certfile='kobe_kosen_01.cert',
    #     keyfile='kobe_kosen_01.key'
    # )
    client.on_connect = on_connect
    client.connect(broker, port)
    return client
def publish(client):
    msg_count = 1
    while True:
        time.sleep(1)
        msg = '{\
    “device”:{\
        “device_id”: “oit_01",\
        “location_timestamp”: 1709296356539,\
        “position”: {\
            “position_type”: 1,\
            “latitude”: 35.620227385794,\
            “longitude”: 139.8333141282929,\
            “altitude”: 0,\
            “voxel_id”: null\
        },\
        “orientation”: {\
            “orientation_x”: 0,\
            “orientation_y”: 0,\
            “orientation_z”: -0.002617990887417953,\
            “orientation_w”: -0.9999965730559848\
        },\
        “floor_name”: “”\
    }\
}'
#“messages: {msg_count}”
        result = client.publish(topic, msg)
        status = result[0]
        if status == 0:
            print(f"Send '{msg}' to topic '{topic}'")
        else:
            print(f"Failed to send message to topic {topic}")
        msg_count += 1
        if msg_count > 5:
            break
def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)
    client.loop_stop()
if __name__ == '__main__':
    run()