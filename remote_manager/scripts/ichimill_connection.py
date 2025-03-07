#!/usr/bin/env python3
import rospy
import json
import time
import ssl
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseStamped

# MQTT設定
MQTT_CLIENT_ID = 'kobe_kosen_01'
MQTT_HOST = 'spatialintegration.japaneast-1.ts.eventgrid.azure.net'
MQTT_PORT = 8883
MQTT_TOPIC_DEVICE_INFO = 'hic/dwh/device-information-adapter'
MQTT_TOPIC_LOCATION_ADAPTER = 'hic/dwh/location-adapter'
MQTT_USER = MQTT_CLIENT_ID
MQTT_CERTFILE = '/home/catkin_ws/src/naviton_navigation/remote_manager/scripts/assets/{}.cert'.format(MQTT_CLIENT_ID)
MQTT_KEYFILE = '/home/catkin_ws/src/naviton_navigation/remote_manager/scripts/assets/{}.key'.format(MQTT_CLIENT_ID)

class MqttHandler:
    def __init__(self):
        self.client = mqtt.Client(client_id=MQTT_CLIENT_ID)
        self.client.username_pw_set(username=MQTT_USER)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_publish = self.on_publish
        self.mqtt_connected = False

        # TLS/SSL設定
        self.context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        self.context.load_cert_chain(certfile=MQTT_CERTFILE, keyfile=MQTT_KEYFILE)
        self.client.tls_set_context(self.context)

        # MQTTブローカーへ接続
        self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        """MQTT接続時の処理"""
        if rc == 0:
            rospy.loginfo("MQTT Connected successfully")
            self.mqtt_connected = True
        else:
            rospy.logerr(f"MQTT Connection failed with code {rc}")

    def on_disconnect(self, client, userdata, rc):
        """MQTT切断時の処理"""
        if rc != 0:
            rospy.logerr(f"MQTT Disconnected with code {rc}")

    def on_publish(self, client, userdata, mid):
        """MQTTメッセージ送信時の処理"""
        pass  # ここで特に何もしない

    def publish_fixed_location_message(self):
        """デバイスの固定情報をMQTTで送信"""
        ts = int(time.time() * 1000)
        message = json.dumps({
            "device": {
                "device_id": MQTT_CLIENT_ID,
                "location_timestamp": ts,
                "device_information_timestamp": 1733804049317,
                "battery_charge_remaining": 50,
                "device_status": 2,
                "task_status": 5001
            }
        })
        self.client.publish(MQTT_TOPIC_DEVICE_INFO, message)
        rospy.loginfo(f"Published device info message: {message}")

    def publish_location_message(self):
        """デバイスの位置情報をMQTTで送信（デバッグ用）"""
        if self.mqtt_connected:
            ts = int(time.time() * 1000)
            location_message = json.dumps({
                "device":{
                    "device_id": MQTT_CLIENT_ID,
                    "location_timestamp": 1709196356539,
                    "position": {
                        "position_type": 1,
                        "latitude": 35.620237385794,
                        "longitude": 139.8335141282929,
                        "altitude": 0,
                    },
                    "orientation": {
                        "orientation_x": 0,
                        "orientation_y": 0,
                        "orientation_z": -0.002617990887417953,
                        "orientation_w": -0.9999965730559848
                    },
                    "floor_name": ""
                }
            })
            self.client.publish(MQTT_TOPIC_LOCATION_ADAPTER, location_message)
            rospy.loginfo(f"Published location message: {location_message}")

    def disconnect(self):
        """MQTT接続を停止"""
        self.client.loop_stop()
        self.client.disconnect()

def main():
    """ROSとMQTTを統合するメイン関数"""
    global mqtt_handler
    rospy.init_node("ros_mqtt_bridge", anonymous=True)
    
    # MQTTハンドラーを初期化
    mqtt_handler = MqttHandler()

    # 定期的に位置情報とデバイス情報を送信するためのループ
    rate = rospy.Rate(1)  # 1 Hzで送信
    while not rospy.is_shutdown():
        if mqtt_handler.mqtt_connected:
            mqtt_handler.publish_fixed_location_message()  # デバイス情報を送信
            mqtt_handler.publish_location_message()  # 位置情報を送信
        rate.sleep()  # 1秒待機

if __name__ == "__main__":
    main()
