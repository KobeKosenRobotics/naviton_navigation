#!/usr/bin/env python3
import rospy
import json
import ssl
from geometry_msgs.msg import PoseStamped
from paho.mqtt import client as mqtt_client

# MQTT broker settings
broker = "spatialintegration.japaneast-1.ts.eventgrid.azure.net"
port = 8883
topic = 'hic/dwh/location-adapter'
client_id = "kobe_kosen_01"

# Connect to MQTT broker
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo("Connected to MQTT Broker!")
        else:
            rospy.logerr(f"Failed to connect, return code {rc}")
    
    client = mqtt_client.Client(client_id)
    client.username_pw_set(username=client_id)
    client.tls_set(certfile='./kobe_kosen_01.cert', keyfile='./kobe_kosen_01.key', tls_version=ssl.PROTOCOL_TLSv1_2)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

# ROS callback function
def ros_callback(msg):
    data = {
        "device": {
            "device_id": "oit_01",
            "location_timestamp": rospy.Time.now().to_sec(),
            "position": {
                "position_type": 1,
                "latitude": msg.pose.position.x,  # Replace with real lat/lon if needed
                "longitude": msg.pose.position.y,
                "altitude": msg.pose.position.z,
                "voxel_id": None
            },
            "orientation": {
                "orientation_x": msg.pose.orientation.x,
                "orientation_y": msg.pose.orientation.y,
                "orientation_z": msg.pose.orientation.z,
                "orientation_w": msg.pose.orientation.w
            },
            "floor_name": ""
        }
    }
    
    msg_json = json.dumps(data)
    result = mqtt_client.publish(topic, msg_json)
    status = result[0]
    if status == 0:
        rospy.loginfo(f"Sent: {msg_json}")
    else:
        rospy.logerr("Failed to send message")

# Main function
def main():
    rospy.init_node("ros_mqtt_bridge", anonymous=True)
    client = connect_mqtt()
    client.loop_start()
    rospy.Subscriber("wpManager/absolute_position", PoseStamped, ros_callback)
    rospy.spin()
    client.loop_stop()

if __name__ == "__main__":
    main()
