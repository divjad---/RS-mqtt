#
# Copyright 2021 HiveMQ GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import datetime
import json

import paho.mqtt.client as paho
from flask import Flask, jsonify
from flask_cors import CORS
from paho import mqtt

# Flask constructor takes the name of
# current module (__name__) as argument.
app = Flask(__name__)
app.debug = True
CORS(app)

topic = "RS-data"
password = "RSprojectESP8266"
user = "rs-user"


@app.route('/')
def serve_file():
    with open('data.json', 'r+') as jsonfile:
        file = jsonfile.read()
        data = eval(file)
        return jsonify(data)


# setting callbacks for different events to see if it works, print the message etc.
def on_connect(client, userdata, flags, rc, properties=None):
    print("CONNACK received with code %s." % rc)
    if rc == 0:
        print("Connected to MQTT broker")
    else:
        print(f"Connection failed with error code {rc}")


# with this callback you can see if your publish was successful
def on_publish(client, userdata, mid, properties=None):
    print("mid: " + str(mid))


# print which topic was subscribed to
def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


# print message, useful for checking if it was successful
def on_message(client, userdata, msg):
    payload: str = msg.payload.decode("utf-8")
    print(msg.topic + " " + str(msg.qos) + " " + payload)

    if msg.topic == topic:
        data = ""
        with open('data.json', 'r+') as jsonfile:
            file = jsonfile.read()
            data = eval(file)

        with open('data.json', "w") as jsonfile:
            print("File", data)
            # data = json.loads(str(data))
            json_payload = json.loads(payload)
            json_payload["time"] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            activity = json_payload["activity"]
            del json_payload["activity"]
            payload = eval(payload)
            del payload["activity"]
            payload["time"] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            data[activity].append(payload)

            jsonfile.write(json.dumps(data))


def run_app():
    app.run(debug=False, threaded=True)


def connect_mqtt():
    print("Connect to MQTT")

    # using MQTT version 5 here, for 3.1.1: MQTTv311, 3.1: MQTTv31
    # userdata is user defined data of any type, updated by user_data_set()
    # client_id is the given name of the client
    client = paho.Client(client_id="python-mqtt", userdata=None, protocol=paho.MQTTv5)
    client.on_connect = on_connect

    # enable TLS for secure connection
    client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
    # set username and password
    client.username_pw_set(user, password)
    # connect to HiveMQ Cloud on port 8883 (default for MQTT)
    client.connect("37aad5450fca492297d7d3bc3329f4ba.s2.eu.hivemq.cloud", 8883)

    # setting callbacks, use separate functions like above for better visibility
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    client.on_publish = on_publish

    # subscribe to all topics of encyclopedia by using the wildcard "#"
    client.subscribe(topic, qos=2)

    # loop_forever for simplicity, here you need to stop the loop manually
    # you can also use loop_start and loop_stop
    client.loop_start()


with app.app_context():
    connect_mqtt()
    print("From here")
