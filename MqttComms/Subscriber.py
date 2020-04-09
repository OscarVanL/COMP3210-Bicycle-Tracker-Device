import paho.mqtt.client
import sys
import yaml
import uuid
from MqttComms.proto_files import BikeTrackerPayload_pb2  # Our compiled Schema for Python


def on_connect(client, userdata, flags, rc):
    print('connected (%s)' % client._client_id)
    client.subscribe(topic='[topic]', qos=2)


def on_message(client, userdata, message):
    print('------------------------------')
    print('topic: %s' % message.topic)
    #print('payload: %s' % message.payload)
    print('payload: ', end='')
    print(''.join('\\x{:02x}'.format(letter) for letter in message.payload))
    print('qos: %d' % message.qos)

    print('Decoding message:')
    tracker_payload = BikeTrackerPayload_pb2.GpsPayload()
    tracker_payload.ParseFromString(message.payload)

    print('GPS Fix:', tracker_payload.gps_fix)
    print('Epoch:', tracker_payload.epoch)
    print('Battery:', tracker_payload.battery, '%')
    print('Latitude:', tracker_payload.latitude)
    print('Longitude:', tracker_payload.longitude)
    print('Speed km/h:', tracker_payload.speed_kph)
    print('Heading:', tracker_payload.heading)
    print('Altitude:', tracker_payload.altitude)


def main():
    # Load MQTT configuration from MQTT_Config.yaml
    with open(r'MQTT_Config.yaml') as configuration:
        config = yaml.load(configuration, Loader=yaml.FullLoader)
        print(config)

    # Start MQTT subscription listening for data from the tracker.
    client = paho.mqtt.client.Client(client_id=str(uuid.getnode()), clean_session=False)
    client.username_pw_set(config['MQTT_USER'], config['MQTT_PASS'])
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host=config['MQTT_BROKER'], port=config['MQTT_PORT'])
    client.subscribe(topic="biketracker/payload")
    client.loop_forever()


if __name__ == '__main__':
    main()
    sys.exit(0)
