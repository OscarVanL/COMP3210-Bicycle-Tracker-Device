import paho.mqtt.client
import sys
import yaml


def on_connect(client, userdata, flags, rc):
    print('connected (%s)' % client._client_id)
    client.subscribe(topic='[topic]', qos=2)


def on_message(client, userdata, message):
    print('------------------------------')
    print('topic: %s' % message.topic)
    print('payload: %s' % message.payload)
    print('qos: %d' % message.qos)


def main():
    # Load MQTT configuration from MQTT_Config.yaml
    with open(r'MQTT_Config.yaml') as configuration:
        config = yaml.load(configuration, Loader=yaml.FullLoader)
        print(config)

    # Start MQTT subscription listening for data from the tracker
    client = paho.mqtt.client.Client(client_id=None, clean_session=False)
    client.username_pw_set(config['MQTT_USER'], config['MQTT_PASS'])
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host=config['MQTT_BROKER'], port=config['MQTT_PORT'])
    client.loop_forever()


if __name__ == '__main__':
    main()
    sys.exit(0)
