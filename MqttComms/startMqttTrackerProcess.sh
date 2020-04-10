echo "Activating bike-tracker Anaconda environment"
source activate bike-tracker

echo "Starting MQTT Tracker Logging Process"
screen -dmSL MqttTrackerProcess python Subscriber.py
