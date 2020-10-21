# COMP3210-Bike-Tracker

This project was for a small coursework task in COMP3210, Advanced Computer Networks. 

I decided for this coursework that I would create a discreet battery-powered IoT tracker for my bicycle, this would include a GPS module, 2G/3G cellular connectivity, and operate on a sleepy principle, where it only wakes up briefly periodically to send a GPS location to a Raspberry-Pi hosted MQTT broker.

Procol Buffer is used to make the payload as small as possible, this reduces the cost of mobile data, but also preserves battery life as the radio is active for less time.

The tracker was based on an Adafruit FONA 3G, using a 'Things Mobile' SIM card, to give the best possible IoT cellular coverage. I used an Arduino Nano as the MCU. In concept, the Adafruit FONA 3G breakoutboard would have made this very easy because of their purpose-made Arduino libraries for cellular communications, GPS, and MQTT. 

Instead this project was an absolute nightmare for someone with no experience of cellular modems and AT-commands because this library doesn't support the FONA 3G, only the FONA 808. The library was built for the Simcom SIM808, but my board used the Simcom SIM5320, which had no compatibility.

I would highly advise people against buying the Adafruit FONA 3G if they expect to use libraries for the heavy lifting.
