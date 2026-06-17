"""Module ment to be run as a slave for the nanomq mqtt broker to write to thingspeak and Mongodb"""

from mqtt_functions import subscribe

TOPICS = ["Temp/Office", "get/Office", "get/Livingroom"]

subscribe(TOPICS)
