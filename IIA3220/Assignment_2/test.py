"""NODOC"""
from random import randint
from datetime import datetime
from time import sleep
from pytz import timezone
from mqtt_functions import publish

while True:
    DATA = randint(1, 40)
    DATE = str(datetime.now(timezone("Europe/Oslo")))
    publishable = {"temperature" : DATA, "date" : DATE, "sensor" : "TN10"}
    print(publishable)
    publish(publishable, "Temp/Office")

    sleep(20)
