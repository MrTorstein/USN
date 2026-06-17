"""
Module containing functions used for publishing and subscribing to a nanomq mqtt broker
Functions:
- publish(data, topic) -> None
- subscribe(topic) -> None
"""

from os import getenv
from json import dumps, loads
from ast import literal_eval
from dotenv import load_dotenv
import paho.mqtt.client as mqtt

from mongodb_functions import write_to_db, read_from_db
from thingspeak_functions import write_to_ts

load_dotenv() # Loading from .env to os.environ

BROKER_ADRESSE = getenv("nanomq_adresse")
BROKER_PORT = int(getenv("nanomq_port"))
USERNAME = getenv("nanomq_username")
PASSWORD = getenv("nanomq_password")

def _on_connect(_, __, ___, reason_code, ____):
    """Function run when the broker is connected to"""
    if reason_code == 0:
        print("Connected successfully")
    else:
        print("Connection returned code: ", str(reason_code))

def _on_message(_, __, msg):
    """
    Function run when the broker sends a message  
    Prints message to terminal, writes data to MongoDB and ThingSpeak
    """
    last = False

    if msg.topic.split("/")[0] == "get":
        filter_dict = literal_eval(msg.payload.decode("utf-8"))
        if "last" in filter_dict:
            last = literal_eval(filter_dict.pop("last"))

        data = read_from_db(filter_dict, last, msg.topic.split("/")[-1])
        for dictionary in data:
            del dictionary["_id"]
        print("gotten: ", {"data" : data})
        publish({"data" : data}, "gotten")
    else:
        print("Message: ", msg.topic, " > ", msg.payload.decode("utf-8"))
        write_to_db([loads(msg.payload)], msg.topic.split("/")[-1])
        try:
            write_to_ts({"field1" : loads(msg.payload)["temperature"]})
        except KeyError:
            print("Missing key temperature, cannot write to TS")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = _on_connect
client.on_message = _on_message

client.username_pw_set(USERNAME, PASSWORD)
client.connect(BROKER_ADRESSE, BROKER_PORT)

def publish(data: dict, topic: str) -> None:
    """
    Function used to publish to the broker  
    Ex: publish({"temp" : 22}, "test/topic")
    """
    client.publish(topic, dumps(data))

def subscribe(topics: list[str]) -> None:
    """
    Function used to subscribe to a topic on the broker  
    Ex: subscribe("test/topic")
    """
    try:
        for topic in topics:
            client.subscribe(topic)
        client.loop_forever()
    except KeyboardInterrupt:
        for topic in topics:
            client.unsubscribe(topic)
        print("Exiting...")

DATA = ""
def get_data(filter_dict: dict, room: str) -> str:
    """TODOC"""
    global DATA

    def _on_message(_, __, msg):
        global DATA
        DATA = msg.payload.decode("utf-8")

    client.on_connect = lambda _, __, ___, rc, ____: rc
    client.on_message = _on_message

    client.subscribe("gotten")
    publish(filter_dict, f"get/{room}")

    client.loop_start()
    while DATA == "":
        pass
    client.loop_stop()

    return literal_eval(DATA)

if __name__ == "__main__":
    #DATA = {"temperature" : 12, "date" : "2025-10-14", "sensor" : "TMP36"}
    #print(DATA)
    #publish(DATA, "test/topic")
    subscribe(["test/topic"])
