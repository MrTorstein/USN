"""
Module containing functions used for writing to and reading from ThingSpeak
Functions:
- write_to_ts(dict_to_write, test) -> None
- read_from_ts(field_to_get, last, only_value, test) -> str
"""

from os import getenv
from dotenv import load_dotenv
from thingspeak import Channel

load_dotenv() # Loading from .env to os.environ

channel_id = getenv("temp_logger_channel_id")

def write_to_ts(dict_to_write: dict, test: bool = False) -> None:
    """
    Function for writing to ThingSpeak  
    Ex: write_to_ts({"field1" : 22}, test = True)
    """
    write_key = getenv("temp_logger_write_key")

    channel = Channel(id = channel_id, api_key = write_key)
    response = channel.update(dict_to_write)

    if test:
        print(response)

def read_from_ts(field_to_get: int, last: bool = False, only_value: bool = False, test: bool = False) -> str:
    """
    Function for reading from ThingSpeak  
    Ex: data = read_from_ts(0, False, False, True)
    """
    read_key = getenv("temp_logger_read_key")

    channel = Channel(id = channel_id, api_key = read_key)

    if last:
        response = channel.get_field_last(field_to_get)[:-1]
    else:
        response = channel.get_field(field_to_get)

    if only_value:
        result = []
        for substring in response.split("},{"):
            result.append(substring.split(f'"field{field_to_get}":')[-1])

        if not last:
            result[-1] = result[-1][:-3]
    else:
        result = [response]

    response = ""
    for substring in result:
        response = response + f"{substring},"

    response = response[:-1]

    if test:
        print(response)

    return response

if __name__ == "__main__":
    TEMPERATURE = 25
    read_from_ts(2, False, True, True)
