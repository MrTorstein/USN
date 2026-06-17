"""
Module containing functions used for writing to and reading from a MongoDB
Functions:
- write_to_db(document_list, test) -> None
- read_from_db(filter_dict, last, test) -> str
"""

from os import getenv
from dotenv import load_dotenv
import pymongo

load_dotenv() # Loading from .env to os.environ

DB_ADRESSE = getenv("temp_db_adresse")

client = pymongo.MongoClient(DB_ADRESSE)
database = client["TempDB"]
collection = database["Livingroom"]

def write_to_db(document_list: list, room: str = "Livingroom", test: bool = False) -> None:
    """
    Function used to write to a MongoDB  
    Ex: write_to_db([{"temp" : 22, "date" : "2025-10-12"}], test = True)
    """
    if room != "Livingroom":
        col = database[room]
    else:
        col = collection

    response = col.insert_many(document_list)

    if test:
        print(response)

def read_from_db(filter_dict: dict, last: bool = False, room: str = "Livingroom", test: bool = False) -> str:
    """
    Function used to read from a MongoDB  
    Ex: data = read_from_db({"sensor" : "TMP36"}, last = True, test = True)
    """
    if room != "Livingroom":
        col = database[room]
    else:
        col = collection

    result = col.find(filter_dict).to_list()

    if last:
        result = [result[-1]]

    if test:
        print(result)

    return result

if __name__ == "__main__":
    #write_to_db([{"temperature" : 22.03, "date" : "2025-10-12 20:59:59", "sensor" : "TMP36"}], test = True)
    read_from_db({"sensor" : "TMP36"}, last = True, test = True)
