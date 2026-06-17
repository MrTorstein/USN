"""Publishing data from modbus server to Azure Cosmos database"""

from time import sleep

from cosmos_db import CosmosDB
from modbus_client import ModbusReadClient

DATA_GETTER = ModbusReadClient()
DB_PUBLISHER = CosmosDB()

try:
    while True:
        DATA = DATA_GETTER.run_client(read_if_updated=False)
        if not isinstance(DATA, int):
            if DATA is not None and len(DATA.split(" ")) == 3:
                DATA = DATA.split(" ")
                TIME = DATA[0][-10:] + " " + DATA[1]
                try:
                    PUBLISHABLE = [{"id": TIME, "time": TIME, "temperature": float(DATA[2]), "sensor": "Simulator"}]
                except ValueError:
                    PUBLISHABLE = None
                DB_PUBLISHER.write(PUBLISHABLE)
                print(PUBLISHABLE)
            else:
                print(DATA)
        else:
            print(DATA)
        sleep(10)
except KeyboardInterrupt:
    DATA_GETTER.close()
