"""GUI test module"""
from time import sleep

from modbus_client import ModbusWriteClient, ModbusReadClient

CONTROLLER_CHANGER = ModbusWriteClient(device_nr=1)

CONTROLLER_CHANGER.run_client(25)

DATA_GATHERER = ModbusReadClient(device_nr=0)
try:
    while True:
        read_data = DATA_GATHERER.run_client()
        if not isinstance(read_data, int):
            if read_data is not None and len(read_data.split(" ")) == 3:
                print(read_data)
        sleep(0.01)
except KeyboardInterrupt:
    pass
