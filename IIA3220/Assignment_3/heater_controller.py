"""Module used to run an actual air heater with a pi controller and publishing the data to a modbus server"""

from time import sleep
from datetime import datetime
from pytz import timezone

from low_pass_filter import low_pass_filter
from air_heater import AirHeater
from pi_controller import PIController
from modbus_client import ModbusWriteClient, ModbusReadClient

HEATER = AirHeater(heater_channel="ao1")
CONTROLLER = PIController()
PUBLISHER = ModbusWriteClient(device_nr=0)
CONTROLLER_PARAMETER_GETTER = ModbusReadClient(device_nr=1)

TEMP_ENV = 22
FILTERED_TEMP = TEMP_ENV
TIME = 0
TEMPERATURE = TEMP_ENV
DESIRED_TEMPERATURE = TEMP_ENV
try:
    while True:
        _DESIRED_TEMPERATURE = CONTROLLER_PARAMETER_GETTER.run_client()
        if not isinstance(_DESIRED_TEMPERATURE, int):
            if _DESIRED_TEMPERATURE is not None and len(_DESIRED_TEMPERATURE) < 20:
                DESIRED_TEMPERATURE = float(_DESIRED_TEMPERATURE)
        else:
            if _DESIRED_TEMPERATURE < 50:
                DESIRED_TEMPERATURE = _DESIRED_TEMPERATURE
        TEMP = HEATER.read_temperature_outlet()
        FILTERED_TEMP = low_pass_filter(TEMP, FILTERED_TEMP)
        SIGNAL = CONTROLLER.get_signal(FILTERED_TEMP, DESIRED_TEMPERATURE, u_lim=(0, 5))
        HEATER.write_control_signal(SIGNAL)
        DATE = str(datetime.now(timezone("Europe/Oslo"))).split("+")
        DATE = DATE[0][:-5] + "+" + DATE[1]
        PUBLISHER.run_client(DATE + " " + str(FILTERED_TEMP))
        print(DATE, " | ", FILTERED_TEMP, " deg C | ", SIGNAL, " V")
        sleep(0.1)
except (KeyboardInterrupt, ConnectionError):
    HEATER.close()
    PUBLISHER.close()
    CONTROLLER_PARAMETER_GETTER.close()