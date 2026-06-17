"""Module ment to run a single air heater simulator with a controller and publishing the data to a modbus server"""

from time import sleep
from datetime import datetime
from pytz import timezone

from air_heater import AirHeaterSim
from pi_controller import PIController
from modbus_client import ModbusWriteClient, ModbusReadClient

HEATER = AirHeaterSim()
CONTROLLER = PIController()
PUBLISHER = ModbusWriteClient(device_nr=0)
CONTROLLER_PARAMETER_GETTER = ModbusReadClient(device_nr=1)

TIME = 0
TEMPERATURE = HEATER.t_env
DESIRED_TEMPERATURE = TEMPERATURE + 1
try:
    while True:
        _DESIRED_TEMPERATURE = CONTROLLER_PARAMETER_GETTER.run_client()
        if _DESIRED_TEMPERATURE is not None:
            DESIRED_TEMPERATURE = float(_DESIRED_TEMPERATURE)
        CONTROL_SIGNAL = CONTROLLER.get_signal(TEMPERATURE, DESIRED_TEMPERATURE)
        TIME, TEMPERATURE = HEATER.step_forward(CONTROL_SIGNAL)
        print(TIME, " s |", TEMPERATURE, " Celsius | ", DESIRED_TEMPERATURE)
        DATE = str(datetime.now(timezone("Europe/Oslo"))).split("+")
        DATE = DATE[0][:-5] + "+" + DATE[1]
        PUBLISHER.run_client(DATE + " " + str(TEMPERATURE))
        sleep(0.1)
except KeyboardInterrupt:
    PUBLISHER.close()
    CONTROLLER_PARAMETER_GETTER.close()
