"""
Module containing interaction with air heater and an air heater simulator
"""

from time import sleep

from nidaqmx import Task

class AirHeaterSim():
    """
    Air heater simulation class

    Input parameters:  
    t_env = Environment temperature [deg C]  
    kh = Heater gain [C/V]  
    time_consts = Tuple containing time constants eg. [Time constant, Time delay, Time step]

    Default start values:  
    t_env = 22 [deg C]  
    t = t_env [deg C] :Temperature from heater outlet  
    kh = 3.5 [C/V]  
    u = 0 [V]  
    time_const = 23 [s]  
    time_delay = 3 [s] :Time delay from changing signal to outlet temp is changed because of tube length  
    time = 0 [s] :current time step  

    Model equation:
    dt_dtime = ((t_env - t) + kh * u_{t - time_delay}) / time_const
    """
    def __init__(self, t_env: float = 22, kh: float = 3.5, time_consts: tuple[float, float, float] = (23, 3, 0.1)) -> None:
        self.t_env = t_env
        self.kh = kh
        self.time_consts = time_consts
        self.t = t_env
        self.time = 0
        self.u = [0] * (int(self.time_consts[1] / self.time_consts[2]) - 1)

    def step_forward(self, u: float = 0) -> tuple:
        """
        Function stepping one time step dtime forward, applying the control signal u
        The function uses simple forward Euler algorithm
        Returns list containing new time and temperature
        """
        self.u.append(u)

        while len(self.u) != int(self.time_consts[1] / self.time_consts[2]):
            if len(self.u) > int(self.time_consts[1] / self.time_consts[2]):
                self.u.append(self.u[-1])
            else:
                self.u.pop()

        self.time = self.time + self.time_consts[2]
        dt_dtime = ((self.t_env - self.t) + self.kh * self.u.pop(0)) / self.time_consts[0]
        self.t = self.t + dt_dtime * self.time_consts[2]
        self.t = max(self.t, self.t_env)

        return round(self.time, len(str(self.time_consts[2]))), round(self.t, 1)

    def change_params(self, t_env: float = None, kh: float = None, time_const: float = None, time_delay: float = None) -> None:
        """Function used for changing the constant parameters after initialisation"""
        if t_env is not None:
            self.t_env = t_env
        if kh is not None:
            self.kh = kh
        if time_const is not None:
            self.time_consts[0] = time_const
        if time_delay is not None:
            self.time_consts[1] = time_delay

class AirHeater():
    """
    Class for connecting to and controlling/reading from a USN air heater.  
    This connection uses an NI USB-6008 DAQ device to send control signals and  
    get sensor data  

    Input parameters:  
    daq_name: DAQ device name = "heater"  
    heater_channel: Analog output channel used for controlling the heater = "ao0"  
    temp1_channel: Analog input channel for reading temperature at outlet = "ai0"  
    """
    
    def __init__(self, daq_name: str = "heater", heater_channel: str = "ao0", temp1_channel: str = "ai0"):
        self.write_task = Task()
        self.write_task.ao_channels.add_ao_voltage_chan(f"{daq_name}/{heater_channel}", "heater_write_channel", min_val=0, max_val=5)
        self.write_task.start()
        self.read_task = Task()
        self.read_task.ai_channels.add_ai_voltage_chan(f"{daq_name}/{temp1_channel}", "heater_read_channel",min_val=1, max_val=5)
        self.read_task.start()

    def write_control_signal(self, u: float) -> None:
        """Function for updating control signal of an air heater"""
        if u > 5:
            u = 5
        elif u < 0:
            u = 0

        self.write_task.write(u)

    def read_temperature_outlet(self) -> float:
        """
        Function used to read temperature at air heater outlet  
        The function uses a linear voltage to temperature conversion:  
        Temp = 12.5 * Volt - 12.5
        """
        return round(12.5 * self.read_task.read() - 12.5, 1)

    def close(self):
        """Function used to clean close the connections to the DAQ device"""
        self.write_task.stop()
        self.write_task.close()
        self.read_task.stop()
        self.read_task.close()

if __name__ == "__main__":
    HEATER = AirHeaterSim()
    while True:
        print(HEATER.u)
        print(HEATER.step_forward(u = 1))
        sleep(1)
