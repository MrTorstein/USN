"""
Discret time PI controller for controlling a process live

Algoritm:
u = u_prev + kp * (e - e_prev) + kp / ti * e

Parameters:
u = Control signal
u_prev = Last control signal
kp = Propotional gain
e = Error term r - y
r = Reference value, the value desired by the user
e_prev = Last steps error term
ti = Integral time
ts = Time Step
"""

class PIController():
    """
    PIController class  
    Input parameters:  
    kp = Propotional gain  
    ti = Integral time  
    ts = Time step
    """

    def __init__(self, kp: float = 0.136, ti: float = 2.00, ts: float = 0.1):
        self.kp = kp
        self.ti = ti
        self.ts = ts
        self.e_prev = 0
        self.u_prev = 0

    def get_signal(self, y: float, r: float, u_prev: float = None, e_prev: float = None, u_lim: tuple[float, float] = (None, None)) -> float:
        """Function calculating a signal given a reference value, a current value and previous error and signal"""
        if u_prev is not None:
            self.u_prev = u_prev
        if e_prev is not None:
            self.e_prev = e_prev

        # Calculate error term
        e = r - y

        # Calculate new control signal
        u = self.u_prev + self.kp * (e - self.e_prev) + self.kp / self.ti * self.ts * e

        if u_lim != (None, None):
            if u < u_lim[0]:
                u = u_lim[0]
            elif u > u_lim[1]:
                u = u_lim[1]

        # Update prev
        self.e_prev = e
        self.u_prev = u

        return round(u, 2)

if __name__ == "__main__":
    from matplotlib.pyplot import plot, show
    from air_heater import AirHeaterSim
    HEATER = AirHeaterSim()
    CONTROLLER = PIController()

    time = [0]
    temp = [HEATER.t_env]
    for i in range(4000):
        sig = CONTROLLER.get_signal(temp[i], 25)
        _time, _temp = HEATER.step_forward(sig)
        temp.append(_temp)
        time.append(_time)

    plot(time, temp)
    show()
