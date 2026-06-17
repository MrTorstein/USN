"""Modul setting up GUI for controlling an air heater"""
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from modbus_client import ModbusReadClient, ModbusWriteClient

class Plotter():
    """TODOC"""

    def __init__(self, window):
        self.window = window
        # The figure that will contain the plot
        self.fig = Figure(figsize = (5, 5), dpi = 100)

        # Adding the subplot
        self.subplot = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master = window)

    def plot(self, data: list[list, list]):
        """Plot data to canvas"""

        # Clearing plot
        self.clear()

        # Plotting the graph
        self.subplot.plot(data[0], data[1])

        # Creating the Tkinter canvas containing the Matplotlib figure
        self.canvas.draw()

        # Placing the canvas on the Tkinter window
        self.canvas.get_tk_widget().grid(column=0, row=2)

    def clear(self):
        """Clear canvas and resetup"""
        self.subplot.clear()
        self.subplot.set_ylim(20, 40)
        self.subplot.set_xlabel("Time")
        self.subplot.set_ylabel("Temperature [C]")

def ip_submit():
    """Updates IPs when a new is submitted"""
    UPDATE_CHECKER.update_ip_device()

def device_submit():
    """Updates device number when a new one is submitted"""
    UPDATE_CHECKER.update_ip_device()

def temp_submit():
    """Updates desired temperature when a new is submitted"""
    UPDATE_CHECKER.update_temp()

class CheckForUpdates():
    """Class used to check for and perform updates on plot and controller parameters"""

    def __init__(self):
        self.ip = "127.0.0.1"
        self.device_nr = 1
        
        self.max_plot_length = 100

        self.data_getter = ModbusReadClient(device_nr=self.device_nr - 1)
        self.parameter_setter = ModbusWriteClient(device_nr=self.device_nr)

    def update_ip_device(self):
        """TODOC"""
        ip = ip_input_text.get()
        device_nr = 2 * device_input_text.get() - 2

        if ip != self.ip:
            self.ip = ip
            self.data_getter.update_ip(ip)
            self.parameter_setter.update_ip(ip)
        if device_nr != self.device_nr:
            self.device_nr = device_nr
            self.data_getter.update_device_nr(device_nr)
            self.parameter_setter.update_device_nr(device_nr)

    def update_temp(self):
        """TODOC"""
        print(int(temp_input_text.get()))
        self.parameter_setter.run_client(int(temp_input_text.get()))

    def update_plot(self, data):
        """Get new data and run plotter to update canvas"""
        if len(data.split(" ")[0]) > 30:
            data = data[-30:]
        try:
            if len(DATA[0]) > 0:
                DATA[0].append(DATA[0][-1] + 1)
            else:
                DATA[0].append(0)
            DATA[1].append(float(data.split(" ")[2]))
            if len(DATA[0]) > self.max_plot_length:
                for _ in range(len(DATA[0]) - self.max_plot_length):
                    DATA[0].pop(0)
                DATA[1].pop(0)
            print(DATA)
            plotter_class.plot(DATA)
        except ValueError:
            pass

    def check_for_updates(self):
        """Main function for performing updatetask and rescheduling it"""

        data = self.data_getter.run_client()
        if not isinstance(data, int):
            if data is not None and len(data.split(" ")) == 3:
                self.update_plot(data)

        root.after(5000, self.check_for_updates)

def set_up_layout(window) -> tk.Tk:
    """Function to set up layout of gui"""

    window.bind()

    window.title("Air Heater Controller Application")

    # Set window params
    window_width = 900
    window_height = 600

    # get the screen dimension
    screen_width = window.winfo_screenwidth() + 400
    screen_height = window.winfo_screenheight() + 200

    # find the center point
    center_x = int(screen_width/2 - window_width / 2)
    center_y = int(screen_height/2 - window_height / 2)
    # set the position of the window to the center of the screen
    window.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')

    window.minsize(window_width, window_height)

    window.rowconfigure(0, weight=1)
    window.rowconfigure(1, weight=2)
    window.rowconfigure(2, weight=4)
    window.columnconfigure(0, weight=4)
    window.columnconfigure(1, weight=2)

    # Menu bar
    tk.Button(window, text='Help').grid(column=0, row=0, sticky=tk.NW)


    # Info text
    tk.Label(window, text="Air Heater Controller\n" \
    "Choose an IP, a device number and get data.\n" \
    "You can also choose to set a new temperature target for the heater.").grid(column=0, row=1)

    # Controller choise
    controller_parameters = tk.Frame(window)
    controller_parameters.columnconfigure(0, weight=1)
    controller_parameters.columnconfigure(1, weight=2)
    controller_parameters.columnconfigure(2, weight=1)
    controller_parameters.rowconfigure(0, weight=1)
    controller_parameters.rowconfigure(1, weight=1)

    tk.Label(controller_parameters, text = "IP:").grid(column=0, row=0)
    ip_text_var = tk.StringVar(value = "127.0.0.1")
    tk.Entry(controller_parameters, textvariable=ip_text_var).grid(column=1, row=0)
    tk.Button(controller_parameters,
              text='Submit',
              command=ip_submit).grid(column=2, row=0)

    tk.Label(controller_parameters, text = "Device nr:").grid(column=0, row=1)
    device_int_var = tk.IntVar(value = 1)
    tk.Entry(controller_parameters, textvariable=device_int_var).grid(column=1, row=1)
    tk.Button(controller_parameters,
              text='Submit',
              command=device_submit).grid(column=2, row=1)

    controller_parameters.grid(column=1, row=1)

    # Plot
    plotter = Plotter(window)

    # Desired temperature
    temp = tk.Frame(window)
    temp.columnconfigure(0, weight=1)
    temp.columnconfigure(1, weight=2)
    temp.columnconfigure(2, weight=1)
    tk.Label(temp, text = "Desired temperature:").grid(column=0, row=0)
    temp_float_var = tk.DoubleVar(value = 22)
    tk.Entry(temp, textvariable=temp_float_var).grid(column=1, row=0)
    tk.Button(temp,
              text='Submit',
              command=temp_submit).grid(column=2, row=0)
    temp.grid(column=1, row=2)

    # Scedule update task
    window.after(100, UPDATE_CHECKER.check_for_updates)

    return ip_text_var, device_int_var, temp_float_var, plotter

DATA = [[], []]

UPDATE_CHECKER = CheckForUpdates()

root = tk.Tk()
ip_input_text, device_input_text, temp_input_text, plotter_class = set_up_layout(root)

# Keep the window displaying and in a more focused view
try:
    from ctypes import windll
    windll.shcore.SetProcessDpiAwareness(1)
finally:
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
