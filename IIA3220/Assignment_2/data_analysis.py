"""Module used for getting data from mongodb via nanomq, plotting it and performing some analysis"""

from pandas import DataFrame, to_datetime
import matplotlib.pyplot as plt

from mqtt_functions import get_data

ROOM = "Office"
SENSOR = "TC74"
#SENSOR = "NTC 10K Thermistor"

data = DataFrame(get_data({"sensor": SENSOR}, ROOM)["data"])

data["date"] = to_datetime(data["date"])

print(f"Statistics of temperature in {ROOM}, measured by a {SENSOR}")
print("-------------------------------------------------------------------")
print(f"Mean: {round(data["temperature"].mean(), 1)} C")
print(f"Median: {data["temperature"].median()} C")
print(f"Standard deviation: {round(data["temperature"].std(), 1)} C")
print(f"Variance: {round(data["temperature"].var(), 1)}")

plt.figure()
plt.plot(data["date"], data["temperature"])
plt.title(f"Temperature in {ROOM} aquired from {SENSOR}")
plt.xlabel("date")
plt.ylabel("temperature [Celsius]")

plt.show()
