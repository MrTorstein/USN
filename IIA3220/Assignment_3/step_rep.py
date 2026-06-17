"""TODOC"""

from numpy import zeros, where, array
from matplotlib.pyplot import plot, show, figure
from control import tf, pade, series, feedback, pzmap, margin, bode

from air_heater import AirHeaterSim
from pi_controller import PIController

# Transfer function air heater
N = 2000
time = zeros(N)
temp = zeros(N)
u_step = 0.1
u = zeros(N) + u_step

HEATER = AirHeaterSim()

for i in range(N):
    time[i], temp[i] = HEATER.step_forward(u[i])

k = round(temp[-1] / u_step, 2)
t = round(time[where(temp - (0.63 * (temp[-1] - temp[0]) + temp[0]) > 0)[0][0]], 2)
tau = round(time[where((temp - temp[0]) > 0.001)[0][0]], 2)

print("k = ", k)
print("t =", t)
print("tau =", tau)

plot(time, temp)

show(block = False)

if input("Are these values correct? [Y/N] ").lower() != "y":
    k = float(input("New k = "))
    t = float(input("New t = "))
    tau = float(input("New tau = "))

num = array([k])
den = array([t, 1])
h1 = tf(num, den)

# Transfer function PI controller
CONTROLLER = PIController(kp = 0.136, ti=2.00)
kp = CONTROLLER.kp
ti = CONTROLLER.ti
num = array([kp * ti, kp])
den = array([ti, 0])
h2 = tf(num, den)

[num, den] = pade(tau, 10)
h3 = tf(num, den)

h = series(h1, h2, h3)
l = feedback(h, 1)
figure()
pzmap(l)
bode(h, dB = True, deg = True, display_margins = True)
gm, _, w180, wc = margin(h)
print("wc = ", wc)
print("w180 = ", w180)
kc = kp * gm
print("kc = ", kc)
show()

"""
kp = 0.45 * kc = 0.45 * 0.3028 = 0.136
ti = tc / 1.2 = 2 * pi / 1.2 * w180 = 2 * pi / 1.2 * 2.6229 = 2.00
"""