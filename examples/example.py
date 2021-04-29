from simplecarsim import Car, State, Simulator
import matplotlib.pyplot as plt
import numpy as np
from math import pi

t_fin_s = 50.0

vx_m_per_s = 25.0 # initial velocity
initial_state = State(vx_m_per_s = vx_m_per_s) # set initial state
car = Car(initial_state = initial_state)

s_x = np.zeros(2) # no torque applied to motor
delta_rad = 3.0 * np.ones(2) * pi / 180.0 # constant steering angle deflection
input_time_s = [0., t_fin_s]
sim_CCW = Simulator(t_final_time_s = t_fin_s, s_x = s_x, delta_rad = delta_rad, input_time_s = input_time_s, Delta_t_sim_s = 0.005)

t_output_s_CCW, state_output_CCW = sim_CCW.simulate(car = car) # simulate the car

# plot solution
plt.plot([output.x_m for output in state_output_CCW], [output.y_m for output in state_output_CCW])
plt.xlabel("X-position (m)")
plt.ylabel("Y-position (m)")
plt.title("Car Position")

plt.savefig("example_car_position.png")

# What do you see? Can you explain it? Hint: Think about which kind of forces we do not model in this simulator.