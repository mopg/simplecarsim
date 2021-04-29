from copy import deepcopy
import numpy as np
from scipy.interpolate import interp1d

class Simulator( object ):

    def __init__(
        self,
        s_x = None,
        delta_rad = None,
        input_time_s = None,
        Delta_t_sim_s = None,
        t_final_time_s = None,
    ):

        if s_x is None: s_x = np.zeros(150)
        if delta_rad is None: delta_rad = np.zeros(150)
        if input_time_s is None: input_time_s = np.linspace(0., 15.0, 150)
        if Delta_t_sim_s is None: Delta_t_sim_s = 1e-3
        if t_final_time_s is None: t_final_time_s = 15.0

        assert t_final_time_s <= input_time_s[-1]
        assert len(s_x) == len(delta_rad)
        assert len(s_x) == len(input_time_s)

        self.s_x = s_x
        self.delta_rad = delta_rad
        self.input_time_s = input_time_s

        self.Delta_t_sim_s = Delta_t_sim_s
        self.t_final_time_s = t_final_time_s

        self.delta_rad_interp = interp1d(input_time_s, delta_rad)
        self.s_x_interp = interp1d(input_time_s, s_x)


    def simulate(self, car):

        # time steps through each state, integrating it over time
        # this is described in Section 1.4 of the document.

        curr_time_s = 0.0

        state_output = [deepcopy(car.current_state)]
        time_output_s = [curr_time_s]
        
        while curr_time_s <= self.t_final_time_s:

            # interpolate control inputs
            delta_rad = self.delta_rad_interp(curr_time_s)
            s_x = self.s_x_interp(curr_time_s)

            # advance state
            car.advance_state(delta_rad = delta_rad, s_x = s_x, Delta_t_s = self.Delta_t_sim_s)

            # save state for output
            state_output.append(deepcopy(car.current_state))
            time_output_s.append(curr_time_s)

            curr_time_s += self.Delta_t_sim_s

        return time_output_s, state_output