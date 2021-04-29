from .State import State
from .TireModel import LinearTireModel
from scipy import constants
from math import cos, sin
from copy import copy, deepcopy

class Car( object ):
    '''
    Car object

    Attributes:

        cg_ratio       Center of gravity ratio between front and rear axle. When cg_ratio = 0, the cg is on the front axle.
        l_wheelbase_m  Wheel base of car -- distance between front and rear axle.
        mass_kg        Mass of the car in kg
        Izz_kg_m2      Polar moment of inertia of the car in kg*m2
        tire_model     Tire model to be used in simulation
    '''

    def __init__(
        self,
        l_wheelbase_m = 1.2,
        mass_kg = 163.0,
        Izz_kg_m2 = 75.0,
        initial_state = State(),
        tire_model = LinearTireModel(),
        cg_ratio = 0.5,
    ):

        self.l_wheelbase_m = l_wheelbase_m
        self.mass_kg = mass_kg
        self.Izz_kg_m2 = Izz_kg_m2
        self.current_state = deepcopy(initial_state)
        self.cg_ratio = cg_ratio
        self.tire_model = tire_model

    def advance_state(
        self,
        delta_rad = 0.0,
        s_x = 0.0, # assume front and rear tire have the same longitudinal slip ratio (it's a dumb powertrain)
        Delta_t_s = 0.001,
    ):

        # compute tire forces on front and rear tire
        V_m_per_s = self.current_state.get_V_m_per_s()
        beta_rad = self.current_state.get_beta_rad()

        # compute normal load on tires
        Fz_tire_front = self.mass_kg * constants.g * (1.0 - self.cg_ratio)
        Fz_tire_rear = self.mass_kg * constants.g * self.cg_ratio

        # compute tire slip ratios for front and rear tire
        distance_CG_to_front_axle_m = self.cg_ratio * self.l_wheelbase_m
        distance_CG_to_rear_axle_m = self.l_wheelbase_m - distance_CG_to_front_axle_m

        # compute lateral side slip ratios. Equation 6 in document.
        s_y_front = 
        s_y_rear = 

        # dumb four wheel drive -- we assume the front and rear axle are torqued the same amount, which is not true in most 4WD cars.
        s_x_front = s_x
        s_x_rear = s_x

        # compute the tire forces (use the tire model in self.tire_model to compute the tire forces)
        Fx_tire_front, Fy_tire_front = 
        Fx_tire_rear, Fy_tire_rear = 

        # compute forces and moment (these are in the vehicle body frame). Equation 4 in document.
        Fx = 
        Fy = 
        Mz = 

        # rotate from vehicle body frame into ground-fixed frame. Equation 5 in document.
        Fx_ground = 
        Fy_ground = 

        # compute vehicle accelerations. Newton's second law. Equations 3d to 3f.
        x_ddot_m_per_s2 = 
        y_ddot_m_per_s2 = 
        psi_ddot_rad_per_s2 = 

        # save control commands for plotting
        self.current_state.delta_rad = delta_rad
        self.current_state.s_x = s_x

        self.current_state.advance_state(
            x_ddot_m_per_s2 = x_ddot_m_per_s2,
            y_ddot_m_per_s2 = y_ddot_m_per_s2,
            psi_ddot_rad_per_s2 = psi_ddot_rad_per_s2,
            Delta_t_s = Delta_t_s,
        )   
