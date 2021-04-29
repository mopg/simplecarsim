from math import sqrt, atan2, pi, ceil

class State( object ):
    '''
    State object

    Attributes:
        x_m                 x-position, in meters
        y_m                 y-position, in meters
        psi_rad             yaw angle of car, in radians
        vx_m_per_s          velocity in x-direction, in meters/second
        vy_m_per_s          velocity in y-direction, in meters/second
        psi_dot_rad_per_s   angular velocity in yaw, in radians/second

        delta_rad           steering angle, in radians (only used for storage and plotting later)
        s_x                 longitudinal slip ratio, unitless (only used for storage and plotting later)
    '''

    def __init__(
        self,
        x_m = 0.0,
        y_m = 0.0,
        vx_m_per_s = 1.0,
        vy_m_per_s = 0.0,
        psi_rad = 0.5 * pi,
        psi_dot_rad_per_s = 0.0,
    ):

        self.x_m = x_m
        self.y_m = y_m
        self.vx_m_per_s = vx_m_per_s
        self.vy_m_per_s = vy_m_per_s
        self.psi_rad = psi_rad
        self.psi_dot_rad_per_s = psi_dot_rad_per_s

        # just to save output
        self.delta_rad = 0.0
        self.s_x = 0.0
        
    def advance_state(
        self,
        x_ddot_m_per_s2,
        y_ddot_m_per_s2,
        psi_ddot_rad_per_s2,
        Delta_t_s,
    ):
        '''
            Given accelerations in x (x_ddot_m_per_s2), y (y_ddot_m_per_s2), and yaw (psi_ddot_rad_per_s2) and a time step Delta_t_s, increment the state.
            This is described in Section 1.4 of the document.
        '''

        V_m_per_s = self.get_V_m_per_s()

        # update positions -- this uses the velocities from the current time step (remember delta_x is approximate vx * delta_t)
        self.x_m += self.vx_m_per_s * Delta_t_s
        self.y_m += self.vy_m_per_s * Delta_t_s
        self.psi_rad += self.psi_dot_rad_per_s * Delta_t_s

        # update velocities
        self.vx_m_per_s += x_ddot_m_per_s2 * Delta_t_s
        self.vy_m_per_s += y_ddot_m_per_s2 * Delta_t_s
        self.psi_dot_rad_per_s += psi_ddot_rad_per_s2 * Delta_t_s

    def get_V_m_per_s(self):
        '''
            Computes total velocity of car.
        '''
        return sqrt(pow(self.vx_m_per_s, 2) + pow(self.vy_m_per_s, 2))

    def get_beta_rad(self):
        '''
            Computes sideslip angle (β) in radians.
        '''
        velocity_angle_rad = atan2(self.vx_m_per_s, self.vy_m_per_s) # this part should be 0 if vx = 0., vy = 1.0 and should be 0.5*pi when vx = 1. and vy = 0.0
        if velocity_angle_rad < 0.0: # atan2 returns between -pi and pi, we want between 0 and 2pi
            velocity_angle_rad += 2 * pi

        beta_rad = velocity_angle_rad - self.psi_rad 

        # now need to make sure result is within -pi to pi
        if beta_rad > pi:
            N = ceil((beta_rad - pi) / (2*pi))
            beta_rad -= N * 2*pi
        elif beta_rad < -pi:
            N = ceil(-(beta_rad + pi) / (2*pi))
            beta_rad += N * 2*pi

        return beta_rad