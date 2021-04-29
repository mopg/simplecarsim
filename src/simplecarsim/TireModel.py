from math import sqrt

class TireModel( object ):
    """
        Abstract TireModel class.

        This makes it easier to use different tire models later and only having to change code in this file (and not in Car for instance).
    """
    pass

class LinearTireModel( TireModel ):

    """
        LinearTireModel

        This class inherits from TireModel and implements the linear tire model in Section 1.3 of the document.

        Attributes:
            linear_slope        k_t in the document
    """
    def __init__(
        self,
        linear_slope = 1.75,
    ):

        self.linear_slope = linear_slope

    def compute_tire_forces(
        self,
        s_x,
        s_y,
        F_z,
    ):

        # Equation 7a
        s = sqrt(pow(s_x, 2) + pow(s_y, 2))

        if s < 1e-8:
            return 0., 0.,

        # Equation 7b through d
        F_t = F_z * self.linear_slope * s
        F_x = (s_x / s) * F_t
        F_y = (s_y / s) * F_t

        return F_x, F_y
        
        