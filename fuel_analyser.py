import cars


class FuelTank(object):
    def __init__(self, _ego: cars.Ego):
        self.history = []
        self.instantaneous_fuel_consumption = 0

        self.mass = _ego.mass

        # parameters of eq2
        self.f_e = 0.  # consumption rate (liter/s)
        self.mue = 0.  # specific fuel consumption ( kg/(kj/s) )
        self.k = 0.  # engine friction (kPa)
        self.omega_e_t = 0.  # engine speed at t time (revolution/s)'

        # parameters of eq3
        self.rho = 1.2256  # air density at sea level(kg/m3)

    # eq2
    def p_of_t(self, _ego: cars.Ego):
        s = _ego.speed
        a = _ego.acceleration
        m = _ego.mass
        r_t = self.r_of_t(_ego)
        epsilon = _ego.gear

        # return (
        #     self.r_of_t(_ego) + _ego.mass * _ego.acceleration *
        #     (1.04 + 0.0025 * )
        #     /
        #     (3600 * )
        # )

    # eq3
    def r_of_t(self, _ego: cars.Ego):
        c_d = _ego.drag
        m = _ego.mass
        c_h = 0
        a_f = 1  # vehicle frontal area
        c_r = 1.75  # rolling coefficient
        c_1 = .0328
        c_2 = 4.575
        g_t = 0
        v_t = _ego.speed_mps

        return (
                (self.rho / 25.92) * c_d * c_h * a_f
                +
                (9.8066 * c_r / 1000) * c_1 * (v_t ** 2) + c_2
                +
                9.8066 * m * g_t
        )

    # eq4
    def fuel_consumption_of_t(self, _ego: cars.Ego):
        pass

    # eq7
    @staticmethod
    def alpha_0(_ego: cars.Ego):
        p_mfo = 400000  # idling fuel pressure (Pa)
        omega_idling = 650  # idling rpm
        d = 2.7  # motor displacement (liters)
        q = 43000000  # fuel lowe heating (j/kg)
        n = 6  # number of cylinders

        return (
                p_mfo * omega_idling * d
                /
                (22164 * q * n)
        )

    def alpha_0_min(self, _ego: cars.Ego):
        pass

    def alpha_0_max(self, _ego: cars.Ego):
        pass

    def f_city(self, _ego: cars.Ego):
        pass

    def f_hwy(self, _ego: cars.Ego):
        pass


    def total_resistance_force(self, t: float):
        pass

    def on_tick(self, _ego: cars.Ego):
        pass
