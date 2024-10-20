import math

# Constants
gravitational_acceleration = 9.81  # m/s²
air_density = 1.225  # kg/m³
theta = 0  # Road slope in radians
c_rr = 0.01  # Rolling resistance coefficient
c_d = 0.32  # Drag coefficient
frontal_area = 2.2  # m²
m = 2000  # Mass of the vehicle in kg
engine_efficiency = 0.3  # Engine efficiency
epsilon = 34.2e6  # Energy content of fuel in J/L
idling_fuel_rate = 0.9 / 3600  # Idling fuel consumption rate (liters per second, assume 0.9 L/h)


# Function to calculate rolling resistance force
def rolling_resistance_force(c_rr, m, g):
    return c_rr * m * g


# Function to calculate drag resistance force
def drag_resistance_force(v, c_d, A):
    return 0.5 * air_density * v ** 2 * c_d * A


# Function to calculate gravitational resistance force
def gravitational_force(m, g, theta):
    return m * g * math.sin(theta)


# Function to calculate inertial resistance force
def inertial_resistance_force(m, a):
    return m * a


# Main function to calculate fuel consumption rate
def calculate_fuel_consumption(speed, acceleration):
    # Check if the vehicle is idling (speed == 0)
    if speed == 0:
        # Return idling fuel consumption rate
        return idling_fuel_rate * 3600  # Convert to liters per hour

    # Normal driving fuel consumption calculation
    F_rr = rolling_resistance_force(c_rr, m, gravitational_acceleration)
    F_d = drag_resistance_force(speed, c_d, frontal_area)
    F_g = gravitational_force(m, gravitational_acceleration, theta)

    # Inertial force might be negative during deceleration
    F_i = inertial_resistance_force(m, acceleration)

    # Total force should include all forces except negative inertial forces
    total_force = F_rr + F_d + F_g + F_i

    # Even if decelerating, fuel consumption still happens due to the other forces
    # total_force += max(F_i, 0)  # Add only positive inertial forces

    # Power required (in watts)
    power_required = total_force * speed

    # Fuel consumption rate (liters per second)
    fuel_consumption_rate = power_required / (engine_efficiency * epsilon)

    # Ensure a minimum fuel consumption for idling even if power is very low
    fuel_consumption_rate = max(fuel_consumption_rate, idling_fuel_rate)

    # Convert to liters per hour
    return fuel_consumption_rate * 3600
