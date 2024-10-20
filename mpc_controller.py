import numpy as np
from scipy.optimize import minimize
import carla

# Constants for the vehicle model
L_f = 2.5  # Distance between the center of mass and the front axle (vehicle wheelbase)
dt = 0.1  # Time step for the prediction (seconds)
N = 10  # Prediction horizon


# MPC class that controls the ego vehicle using model predictive control
class MPCController:
    def __init__(self, target_speed):
        self.target_speed = target_speed  # Desired speed (in km/h)
        self.prev_delta = 0.0  # Previous steering angle
        self.prev_a = 0.0  # Previous acceleration

    # Kinematic bicycle model to predict the vehicle's next state
    def predict_vehicle_state(self, state, delta, a, dt):
        """
        Predicts the next vehicle state using the kinematic bicycle model.
        :param state: Current state [x, y, yaw, v] (position, heading, speed)
        :param delta: Steering angle
        :param a: Acceleration (throttle input)
        :param dt: Time step for the prediction
        :return: Next state [x, y, yaw, v]
        """
        x, y, yaw, v = state
        x_next = x + v * np.cos(yaw) * dt
        y_next = y + v * np.sin(yaw) * dt
        yaw_next = yaw + v * delta / L_f * dt
        v_next = v + a * dt
        return [x_next, y_next, yaw_next, v_next]

    # Objective function to minimize (used by the optimizer)
    def objective(self, vars, *args):
        """
        Objective function to minimize (sum of errors).
        :param vars: Array of variables (steering angles, accelerations)
        :param args: Additional arguments (current state, waypoints)
        :return: Cost (objective function value)
        """
        state, waypoints = args
        cost = 0
        x, y, yaw, v = state
        for t in range(N):
            delta = vars[t]
            a = vars[t + N]
            state = self.predict_vehicle_state(state, delta, a, dt)
            x_ref, y_ref = waypoints[t]
            cost += (state[0] - x_ref) ** 2 + (state[1] - y_ref) ** 2  # Cross-track error
            cost += (state[3] - self.target_speed) ** 2  # Speed error
        return cost

    # Constraints for the optimization (control input limits)
    def constraints(self, vars):
        """
        Constraint function to ensure that the control inputs are within limits.
        :param vars: Array of variables (steering angles, accelerations)
        :return: List of constraint violations (empty if all constraints are satisfied)
        """
        delta_constraints = np.clip(vars[:N], -np.radians(25),
                                    np.radians(25))  # Steering angle constraints (-25 to 25 degrees)
        a_constraints = np.clip(vars[N:], -1.0, 1.0)  # Throttle constraints (-1.0 to 1.0)
        return np.concatenate((delta_constraints, a_constraints))

    # Main MPC control function
    def compute_control(self, current_state, waypoints):
        """
        Computes the optimal control inputs (steering and throttle) using MPC.
        :param current_state: Current state of the vehicle [x, y, yaw, v]
        :param waypoints: List of future waypoints [x_ref, y_ref]
        :return: Optimal steering angle and throttle
        """
        # Initial guess for control inputs (previous steering and throttle values)
        vars_initial = np.zeros(2 * N)
        vars_initial[:N] = self.prev_delta
        vars_initial[N:] = self.prev_a

        # Optimization bounds
        bounds = [(-np.radians(25), np.radians(25))] * N + [
            (-1.0, 1.0)] * N  # Steering angle (-25 to 25 degrees), throttle (-1.0 to 1.0)

        # Perform the optimization
        result = minimize(
            self.objective, vars_initial, args=(current_state, waypoints),
            method='SLSQP', bounds=bounds, constraints={'type': 'eq', 'fun': self.constraints}
        )

        # Extract the optimal control inputs
        if result.success:
            optimal_delta = result.x[0]  # Optimal steering angle
            optimal_a = result.x[N]  # Optimal acceleration (throttle)
        else:
            # Use previous controls if optimization fails
            optimal_delta = self.prev_delta
            optimal_a = self.prev_a

        # Store previous control inputs for next iteration
        self.prev_delta = optimal_delta
        self.prev_a = optimal_a

        return optimal_delta, optimal_a  # Return the steering angle and throttle


# Function to apply the computed MPC control to the ego vehicle
def apply_mpc_control(mpc_controller, ego_vehicle, waypoints):
    """
    Applies the MPC control to the ego vehicle.
    :param mpc_controller: The MPCController object.
    :param ego_vehicle: The ego vehicle to be controlled.
    :param waypoints: The reference waypoints for the vehicle to follow.
    """
    # Get the current state of the ego vehicle
    transform = ego_vehicle.get_transform()
    velocity = ego_vehicle.get_velocity()
    yaw = np.radians(transform.rotation.yaw)  # Convert yaw to radians
    v = np.linalg.norm([velocity.x, velocity.y])  # Calculate speed

    # Current state [x, y, yaw, v]
    current_state = [transform.location.x, transform.location.y, yaw, v]

    # Compute optimal control inputs using MPC
    optimal_delta, optimal_a = mpc_controller.compute_control(current_state, waypoints)

    # Create the vehicle control command
    control = carla.VehicleControl()
    control.steer = float(optimal_delta / np.radians(25))  # Normalize steering to [-1, 1]
    control.throttle = float(np.clip(optimal_a, 0, 1))  # Throttle is between [0, 1]
    control.brake = float(np.clip(-optimal_a, 0, 1))  # Brake is applied if acceleration is negative

    # Apply the control to the ego vehicle
    ego_vehicle.apply_control(control)
