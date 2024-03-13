#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""
from math import sqrt, atan2, sin, cos
import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx = 0
        min_dist = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                self._waypoints[i][0] - self._current_x,
                self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints) - 1:
            self._desired_speed = self._waypoints[min_idx][2]
            return min_idx
        else:
            self._desired_speed = self._waypoints[-1][2]
            x = -1
            return x

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        min_index = self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################

        # Throttle to engine torque
        a_0 = 400
        a_1 = 0.1
        a_2 = -0.0002

        # Gear ratio, effective radius, mass + inertia
        GR = 0.35
        r_e = 0.3
        J_e = 10
        m = 2000
        g = 9.81

        # Aerodynamic and friction coefficients
        c_a = 1.36
        c_r1 = 0.01

        #PID Gains
        k_p = 1 / 1.13
        k_i = k_p / 10

        #latteral controller gains
        k = 2
        k_s = 10
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """

        self.vars.create_var('sum_integral', 0.0)
        self.vars.create_var('steer_output_old', 0.0)
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('v_total_error', 0.0)
        self.vars.create_var('v_previous_error', 0.0)
        self.vars.create_var('t_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################

            # ====================================================
            # feed forward controller
            # ====================================================
            # calculate F_load and T_e respectively
            # F_load calculations
            f_aero = c_a * (v_desired ** 2)
            r_x = c_r1 * v_desired
            f_g = m * g * np.sin(0)
            f_load = f_aero + r_x + f_g

            # T_e calculation (assuming t_e = t_load)
            t_e = GR * r_e * f_load

            # calculate engine speed w_e
            w_e = v_desired / (GR * r_e)

            # now update throttle according to the updated engine speed w_e
            throttle_forward = t_e / (a_0 + (a_1 * w_e) + (a_2 * w_e ** 2))

            # ====================================================
            # feedback controller
            # ====================================================

            sample_time = 1 / 30                                                         # time step = 1 / FPS
            v_error = v_desired - v
            self.vars.sum_integral = self.vars.sum_integral + ( v_error * sample_time )  #integration term is turned into submission
            throttle_feedback = ( k_p * v_error ) + ( k_i * self.vars.sum_integral )

            throttle_output = throttle_feedback + throttle_forward
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            Kp_ld = 0.8
            min_ld = 10
            L = 3

            x_rear = x - L * cos(yaw) / 2
            y_rear = y - L * sin(yaw) / 2
            lookahead_distance = max(min_ld, Kp_ld * v)
            #print(lookahead_distance)
            #print (yaw,"   ",path_heading,"   ",heading_error,"   ",cross_track_error_term,"   ",steer_output)

            for wp in waypoints:
                dist = sqrt((wp[0] - x_rear)**2 + (wp[1] - y)**2)
                if dist > lookahead_distance:
                    carrot = wp
                    break
            else:
                carrot = waypoints[0]
            alpha = atan2(carrot[1] - y_rear, carrot[0] - x_rear) - yaw

            # Change the steer output with the lateral controller.
            steer_output    = atan2(2 * L * sin(alpha), lookahead_distance)

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

            print (yaw,"   ",lookahead_distance,"  ",steer_output)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
       # self.vars.v_total_error = v_total_error # From PID implementation
       # self.vars.v_previous_error = v_current_error # From PID implementation
        self.vars.t_previous = t
        self.vars.steer_output_old = steer_output # Stanley controller
