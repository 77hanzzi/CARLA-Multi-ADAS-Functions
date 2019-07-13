#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math

import numpy as np

import carla
from agents.tools.misc import distance_vehicle, get_speed

class SpeedDistance_VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, vehicle, front_vehicle,
                 args_lateral={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0},
                 args_longitudinal={'K_Pv': 1.0, 'K_Dv': 0.0, 'K_Iv': 0.0,
                                    'K_Pd': 1.0, 'K_Dd': 0.0, 'K_Id': 0.0}):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller using the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal PID controller using the following
        semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        """
        self._vehicle = vehicle
        self._front_vehicle = front_vehicle
        self._world = self._vehicle.get_world()
        self._lon_controller = SpeedDistance_PIDLongitudinalController(
            self._vehicle, self._front_vehicle,**args_longitudinal)
        self._lat_controller = SpeedDistance_PIDLateralController(
            self._vehicle,  **args_lateral)

    def run_step(self, target_speed, target_distance, target_waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: distance (in meters) to the waypoint
        """
        throttle = self._lon_controller.run_step(target_speed,target_distance)
        steering = self._lat_controller.run_step(target_waypoint)

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class SpeedDistance_PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """
 
    def __init__(self, vehicle, front_vehicle, K_Pv=1.0, K_Dv=0.0, K_Iv=0.0, K_Pd=0.0, K_Dd=0.0, K_Id=0.0 ,dt=0.03):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._front_vehicle = front_vehicle
        self._K_Pv = K_Pv
        self._K_Dv = K_Dv
        self._K_Iv = K_Iv
        self._K_Pd = K_Pd
        self._K_Dd = K_Dd
        self._K_Id = K_Id       
        self._dt = dt
        self._ev_buffer = deque(maxlen=30)
        self._ed_buffer = deque(maxlen=30)
    

    def run_step(self, target_speed, target_distance, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in Km/h
        :return: throttle control in the range [0, 1]
        """
        current_speed = get_speed(self._vehicle)
        if self._front_vehicle is not None:
            dx = self._vehicle.get_location().x - self._front_vehicle.get_location().x
            dy = self._vehicle.get_location().y - self._front_vehicle.get_location().y
            current_distance = math.sqrt(dx * dx + dy * dy) 
            if debug:
                print('Current speed = {}'.format(current_speed))

            return self._pid_control(target_speed, current_speed, target_distance, current_distance)
        
        else:
            return self._pid_control(target_speed, current_speed, 0.0, 0.0)

    def _pid_control(self, target_speed, current_speed, target_distance, current_distance):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        #  Processing the delta speed.
        _ev = (target_speed - current_speed)
        self._ev_buffer.append(_ev)

        if len(self._ev_buffer) >= 2:
            _de_v = (self._ev_buffer[-1] - self._ev_buffer[-2]) / self._dt
            _ie_v = sum(self._ev_buffer) * self._dt
        else:
            _de_v = 0.0
            _ie_v = 0.0
        
        #  Processing the delta distance.
        _ed = (target_distance - current_distance)
        self._ed_buffer.append(_ed)

        if len(self._ed_buffer) >= 2:
            _de_d = (self._ed_buffer[-1] - self._ed_buffer[-2]) / self._dt
            _ie_d = sum(self._ed_buffer) * self._dt
        else:
            _de_d = 0.0
            _ie_d = 0.0        

        #  Processing the error of speed and distance together.
        output = (self._K_Pv * _ev + self._K_Pd * _ed) + (self._K_Dv * _de_v / self._dt + self._K_Dd * _de_d / self._dt) + \
                                                    (self._K_Iv * _ie_v * self._dt + self._K_Id * _ie_d * self._dt)
        return np.clip(output, 0.0, 1.0)


class SpeedDistance_PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoint):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x -
                          v_begin.x, waypoint.transform.location.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                         (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _dot) + (self._K_D * _de /
                                             self._dt) + (self._K_I * _ie * self._dt), -1.0, 1.0)


