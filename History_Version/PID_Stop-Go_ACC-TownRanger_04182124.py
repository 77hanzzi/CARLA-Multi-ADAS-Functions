#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
    Example of automatic vehicle control from client side.
"""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import weakref
from collections import deque
import numpy as np
import xlwt
import time
try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
from agents.navigation.agent import *
from agents.navigation.roaming_agent import *
# from agents.navigation.basic_agent import *
from agents.tools.misc import distance_vehicle, get_speed
from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import *
from enum import Enum

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name

# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    def __init__(self, carla_world, hud):
        self.world = carla_world
        self.hud = hud
        self.vehicle = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager._index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager._transform_index if self.camera_manager is not None else 0

        blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.tesla.*'))
        # blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.ford.mustang'))
        # blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.bmw.grandtourer'))        
        blueprint.set_attribute('role_name', 'hero')
        # blueprint.set_attribute('color', '120,0,0')
        blueprint.set_attribute('color', '0,200,200')

        # if blueprint.has_attribute('color'):
        #     color = random.choice(blueprint.get_attribute('color').recommended_values)
        #     blueprint.set_attribute('color', color)

        # Spawn the vehicle.
        if self.vehicle is not None:
            spawn_point = self.vehicle.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()

            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = spawn_points[1]
            self.vehicle = self.world.spawn_actor(blueprint, spawn_point)

        while self.vehicle is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = spawn_points[7]
            # Town 01 Critical Point
            spawn_point = carla.Transform(carla.Location(x=1.54, y= 109.40,z=1.37))
            # spawn_point = random.choice(spawn_points)
            self.vehicle = self.world.spawn_actor(blueprint, spawn_point)
    
            # support_actor = random.choice(self.world.get_actors().filter("*vehicle*"))
            # support_actor_point = support_actor.get_transform()
            # spawn_point = support_actor_point
            # spawn_point.location.y = spawn_point.location.y - 10.0
            # spawn_point = carla.Transform(carla.Location(x=373.40, y=-8.7, z=0.40), carla.Rotation(pitch=0, yaw=-181.00004, roll=0))
            # Tesla spawn parameter
            # spawn_point_1 = carla.Transform(carla.Location(x=-2.10 ,y=-135, z=0.80), carla.Rotation(pitch=0, yaw=90.0, roll=0))
            spawn_point_2 = carla.Transform(carla.Location(x=-2410 ,y=12.25, z=0.40), carla.Rotation(pitch=0, yaw=0, roll=0))


            # spawn_point.rotation.roll = 90.0
            # spawn_point.rotation.pitch = 90.0
                        
            # spawn_point = carla.Transform (carla.Location(x=232,y=160,z=2),carla.Rotation(roll=0,pitch=0,yaw=180))
            # self.vehicle = self.world.try_spawn_actor(blueprint, spawn_point_2)
        
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.vehicle, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.vehicle, self.hud)
        self.camera_manager = CameraManager(self.vehicle, self.hud)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.vehicle)
        self.hud.notification(actor_type)
    
    # def record(self,ve-
    #     global counter-
    #     name = str(get_actor_display_name(vehicle, truncate=20))
    #     velocity = vehicle.get_velocity()
    #     speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    #     wb = xlwt.Workbook()
    #     sheet1 = wb.add_sheet(name)
    #     row = counter
    #     sheet1.write(row,0,'jajaja')
    #     wb.save('record_2.xlsx')


    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.vehicle.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.vehicle]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        world.vehicle.set_autopilot(self._autopilot_enabled)
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                elif event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self._control.gear = world.vehicle.get_vehicle_control().gear
                    world.hud.notification(
                        '%s Transmission' % ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.vehicle.set_autopilot(self._autopilot_enabled)
                    world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled:
            self._parse_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0

    def _parse_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)



# ==============================================================================
# -- TrafficDetector ---------------------------------------------------------------
# ==============================================================================
class TrafficDetector(object):
    
    def __init__(self, vehicle, target_waypoint, sign_distance, vehicle_distance):
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._last_traffic_light = None
        self._lightslist = self._world.get_actors().filter("*traffic_light*")
        self._vehiclelist = self._world.get_actors().filter("*vehicle*")
        self._speedlimit_list = self._world.get_actors().filter("*speed_limit*")
        self._sign_distance = sign_distance
        self._vehicle_distance = vehicle_distance
        self._target_waypoint = target_waypoint
        self._last_us_traffic_light = None
        self._speedlimit_history = []
        self._speedlimit_history.append("Init")
    
    def search_front_vehicle(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,distance):        
                # print (target_vehicle.id == vehicle_list[0])
                return (True,target_vehicle)
        return (False,None)
    
    def define_hazard_vehicle(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,15.0):        
                return (True,target_vehicle)
        return (False,None)

    def get_speed_limit(self):
        """

        """
        vehicle = self._vehicle
        current_map = self._map
        speed_limit_list = self._speedlimit_list
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        # while Ture:

        if len(self._speedlimit_history) > 4000:
            last_value = self._speedlimit_history[-1]
            self._speedlimit_history.pop(0)
            self._speedlimit_history.append(last_value)

        for speed_limit in speed_limit_list:
            # print (len(speed_limit_list))
            # speed_limit_history.append("a")

            object_waypoint = current_map.get_waypoint(speed_limit.get_location())
            if object_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            object_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = speed_limit.get_location()
            if is_within_distance_ahead(loc, ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,
                                        20.0):
            # if traffic_light.state == carla.libcarla.TrafficLightState.Red:
                speed_limit_value = (str(speed_limit).split('.'))[-1]
                speed_limit_value = speed_limit_value[0] + speed_limit_value[1]
                # print (type(speed_limit_value))
                self._speedlimit_history.append(speed_limit_value)            
                # print (speed_limit_value + str(len(speed_limit_history)))
                return (self._speedlimit_history[-1])
    
        # print (str(len(speed_limit_history)))
        return (self._speedlimit_history[-1])


    def is_light_red(self):
        """
        Method to check if there is a red light affecting us. This version of
        the method is compatible with both European and US style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
        """
        world = self._world
        if world.map_name == 'Town01' or world.map_name == 'Town02':
            return self._is_light_red_europe_style()
        else:
            return self._is_light_red_us_style()



    def _is_light_red_europe_style(self):
        """
        This method is specialized to check European style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
        """
        vehicle = self._vehicle
        lights_list = self._lightslist
        current_map = self._map    
        distance = self._sign_distance
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            object_waypoint = current_map.get_waypoint(traffic_light.get_location())
            if object_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            object_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = traffic_light.get_location()
            if is_within_distance_ahead(loc, ego_vehicle_location,
                                        vehicle.get_transform().rotation.yaw,
                                        distance):
                if traffic_light.state == carla.libcarla.TrafficLightState.Red:
                    return (True, traffic_light)

        return (False, None)

    
    def _is_light_red_us_style(self):
        """
        This method is specialized to check US style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
                    - bool_flag is True if there is a traffic light in RED
                    affecting us and False otherwise
                    - traffic_light is the object itself or None if there is no
                    red traffic light affecting us
        """
        last_us_traffic_light = self._last_us_traffic_light
        vehicle = self._vehicle
        lights_list = self._lightslist
        current_map = self._map 
        target_waypoint = self._target_waypoint
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)

        if ego_vehicle_waypoint.is_intersection:
            # It is too late. Do not block the intersection! Keep going!
            return (False, None)

        if target_waypoint is not None:            
            suitable_waypoint = optimize_target_waypoint(current_map,target_waypoint,vehicle.get_transform().rotation.yaw)
            if suitable_waypoint.is_intersection:
            # if self._local_planner._target_waypoint.is_intersection:
                # potential_lights = []
                min_angle = 180.0
                sel_magnitude = 0.0
                sel_traffic_light = None
                for traffic_light in lights_list:
                    loc = traffic_light.get_location()
                    magnitude, angle = compute_magnitude_angle(loc,
                                                                ego_vehicle_location,
                                                                vehicle.get_transform().rotation.yaw)
                    if magnitude < 80.0 and angle < min(25.0, min_angle):
                    # if magnitude < 280.0 and angle < 40.0:

                        sel_magnitude = magnitude
                        sel_traffic_light = traffic_light
                        min_angle = angle

                if sel_traffic_light is not None:
                    print('=== Magnitude = {} | Angle = {} | ID = {} | Status = {}'.format(sel_magnitude, min_angle, sel_traffic_light.id, sel_traffic_light.state ))

                    if last_us_traffic_light is None:
                        last_us_traffic_light = sel_traffic_light

                    if last_us_traffic_light.state == carla.libcarla.TrafficLightState.Red:
                        return (True, last_us_traffic_light)
                else:
                    last_us_traffic_light = None

        return (False, None)


# ==============================================================================
# -- ACC_Controller -----------------------------------------------------------
# ==============================================================================

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4


class ACC_Controller(object):
    
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self,vehicle, opt_dict={}):
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._last_us_traffic_light = None
        self._dt = None
        self._target_speed = None
        self._sampling_radius = None
        self._min_distance = None
        self._current_waypoint = None
        self._target_road_option = None
        self._next_waypoints = None
        self._target_waypoint = None
        self._vehicle_controller = None
        self._global_plan = None
        self._trigger_counter = 0
        # queue with tuples of (waypoint, RoadOption)
        self._waypoints_queue = deque(maxlen=600)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        self._lightslist = self._world.get_actors().filter("*traffic_light*")
        self._vehiclelist = self._world.get_actors().filter("*vehicle*")

        # SETTING controller
        self.preset_speed = 30.0
        self.set_PID_controller(opt_dict)

        self._traffic_detector = TrafficDetector(self._vehicle,self._target_waypoint,12.5,10.0)
    
    def refresh_traffic_detector(self):
        self._traffic_detector._target_waypoint = self._target_waypoint
    
    def set_sampling_radius(self):
        if get_speed(self._vehicle) <= 15.0:
            return 2.0  # 0.5 seconds horizon

        elif get_speed(self._vehicle)>15.0 and get_speed(self._vehicle) <= 45.0:
            return get_speed(self._vehicle) * 0.5 / 3.8  # 0.5 seconds horizon
        elif get_speed(self._vehicle) > 45.0 and get_speed(self._vehicle) <= 65.0:
            return get_speed(self._vehicle) * 0.5 / 3.2
        elif get_speed(self._vehicle) > 65.0:
            return 12.0
    
    def emergency_brake_situation(self):
        emergency_1 = False
        emergency_2 = False
        hazard_vehicle_state, hazard_vehicle = self._traffic_detector.define_hazard_vehicle()               
        light_state, traffic_light = self._traffic_detector.is_light_red()
        if hazard_vehicle_state:
            dx = hazard_vehicle.get_location().x - self._vehicle.get_location().x
            dy = hazard_vehicle.get_location().y - self._vehicle.get_location().y
            distance = math.sqrt(dx * dx + dy * dy )
            if get_speed(hazard_vehicle) <= 0.30 * get_speed(self._vehicle) or get_speed(hazard_vehicle) < 3.0 or distance < 6.0:
                emergency_1 = True
        if light_state:
            emergency_2 = True
        if (emergency_1 is True) or (emergency_2 is True) or (emergency_1 and emergency_2 is True):
            return True
        return False

    def set_target_speed(self):
        # vehicle_state, target_vehicle = self.search_front_vehicle()                
        # print(vehicle_state)
        vehicle_state, target_vehicle = self._traffic_detector.search_front_vehicle()                
        # print (vehicle_state)
        if vehicle_state is True:
            return get_speed(target_vehicle) * 0.95
        else:
            if self._traffic_detector.get_speed_limit() != 'Init':
                if self._traffic_detector.get_speed_limit() == 90:
                    return float(self._traffic_detector.get_speed_limit()) * 0.8
                else:
                    return float(self._traffic_detector.get_speed_limit()) * 0.8
        
        return self.preset_speed


    def emergency_stop(self):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False

        return control        

    
    def set_PID_controller(self, opt_dict):
        self._dt = 1.0 / 20.0
        self._target_speed = self.preset_speed  # Km/h
        self._sampling_radius = 4.0
        # self._sampling_radius = self._target_speed * 0.5 / 2.8  # 0.5 seconds horizon
        self._min_distance = self._sampling_radius * self.MIN_DISTANCE_PERCENTAGE
        args_lateral_dict = {
            'K_P': 1.12,
            'K_D': 0.001,
            'K_I': 1.17,
            'dt': self._dt}
        args_longitudinal_dict = {
            'K_P': 1.2,
            'K_D': 0.0,
            'K_I': 1.0,
            'dt': self._dt}             
        if 'dt' in opt_dict:
            self._dt = opt_dict['dt']
        if 'target_speed' in opt_dict:
            self._target_speed = opt_dict['target_speed']
        if 'sampling_radius' in opt_dict:
            self._sampling_radius = self._target_speed * \
                opt_dict['sampling_radius'] / 3.6
        if 'lateral_control_dict' in opt_dict:
            args_lateral_dict = opt_dict['lateral_control_dict']
        if 'longitudinal_control_dict' in opt_dict:
            args_longitudinal_dict = opt_dict['longitudinal_control_dict']

        self._current_waypoint = self._map.get_waypoint(
            self._vehicle.get_location())

        self._vehicle_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=args_lateral_dict,
                                                        args_longitudinal=args_longitudinal_dict)

        self._global_plan = False

        # compute initial waypoints
        self._waypoints_queue.append( (self._current_waypoint.next(self._sampling_radius)[0], RoadOption.LANEFOLLOW))
        self._target_road_option = RoadOption.LANEFOLLOW
        # fill waypoint trajectory queue
        self._compute_next_waypoints(k=200)        




    def _compute_next_waypoints(self, k=1):
        """
        Add new waypoints to the trajectory queue.
        Calculate each waypoint in the path and work out the corresponding operation.

        :param k: how many waypoints to compute
        :return:
        
        """
        
        # check we do not overflow the queue
        available_entries = self._waypoints_queue.maxlen - len(self._waypoints_queue)
        k = min(available_entries, k)

        for _ in range(k):
            last_waypoint = self._waypoints_queue[-1][0]
            next_waypoints = list(last_waypoint.next(self._sampling_radius))

            
            if len(next_waypoints) == 1:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                road_option = RoadOption.LANEFOLLOW
            else:
                # random choice between the possible options
                road_options_list = retrieve_options(
                    next_waypoints, last_waypoint)
                road_option = random.choice(road_options_list)
                next_waypoint = next_waypoints[road_options_list.index(
                    road_option)]
            
            # print (road_option)
            self._waypoints_queue.append((next_waypoint, road_option))
            # print (str(self._waypoints_queue[1].name))
    
    def run_step(self, debug=True):
        """
        Execute one step of local planning which involves running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

        :param debug: boolean flag to activate waypoints debugging
        :return:
        """
        self.refresh_traffic_detector()
        # print (self._sampling_radius)
        # not enough waypoints in the horizon? => add more!
        if len(self._waypoints_queue) < int(self._waypoints_queue.maxlen * 0.5):
            # if not self._global_plan:
            self._compute_next_waypoints(k=100)

        if len(self._waypoints_queue) == 0:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.0
            control.hand_brake = False
            control.manual_gear_shift = False

            return control

        #   Buffering the waypoints
        if not self._waypoint_buffer:
            for i in range(self._buffer_size):
                if self._waypoints_queue:
                    self._waypoint_buffer.append(
                        self._waypoints_queue.popleft())
                else:
                    break
        # self._target_speed = 60.0
        self._sampling_radius = self.set_sampling_radius()  # 0.5 seconds horizon
        # print(self._sampling_radius)
        self._traffic_detector.get_speed_limit()
        # current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        # target waypoint
        self._target_waypoint, self._target_road_option = self._waypoint_buffer[0]
        self._target_speed = self.set_target_speed()

        # move using PID controllers
        # print(self._traffic_detector.get_speed_limit())
        
        # light_state, traffic_light = self._traffic_detector._is_light_red_us_style()
        if self.emergency_brake_situation():
            control = self.emergency_stop()
        else:
            control = self._vehicle_controller.run_step(self._target_speed, self._target_waypoint)

        # purge the queue of obsolete waypoints
        vehicle_transform = self._vehicle.get_transform()
        max_index = -1

        for i, (waypoint, _) in enumerate(self._waypoint_buffer):
            if distance_vehicle(
                    waypoint, vehicle_transform) < self._min_distance:
                max_index = i
        
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        if debug:
            draw_waypoints(self._vehicle.get_world(), [self._target_waypoint], self._vehicle.get_location().z + 1.0)

        return control

def retrieve_options(list_waypoints, current_waypoint):
    """
    Compute the type of connection between the current active waypoint and the multiple waypoints present in
    list_waypoints. The result is encoded as a list of RoadOption enums.

    :param list_waypoints: list with the possible target waypoints in case of multiple options
    :param current_waypoint: current active waypoint
    :return: list of RoadOption enums representing the type of connection from the active waypoint to each
            candidate in list_waypoints
    """
    options = []
    for next_waypoint in list_waypoints:
        # this is needed because something we are linking to
        # the beggining of an intersection, therefore the
        # variation in angle is small
        next_next_waypoint = next_waypoint.next(3.0)[0]
        link = compute_connection(current_waypoint, next_next_waypoint)
        options.append(link)

    return options


def compute_connection(current_waypoint, next_waypoint):
    """
    Compute the type of topological connection between an active waypoint (current_waypoint) and a target waypoint
    (next_waypoint).

    :param current_waypoint: active waypoint
    :param next_waypoint: target waypoint
    :return: the type of topological connection encoded as a RoadOption enum:
            RoadOption.STRAIGHT
            RoadOption.LEFT
            RoadOption.RIGHT
    """
    n = next_waypoint.transform.rotation.yaw
    n = n % 360.0

    c = current_waypoint.transform.rotation.yaw
    c = c % 360.0

    diff_angle = (n - c) % 180.0
    if diff_angle < 1.0:
        return RoadOption.STRAIGHT
    elif diff_angle > 90.0:
        return RoadOption.LEFT
    else:
        return RoadOption.RIGHT        

  

# ==============================================================================
# -- HUD -----------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame_number = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame_number = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        if not self._show_info:
            return
        t = world.vehicle.get_transform()
        v = world.vehicle.get_velocity()
        c = world.vehicle.get_vehicle_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame_number - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16d FPS' % self.server_fps,
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.vehicle, truncate=20),
            'Map:     % 20s' % world.world.map_name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            'Traffic_lights:  % 16d FPS' % self.server_fps,
            '',            
            '',
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            ('Manual:', c.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear),
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)
        ]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.vehicle.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))
        self._notifications.tick(world, clock)

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item: # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================

class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._history = []
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self._history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self._hud.notification('Collision with %r, id = %d' % (actor_type, event.other_actor.id))
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self._history.append((event.frame_number, intensity))
        if len(self._history) > 4000:
            self._history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================

class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self._hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_detector')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        text = ['%r' % str(x).split()[-1] for x in set(event.crossed_lane_markings)]
        self._hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._surface = None
        self._parent = parent_actor
        self._hud = hud
        self._recording = False
        self._camera_transforms = [
            # carla.Transform(carla.Location(x=0.1,y=-0.3, z=1.2), carla.Rotation(pitch=-15)),
            # carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=-8.5, z=4.8), carla.Rotation(pitch=-17)),

            carla.Transform(carla.Location(x=1.6, z=1.7)),

            ]
        self._transform_index = 1
        self._sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self._sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            item.append(bp)
        self._index = None

    def toggle_camera(self):
        self._transform_index = (self._transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self._transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self._sensors)
        needs_respawn = True if self._index is None \
            else self._sensors[index][0] != self._sensors[self._index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self._surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self._sensors[index][-1],
                self._camera_transforms[self._transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self._hud.notification(self._sensors[index][2])
        self._index = index

    def next_sensor(self):
        self.set_sensor(self._index + 1)

    def toggle_recording(self):
        self._recording = not self._recording
        self._hud.notification('Recording %s' % ('On' if self._recording else 'Off'))

    def render(self, display):
        if self._surface is not None:
            display.blit(self._surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self._sensors[self._index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._hud.dim) / 100.0
            lidar_data += (0.5 * self._hud.dim[0], 0.5 * self._hud.dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._hud.dim[0], self._hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self._sensors[self._index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self._recording:
            image.save_to_disk('_out/%08d' % image.frame_number)


# # ==============================================================================
# # -- Recorder() ---------------------------------------------------------
# # ==============================================================================
# class Recorder(object):
#     def __init__(self,vehicle,support_vehicle,controller,workbook):
#         self.vehicle = vehicle
#         self.support_vehicle = support_vehicle
#         self.world = vehicle.get_world()
#         self.workbook = workbook
#         self.counter = 1
#         self.sheetname = str(get_actor_display_name(vehicle, truncate=20))
#         self.sheet = workbook.add_sheet(self.sheetname)
#         self.vehicle_list = self.world.get_actors().filter("vehicle*")
#         self.controller = controller
#         self.workbookname =  str(time.strftime('%Y.%m.%d_%H%M%S',time.localtime(time.time()))) \
#                                     + '_ID_' + str(vehicle.id) + '.xls'
#         self.sheet.write(0,0,"Ego_Speed")
#         self.sheet.write(0,1,"Supporting_Speed")        
#         self.sheet.write(0,2,"Target_Speed")
#         self.sheet.write(0,3,"Ego_Acceleration")
#         self.sheet.write(0,4,"Supporting_Acceleration")
#         self.sheet.write(0,5,"Ego_Location_y")
#         self.sheet.write(0,6,"Support_Location_y")
#         self.sheet.write(0,7,"Support_Yaw_Angle")
#         self.sheet.write(0,8,"Ego_Yaw_Angle")
#         self.sheet.write(0,9,"Throttle")
#         self.sheet.write(0,10,"Steer")
#         self.sheet.write(0,11,"Brake")
#         self.sheet.write(0,12,"Relative_distance")

    
#     def start_recorder(self):
#         wb = self.workbook
#         vehicle = self.vehicle
#         row = self.counter
#         worksheet = self.sheet
#         supporting_actor = self.support_vehicle
#         controller = self.controller

#         control = vehicle.get_vehicle_control()
#         ego_velocity = vehicle.get_velocity()   
#         support_velocity = supporting_actor.get_velocity()
#         target_speed = controller.export_target_speed()
#         ego_acceleration_vector = vehicle.get_acceleration()
#         sup_acceleration_vector = supporting_actor.get_acceleration()

#         s_loc = supporting_actor.get_location()
#         s_loc_x = s_loc.x
#         s_loc_y = s_loc.y
#         s_loc_z = s_loc.z
        
#         support_speed = 3.6 * math.sqrt(support_velocity.x**2 + support_velocity.y**2 + support_velocity.z**2)
#         ego_speed = 3.6 * math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
#         # target_speed = controller._lon_controller.speed_setting()

#         ego_acceleration = math.sqrt(ego_acceleration_vector.x**2 + ego_acceleration_vector.y**2 + ego_acceleration_vector.z**2)
#         sup_acceleration = math.sqrt(sup_acceleration_vector.x**2 + sup_acceleration_vector.y**2 + sup_acceleration_vector.z**2)
#         # sup_acceleration = 10
#         ego_location_x = vehicle.get_location().x
#         ego_location_y = vehicle.get_location().y
#         ego_location_z = vehicle.get_location().z
        
#         ego_yaw = vehicle.get_transform().rotation.yaw
#         support_yaw = supporting_actor.get_transform().rotation.yaw

#         throttle = control.throttle
#         steer = control.steer
#         brake = control.brake

#         distance = math.sqrt((s_loc_x - ego_location_x)**2 + (s_loc_y - ego_location_y)**2 + (s_loc_z - ego_location_z)**2)

#         worksheet.write(row,0,ego_speed)
#         worksheet.write(row,1,support_speed)        
#         worksheet.write(row,2,target_speed)
#         worksheet.write(row,3,ego_acceleration)
#         worksheet.write(row,4,sup_acceleration)
#         worksheet.write(row,5,ego_location_y)
#         worksheet.write(row,6,s_loc_y)
#         worksheet.write(row,7,support_yaw)
#         worksheet.write(row,8,ego_yaw)
#         worksheet.write(row,9,throttle)
#         worksheet.write(row,10,steer)
#         worksheet.write(row,11,brake)
#         worksheet.write(row,12,distance)

#         self.counter += 1
#         wb.save(self.workbookname)

#     def finish_recorder(self):
#         wb = self.workbook
#         wb.save()

# ==============================================================================
# -- game_loop() ---------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    supporting_actor_list = []
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud)
        # transform_1 = carla.Transform (carla.Location(x=-2.10, y=-125.30, z=0.4),carla.Rotation(pitch=0, yaw=90.0, roll=0))
        transform_2 = carla.Transform (carla.Location(x=-2410, y=5.7, z=0.4),carla.Rotation(pitch=0, yaw=0, roll=0))

        # carModel_1 = random.choice (blueprint.filter('vehicle.tesla.*')) 
        carModel_1 = world.world.get_blueprint_library().find('vehicle.tesla.model3')
        carModel_1.set_attribute('color','10,10,10')
        # carActor_1 = world.world.try_spawn_actor(carModel_1,transform_2)
        # carActor_1.set_autopilot (True)        
        # supporting_actor_list.append(carActor_1)        
        
        spawn_number = 0
        bp_stream = world.world.get_blueprint_library().filter('vehicle*')
        bp_stream = [x for x in bp_stream if int(x.get_attribute('number_of_wheels')) == 4]
        bp_stream = [x for x in bp_stream if not x.id.endswith('isetta')]
        
        for vehicle_index in range(0,spawn_number):
            vehicle_model = random.choice(bp_stream)
            vehicle_model.set_attribute('role_name','autopilot')
            vehicle_actor = world.world.try_spawn_actor(vehicle_model,random.choice(world.world.get_map().get_spawn_points()))
            if vehicle_actor is not None:
                supporting_actor_list.append(vehicle_actor)
                # vehicle_actor.apply_control (carla.VehicleControl (throttle = 0.5, steer=0.0, brake =0.0) )
                vehicle_actor.set_autopilot(True)
               
            # print (vehicle_actor.get_transform())

        controller = KeyboardControl(world, False)
        
        hero = world.vehicle
        ACC_controller =ACC_Controller(hero)


        
        # if args.agent == "Roaming":
        #     agent = RoamingAgent(world.vehicle)
        # else:
        # agent = BasicAgent(world.vehicle)
        # spawn_point = world.world.get_map().get_spawn_points()[0]
        # agent.set_destination((spawn_point.location.x,
        #                         spawn_point.location.y,
        #                         spawn_point.location.z))

        clock = pygame.time.Clock()
        
        # this vehicle list only has 1 member
        vehicle_list_1 = world.world.get_actors().filter("*vehicle*")
        # this vehicle list has 2 members
        # vehicle_list_2 = PID_contoller._world.get_actors().filter("*vehicle*")
        
        # if len(vehicle_list) > 1:
        #     for index in vehicle_list:
        #         if index.id != world.vehicle.id:
        #             front_vehicle = index
        #         else:
        #             pass 
        # else:
        #     front_vehicle = hero
        
        # way_point = world.world.get_map().get_waypoint(front_vehicle.get_location())
        # target_transform = carla.Transform (carla.Location(x=-500, y=-7.7, z=0.4),carla.Rotation(pitch=0, yaw=-179.000000, roll=0))
        # spawn_point = world.world.get_map().get_spawn_points()[0]
        # way_point = world.world.get_map().get_waypoint(spawn_point.location)
        # wb = xlwt.Workbook()
        # ego_recorder = Recorder(hero,front_vehicle,PID_contoller,wb)
        while True:
            # print (way_point)
            if controller.parse_events(world, clock):
                return

            # as soon as the server is ready continue!
            if not world.world.wait_for_tick(10.0):
                continue
            # print (front_vehicle.get_location())
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            control = ACC_controller.run_step()
            # control = agent.run_step()

            world.vehicle.apply_control(control)
            # ego_recorder.start_recorder()
            # print (counter)
            
            # sheet1.write(counter,0,'haha')
            # counter = counter + 1 
            # wb.save('xlwt example_4.xlsx')

    finally:
      
        if world is not None:
            world.destroy()

        print('\ndestroying %d actors' % len(supporting_actor_list))
        for actor in supporting_actor_list:
            actor.destroy()  

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')

    argparser.add_argument("-a", "--agent", type=str,
                           choices=["Roaming", "Basic"],
                           help="select which agent to run",
                           default="Basic")
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)
    actor_list = []

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)
    
    finally:
        print('\ndestroying %d actors' % len(actor_list))
        for actor in actor_list:
            actor.destroy()


if __name__ == '__main__':

    main()
