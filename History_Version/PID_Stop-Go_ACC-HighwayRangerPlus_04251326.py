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
# from agents.navigation.agent import *
# from agents.navigation.roaming_agent import *
# from agents.navigation.basic_agent import *
from agents.tools.misc import distance_vehicle, get_speed
from agents.navigation.SpeedDistance_controller import SpeedDistance_VehiclePIDController
from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import *
from enum import Enum
import networkx as nx

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
        self._lane_history = []
        self._speedlimit_history = []
        self._speedlimit_history.append("Init")
    
    def record_lane_id(self):
        vehicle = self._vehicle
        current_map = self._map
        speed_limit_list = self._speedlimit_list
        ego_vehicle_location = vehicle.get_location()
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        self._lane_history.append(ego_vehicle_waypoint.lane_id)
    
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
    
    def search_rear_vehicle_left_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y -3.5
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
                                        180.0 + vehicle.get_transform().rotation.yaw,50.0):        
                # print (target_vehicle.id == vehicle_list[0])
                return (True,target_vehicle)
        return (False,None)  

    def search_front_vehicle_left_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y -3.5
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
                                        vehicle.get_transform().rotation.yaw,50.0):        
                # print (target_vehicle.id == vehicle_list[0])
                return (True,target_vehicle)
        return (False,None)    
        
    def search_rear_vehicle_right_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y +3.5
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        # shadow_location = carla.Location(x= ego_vehicle_location.x, y= ego_vehicle_location.y + 3.5, z= ego_vehicle_location.z)
        # shadow_waypoint = self._map.get_waypoint(shadow_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue
            # print (target_vehicle)
            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        180.0 + vehicle.get_transform().rotation.yaw,50.0):        
                return (True,target_vehicle)
        return (False,None)       
    
    def search_front_vehicle_right_lane(self):
        vehicle = self._vehicle
        vehicle_list = self._vehiclelist
        current_map = self._map  
        distance = self._vehicle_distance  
        ego_vehicle_location = vehicle.get_location()        
        ego_vehicle_location.y = ego_vehicle_location.y +3.5
        ego_vehicle_waypoint = current_map.get_waypoint(ego_vehicle_location)
        # shadow_location = carla.Location(x= ego_vehicle_location.x, y= ego_vehicle_location.y + 3.5, z= ego_vehicle_location.z)
        # shadow_waypoint = self._map.get_waypoint(shadow_location)
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint =  current_map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                            target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue
            # print (target_vehicle)
            loc = target_vehicle.get_location()

            if is_within_distance_ahead(loc,ego_vehicle_location,
                                        180.0 + vehicle.get_transform().rotation.yaw,50.0):        
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

        # blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.tesla.*'))
        blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.tesla.model3'))
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
            # spawn_point = spawn_points[10]
            spawn_point = random.choice(spawn_points)
            # self.vehicle = self.world.spawn_actor(blueprint, spawn_point)
    
            # support_actor = random.choice(self.world.get_actors().filter("*vehicle*"))
            # support_actor_point = support_actor.get_transform()
            # spawn_point = support_actor_point
            # spawn_point.location.y = spawn_point.location.y - 10.0
            # spawn_point = carla.Transform(carla.Location(x=373.40, y=-8.7, z=0.40), carla.Rotation(pitch=0, yaw=-181.00004, roll=0))
            # Tesla spawn parameter
            # spawn_point_1 = carla.Transform(carla.Location(x=-2.10 ,y=-135, z=0.80), carla.Rotation(pitch=0, yaw=90.0, roll=0))
            spawn_point_2 = carla.Transform(carla.Location(x=-2420 ,y=12.25, z=0.40), carla.Rotation(pitch=0, yaw=0, roll=0))
            spawn_point_3 = carla.Transform(carla.Location(x=2420 ,y=-8.25, z=0.40), carla.Rotation(pitch=0, yaw=180, roll=0))


            # spawn_point.rotation.roll = 90.0
            # spawn_point.rotation.pitch = 90.0
                        
            # spawn_point = carla.Transform (carla.Location(x=232,y=160,z=2),carla.Rotation(roll=0,pitch=0,yaw=180))
            self.vehicle = self.world.try_spawn_actor(blueprint, spawn_point_3)
        
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
# -- Global Route Planner ------------------------------------------------------
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

class GlobalRoutePlannerDAO(object):
    """
    This class is the data access layer for fetching data
    from the carla server instance for GlobalRoutePlanner
    """

    def __init__(self, wmap):
        """
        Constructor

        wmap    :   carl world map object
        """
        self._wmap = wmap

    def get_topology(self):
        """
        Accessor for topology.
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects.

        return: list of dictionary objects with the following attributes
                entry   -   (x,y) of entry point of road segment
                exit    -   (x,y) of exit point of road segment
                path    -   list of waypoints separated by 1m from entry
                            to exit
                intersection    -   Boolean indicating if the road segment
                                    is an intersection
        """
        topology = []
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():
            x1 = segment[0].transform.location.x
            y1 = segment[0].transform.location.y
            x2 = segment[1].transform.location.x
            y2 = segment[1].transform.location.y
            seg_dict = dict()
            seg_dict['entry'] = (x1, y1)
            seg_dict['exit'] = (x2, y2)
            seg_dict['path'] = []
            wp1 = segment[0]
            wp2 = segment[1]
            seg_dict['intersection'] = True if wp1.is_intersection else False
            endloc = wp2.transform.location
            w = wp1.next(1)[0]
            while w.transform.location.distance(endloc) > 1:
                x = w.transform.location.x
                y = w.transform.location.y
                seg_dict['path'].append((x, y))
                w = w.next(1)[0]

            topology.append(seg_dict)
        return topology

class GlobalRoutePlanner(object):
    """
    This class provides a very high level route plan.
    Instantiate the calss by passing a reference to
    A GlobalRoutePlannerDAO object.
    """

    def __init__(self, dao):
        """
        Constructor
        """
        self._dao = dao
        self._topology = None
        self._graph = None
        self._id_map = None

    def setup(self):
        """
        Perform initial server data lookup for detailed topology
        and builds graph representation of the world map.
        """
        self._topology = self._dao.get_topology()
        # Creating graph of the world map and also a maping from
        # node co-ordinates to node id
        self._graph, self._id_map = self.build_graph()

    def plan_route(self, origin, destination):
        """
        The following function generates the route plan based on
        origin      : tuple containing x, y of the route's start position
        destination : tuple containing x, y of the route's end position
        return      : list of turn by turn navigation decisions as
        agents.navigation.local_planner.RoadOption elements
        Possible values (for now) are STRAIGHT, LEFT, RIGHT, LANEFOLLOW, VOID
        """

        threshold = math.radians(4.0)
        route = self.path_search(origin, destination)
        plan = []

        # Compare current edge and next edge to decide on action
        for i in range(len(route) - 2):
            current_edge = self._graph.edges[route[i], route[i + 1]]
            next_edge = self._graph.edges[route[i + 1], route[i + 2]]
            cv = current_edge['exit_vector']
            nv = next_edge['net_vector']
            cv, nv = cv + (0,), nv + (0,)  # Making vectors 3D
            num_edges = 0
            cross_list = []
            # Accumulating cross products of all other paths
            for neighbor in self._graph.neighbors(route[i+1]):
                num_edges+=1
                if neighbor != route[i + 2]:
                    select_edge = self._graph.edges[route[i+1], neighbor]
                    sv = select_edge['net_vector']
                    cross_list.append(np.cross(cv, sv)[2])
            # Calculating turn decision
            if next_edge['intersection'] and num_edges > 1:
                next_cross = np.cross(cv, nv)[2]
                deviation = math.acos(np.dot(cv, nv) /\
                    (np.linalg.norm(cv)*np.linalg.norm(nv)))
                if deviation < threshold:
                    action = RoadOption.STRAIGHT
                elif next_cross < min(cross_list):
                    action = RoadOption.LEFT
                elif next_cross > max(cross_list):
                    action = RoadOption.RIGHT
                plan.append(action)
        return plan

    def _distance_heuristic(self, n1, n2):
        """
        Distance heuristic calculator for path searching
        in self._graph
        """
        (x1, y1) = self._graph.nodes[n1]['vertex']
        (x2, y2) = self._graph.nodes[n2]['vertex']
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def path_search(self, origin, destination):
        """
        This function finds the shortest path connecting origin and destination
        using A* search with distance heuristic.
        origin      :   tuple containing x, y co-ordinates of start position
        desitnation :   tuple containing x, y co-ordinates of end position
        return      :   path as list of node ids (as int) of the graph self._graph
        connecting origin and destination
        """
        xo, yo = origin
        xd, yd = destination
        start = self.localise(xo, yo)
        end = self.localise(xd, yd)

        route = nx.astar_path(
            self._graph, source=self._id_map[start['entry']],
            target=self._id_map[end['exit']],
            heuristic=self._distance_heuristic,
            weight='length')

        return route

    def localise(self, x, y):
        """
        This function finds the road segment closest to (x, y)
        x, y        :   co-ordinates of the point to be localized
        return      :   pair of points, tuple of tuples containing co-ordinates
        of points that represents the road segment closest to x, y
        """
        distance = float('inf')
        nearest = (distance, dict())

        # Measuring distances from segment waypoints and (x, y)
        for segment in self._topology:
            entryxy = segment['entry']
            exitxy = segment['exit']
            path = segment['path']
            for xp, yp in [entryxy] + path + [exitxy]:
                new_distance = self.distance((xp, yp), (x, y))
                if new_distance < nearest[0]:
                    nearest = (new_distance, segment)

        segment = nearest[1]
        return segment

    def build_graph(self):
        """
        This function builds a networkx  graph representation of topology.
        The topology is read from self._topology.
        graph node properties:
            vertex   -   (x,y) of node's position in world map
            num_edges   -   Number of exit edges from the node
        graph edge properties:
            entry_vector    -   unit vector along tangent at entry point
            exit_vector     -   unit vector along tangent at exit point
            net_vector      -   unit vector of the chord from entry to exit
            intersection    -   boolean indicating if the edge belongs to an
                                intersection
        return      :   graph -> networkx graph representing the world map,
                        id_map-> mapping from (x,y) to node id
        """
        graph = nx.DiGraph()
        # Map with structure {(x,y): id, ... }
        id_map = dict()

        for segment in self._topology:

            entryxy = segment['entry']
            exitxy = segment['exit']
            path = segment['path']
            intersection = segment['intersection']
            for vertex in entryxy, exitxy:
                # Adding unique nodes and populating id_map
                if vertex not in id_map:
                    new_id = len(id_map)
                    id_map[vertex] = new_id
                    graph.add_node(new_id, vertex=vertex)

            n1, n2 = id_map[entryxy], id_map[exitxy]
            # Adding edge with attributes
            graph.add_edge(
                n1, n2,
                length=len(path) + 1, path=path,
                entry_vector=self.unit_vector(
                    entryxy, path[0] if len(path) > 0 else exitxy),
                exit_vector=self.unit_vector(
                    path[-1] if len(path) > 0 else entryxy, exitxy),
                net_vector=self.unit_vector(entryxy, exitxy),
                intersection=intersection)
        
        return graph, id_map

    def distance(self, point1, point2):
        """
        returns the distance between point1 and point2
        point1      :   (x,y) of first point
        point2      :   (x,y) of second point
        return      :   distance from point1 to point2
        """
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def unit_vector(self, point1, point2):
        """
        This function returns the unit vector from point1 to point2
        point1      :   (x,y) of first point
        point2      :   (x,y) of second point
        return      :   tuple containing x and y components of unit vector
                        from point1 to point2
        """
        x1, y1 = point1
        x2, y2 = point2

        vector = (x2 - x1, y2 - y1)
        vector_mag = math.sqrt(vector[0]**2 + vector[1]**2)
        vector = (vector[0] / vector_mag, vector[1] / vector_mag)

        return vector

    def dot(self, vector1, vector2):
        """
        This function returns the dot product of vector1 with vector2
        vector1      :   x, y components of first vector
        vector2      :   x, y components of second vector
        return      :   dot porduct scalar between vector1 and vector2
        """
        return vector1[0] * vector2[0] + vector1[1] * vector2[1]

    pass



# ==============================================================================
# -- ACC_Controller -----------------------------------------------------------
# ==============================================================================




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
        self._SpeedDistance_vehicle_controller = None
        self._Speed_vehicle_controller = None
        self._current_plan = None
        self._global_plan = None
        self._trigger_counter = 0
        self._traffic_detector = TrafficDetector(self._vehicle,self._target_waypoint,12.5,160.0)
        # queue with tuples of (waypoint, RoadOption)
        self._hop_resolution = 2.0
        self._waypoints_queue = deque(maxlen=600)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        # this is for the traffic detector
        self._lightslist = self._world.get_actors().filter("*traffic_light*")
        self._vehiclelist = self._world.get_actors().filter("*vehicle*")
        # SETTING controller
        self.preset_speed = 60.0
        self._target_distance = 10.0
        self.initialize_PID_controller(opt_dict)


        # Initialize the overtake parameters
        self._overtake_intention = False
        self._overtake_leftchange = False
        self._overtake_rightchange = False
        self._overtake_target = None
    def set_destination (self, location):
        start_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        end_waypoint = self._map.get_waypoint(
            carla.Location(location[0], location[1], location[2]))
        solution = []

        # Setting up global router
        dao = GlobalRoutePlannerDAO(self._vehicle.get_world().get_map())
        grp = GlobalRoutePlanner(dao)
        grp.setup()     

        # Obtain route plan
        x1 = start_waypoint.transform.location.x
        y1 = start_waypoint.transform.location.y
        x2 = end_waypoint.transform.location.x
        y2 = end_waypoint.transform.location.y
        route = grp.plan_route((x1, y1), (x2, y2))

        current_waypoint = start_waypoint
        route.append(RoadOption.VOID)
        for action in route:

            #   Generate waypoints to next junction
            wp_choice = current_waypoint.next(self._hop_resolution)
            while len(wp_choice) == 1:
                current_waypoint = wp_choice[0]
                solution.append((current_waypoint, RoadOption.LANEFOLLOW))
                wp_choice = current_waypoint.next(self._hop_resolution)
                #   Stop at destination
                if current_waypoint.transform.location.distance(
                    end_waypoint.transform.location) < self._hop_resolution: break
            if action == RoadOption.VOID: break

            #   Select appropriate path at the junction
            if len(wp_choice) > 1:

                # Current heading vector
                current_transform = current_waypoint.transform
                current_location = current_transform.location
                projected_location = current_location + \
                    carla.Location(
                        x=math.cos(math.radians(current_transform.rotation.yaw)),
                        y=math.sin(math.radians(current_transform.rotation.yaw)))
                v_current = vector(current_location, projected_location)

                direction = 0
                if action == RoadOption.LEFT:
                    direction = 1
                elif action == RoadOption.RIGHT:
                    direction = -1
                elif action == RoadOption.STRAIGHT:
                    direction = 0
                select_criteria = float('inf')

                #   Choose correct path 
                #   Choose the wp_select waypoint whose direction is the most similar to the action direction.
                for wp_select in wp_choice:
                    v_select = vector(
                        current_location, wp_select.transform.location)
                    cross = float('inf')
                    if direction == 0:
                        cross = abs(np.cross(v_current, v_select)[-1])
                    else:
                        cross = direction*np.cross(v_current, v_select)[-1]
                    if cross < select_criteria:
                        select_criteria = cross
                        current_waypoint = wp_select

                #   Generate all waypoints within the junction
                #   along selected path
                solution.append((current_waypoint, action))
                current_waypoint = current_waypoint.next(self._hop_resolution)[0]
                while current_waypoint.is_intersection:
                    solution.append((current_waypoint, action))
                    current_waypoint = current_waypoint.next(self._hop_resolution)[0]

        assert solution

        self._current_plan = solution
        self.set_global_plan(self._current_plan)

    def set_global_plan(self, current_plan):
        self._waypoints_queue.clear()
        for elem in current_plan:
            self._waypoints_queue.append(elem)
        self._target_road_option = RoadOption.LANEFOLLOW
        self._global_plan = True

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
        # emergency_2 = False
        hazard_vehicle_state, hazard_vehicle = self._traffic_detector.define_hazard_vehicle()               
        # light_state, traffic_light = self._traffic_detector.is_light_red()
        if hazard_vehicle_state:
            if get_speed(hazard_vehicle) <= 0.30 * get_speed(self._vehicle) or get_speed(hazard_vehicle) < 5.0:
                emergency_1 = True
        # if light_state:
            # emergency_2 = True
        if emergency_1 is True:
            return True
        return False

    def set_target_speed(self):
        vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()                

        if vehicle_state:
            if (get_speed(front_vehicle) < self.preset_speed * 0.75) and  (self.emergency_brake_situation() is False):
                self._overtake_target = front_vehicle
                self._overtake_intention = True
                return self.preset_speed * 0.65
            return get_speed(front_vehicle)
        return self.preset_speed


    def set_target_distance(self):
        front_vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()
        t0=1.5
        Cv=0.05
        Ca=0.3
        th_max=1.6
        th_min=0.2  
        delta_t = 0.0     
        if  front_vehicle_state:

            # ego_x = self._vehicle.get_location().x
            # ego_y = self._vehicle.get_location().y
            
            # front_x = front_vehicle.get_location().x
            # front_y = front_vehicle.get_location().y
            
            # dx = ego_x - front_x
            # dy = ego_y - front_y
            
            # init_distance = math.sqrt(dx * dx + dy * dy)
            RelaSpeed = (get_speed(front_vehicle) - get_speed(self._vehicle)) / 3.6
            # print("RelaSpeed",RelaSpeed)
            ap = front_vehicle.get_acceleration()
            front_acceleration = math.sqrt(ap.x**2+ap.y**2+ap.z**2)
            th = t0 - Cv * RelaSpeed - Ca * front_acceleration
            if th >= th_max:
                delta_t = th_max
            elif th > th_min and th < th_max:
                delta_t = th
            else:
                delta_t = th_min
        
        return  delta_t * (get_speed(self._vehicle) / 3.6) + 15.0
        # print("self._target_distance",self._target_distance)
        # return self._target_distance
    def left_lane_change(self):
        vehicle_state, rear_vehicle_next_lane = self._traffic_detector.search_rear_vehicle_left_lane()
        # print (rear_vehicle_next_lane)

        if vehicle_state is False:
            self._waypoints_queue.clear()
            new_waypoint = self._map.get_waypoint(carla.Location(self._vehicle.get_location().x + 40.0, 
                                                                    self._vehicle.get_location().y - 3.5,
                                                                    self._vehicle.get_location().z)) 
            
            self._waypoints_queue.append((new_waypoint, RoadOption.LANEFOLLOW))
            # control.throttle = 0.55
            # self._vehicle.apply_control(control)

            # self._compute_next_waypoints(200)

        pass
    
    def right_lane_change(self):
        vehicle_state, rear_vehicle_next_lane = self._traffic_detector.search_rear_vehicle_right_lane()
        # print (rear_vehicle_next_lane)
        
        if vehicle_state is False:
            self._waypoints_queue.clear()
            new_waypoint = self._map.get_waypoint(carla.Location(self._vehicle.get_location().x + 60.0, 
                                                                    self._vehicle.get_location().y + 3.5,
                                                                    self._vehicle.get_location().z)) 
            
            self._waypoints_queue.append((new_waypoint, RoadOption.LANEFOLLOW))
            # control.throttle = 0.55
            # self._vehicle.apply_control(control)

            # self._compute_next_waypoints(200)

        pass

    def decide_on_overtake(self):    
        rear_right_vehicle_state, rear_vehicle_right_lane = self._traffic_detector.search_rear_vehicle_right_lane()
        rear_left_vehicle_state, rear_vehicle_left_lane = self._traffic_detector.search_rear_vehicle_left_lane()
        front_right_vehicle_state, front_vehicle_right_lane = self._traffic_detector.search_front_vehicle_right_lane()
        front_left_vehicle_state, front_vehicle_left_lane = self._traffic_detector.search_front_vehicle_left_lane()

        if front_left_vehicle_state:
            # print ("Left Front Vehicle is " + front_vehicle_left_lane.type_id)
            if self._overtake_intention is True:             
                if self._overtake_target is not None:
                    threshold_distance = self._vehicle.get_location().x - self._overtake_target.get_location().x
                
                while self._overtake_target is not None and self._overtake_leftchange is False and  rear_left_vehicle_state is False \
                                                        and front_left_vehicle_state is False:
                    
                    self.left_lane_change()
                    self._overtake_leftchange = True
                    print ("I reach frist move")

                while self._overtake_target is not None and threshold_distance >= 20.0 and rear_right_vehicle_state is False and front_right_vehicle_state is False \
                                                and self._overtake_leftchange is True and self._overtake_rightchange is False:
                    print ("I reach second move")
                    self.right_lane_change() 
                    self._overtake_rightchange = True
                
                while self._overtake_target is not None and self._overtake_leftchange is True and self._overtake_rightchange is True:
                    self._overtake_target = None
                    self._overtake_leftchange = False
                    self._overtake_rightchange = False
                    self._overtake_intention = False
        pass
    
    def emergency_stop(self):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False

        return control                  
    
    def initialize_PID_controller(self, opt_dict):
        self._dt = 1.0 / 20.0
        self._target_speed = self.preset_speed  # Km/h
        self._sampling_radius = 4.0
        # self._sampling_radius = self._target_speed * 0.5 / 2.8  # 0.5 seconds horizon
        self._min_distance = self._sampling_radius * self.MIN_DISTANCE_PERCENTAGE
        # vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()     
        args_lateral_dict = {
            'K_P': 1.1,
            'K_D': 0.0001,
            'K_I': 1.2,
            'dt': self._dt}
        
        SpeedDistance_args_longitudinal_dict = {
            'K_Pv': 0.4,
            'K_Dv': 0.01,
            'K_Iv':0.2,
            'K_Pd': -0.05,
            'K_Dd': 0.0,
            'K_Id': -0.01,            
            'dt': self._dt}  
        
        Speed_args_longitudinal_dict = {
            'K_P': 1.1,
            'K_D': 0.02,
            'K_I':1.0,
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
        if ' Speed_longitudinal_control_dict' in opt_dict:
             Speed_args_longitudinal_dict = opt_dict['longitudinal_control_dict']
        if ' SpeedDistance_longitudinal_control_dict' in opt_dict:
             SpeedDistance_args_longitudinal_dict = opt_dict['SpeedDistance_longitudinal_control_dict']
        self._current_waypoint = self._map.get_waypoint(
            self._vehicle.get_location())

        self._SpeedDistance_vehicle_controller = SpeedDistance_VehiclePIDController(self._vehicle,None,
                                                        args_lateral=args_lateral_dict,
                                                        args_longitudinal= SpeedDistance_args_longitudinal_dict)

        self._Speed_vehicle_controller = VehiclePIDController(self._vehicle,
                                                              args_lateral=args_lateral_dict,
                                                              args_longitudinal= Speed_args_longitudinal_dict)

        self._global_plan = False

        # compute initial waypoints
        self._waypoints_queue.append( (self._current_waypoint.next(self._sampling_radius)[0], RoadOption.LANEFOLLOW))
        self._target_road_option = RoadOption.LANEFOLLOW
       
        # fill waypoint trajectory queue
        self._compute_next_waypoints(k=100)        

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
        # self._traffic_detector.search_rear_vehicle_right_lane()
        self.refresh_traffic_detector()
        # if len(self._SpeedDistance_vehicle_controller._lon_controller._ed_buffer) > 0:
        #     print (self._SpeedDistance_vehicle_controller._lon_controller._ed_buffer[-1])
        
        # not enough waypoints in the horizon? => add more!
        if len(self._waypoints_queue) < int(self._waypoints_queue.maxlen * 0.5):
            if not self._global_plan:
                self._compute_next_waypoints(k=50)

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

        self._sampling_radius = self.set_sampling_radius()
        self._traffic_detector.get_speed_limit()
        
        # current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        
        # target waypoint
        vehicle_state, front_vehicle = self._traffic_detector.search_front_vehicle()                  
        #  refresh the target speed and target distance for PID controller.
        self._target_speed = self.set_target_speed()

        print ("target distance is " + str(self._target_distance))
        self._target_waypoint, self._target_road_option = self._waypoint_buffer[0]

        if self.emergency_brake_situation() is False:
            if vehicle_state:
                self._target_distance = self.set_target_distance()
                self._SpeedDistance_vehicle_controller._lon_controller._front_vehicle = front_vehicle
                # print ("I am using speed-distance PID.")

                control = self._SpeedDistance_vehicle_controller.run_step(self._target_speed,self._target_distance, self._target_waypoint)
            else:
                control = self._Speed_vehicle_controller.run_step(self._target_speed, self._target_waypoint)
                # print ("I am using speed PID.")
            self.decide_on_overtake()
        else:
            control = self.emergency_stop()

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
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
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
class Recorder(object):
    def __init__(self,vehicle,controller,workbook):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.controller = controller
        self.front_vehicle_state, self.front_vehicle = controller._traffic_detector.search_front_vehicle()

        self.workbook = workbook
        self.counter = 1
        self.sheetname = "Raw_data"
        self.sheet = workbook.add_sheet(self.sheetname)
        self.vehicle_list = self.world.get_actors().filter("vehicle*")
        self.workbookname =  str(time.strftime('%Y.%m.%d_%H%M%S',time.localtime(time.time()))) \
                                    + '_ID_' + str(vehicle.id) + '.xls'
        
        self.sheet.write(0,0,"Ego_Speed")
        self.sheet.write(0,1,"Ego_TargetSpeed")        
        self.sheet.write(0,2,"Ego_Acceleration")        
        self.sheet.write(0,3,"Ego_Throttle")
        self.sheet.write(0,4,"Ego_Steering")
        self.sheet.write(0,5,"Ego_Brake")
        self.sheet.write(0,6,"Ego_Location_x")
        self.sheet.write(0,7,"Ego_Location_y")
        self.sheet.write(0,8,"Ego_Rotation_Yaw")
        self.sheet.write(0,9,"Ego_Rotation_Pitch")
        self.sheet.write(0,10,"Ego_Rotation_Roll")
        
        self.sheet.write(0,11,"Front_Speed")
        self.sheet.write(0,12,"Front_Acceleration")
        self.sheet.write(0,13,"Front_Throttle")
        self.sheet.write(0,14,"Front_Steering")
        self.sheet.write(0,15,"Front_Brake")
        self.sheet.write(0,16,"Front_Location_x")
        self.sheet.write(0,17,"Front_Location_y")
        self.sheet.write(0,18,"Front_Rotation_Yaw")
        self.sheet.write(0,19,"Front_Rotation_Pitch")
        self.sheet.write(0,20,"Front_Rotation_Roll")

        self.sheet.write(0,21,"Relative_Distance")
        self.sheet.write(0,22,"Target_Relative_Distance")

    
    def start_recorder(self):
        wb = self.workbook
        vehicle = self.vehicle
        controller = self.controller
        front_vehicle_state, front_vehicle = controller._traffic_detector.search_front_vehicle()

        row = self.counter
        worksheet = self.sheet
        controller = self.controller
        
        # Export the data of Ego car.
        ego_speed = get_speed(vehicle)
        
        ego_target_speed = controller.set_target_speed()
        
        ego_acceleration_vector = vehicle.get_acceleration()
        ego_acceleration = math.sqrt(ego_acceleration_vector.x**2 + ego_acceleration_vector.y**2 + ego_acceleration_vector.z**2)
        
        ego_control = vehicle.get_vehicle_control()
        ego_throttle = ego_control.throttle
        ego_steering = ego_control.steer
        ego_brake = ego_control.brake        

        ego_location_x = vehicle.get_location().x
        ego_location_y = vehicle.get_location().y
        
        ego_rotation_yaw = vehicle.get_transform().rotation.yaw
        ego_rotation_pitch = vehicle.get_transform().rotation.pitch
        ego_rotation_roll = vehicle.get_transform().rotation.roll

        worksheet.write(row,0,ego_speed)
        worksheet.write(row,1,ego_target_speed)        
        worksheet.write(row,2,ego_acceleration)
        worksheet.write(row,3,ego_throttle )
        worksheet.write(row,4,ego_steering)
        worksheet.write(row,5,ego_brake)
        worksheet.write(row,6,ego_location_x)
        worksheet.write(row,7,ego_location_y)
        worksheet.write(row,8, ego_rotation_yaw)
        worksheet.write(row,9, ego_rotation_pitch)
        worksheet.write(row,10, ego_rotation_roll)

        # Export the data of Front car.

        if front_vehicle_state:
            front_speed = get_speed(front_vehicle)
                        
            front_acceleration_vector = front_vehicle.get_acceleration()
            front_acceleration = math.sqrt(front_acceleration_vector.x**2 + front_acceleration_vector.y**2 + front_acceleration_vector.z**2)
            
            front_control = front_vehicle.get_vehicle_control()
            front_throttle = front_control.throttle
            front_steering = front_control.steer
            front_brake = front_control.brake        

            front_location_x = front_vehicle.get_location().x
            front_location_y = front_vehicle.get_location().y
            
            front_rotation_yaw = front_vehicle.get_transform().rotation.yaw
            front_rotation_pitch = front_vehicle.get_transform().rotation.pitch
            front_rotation_roll = front_vehicle.get_transform().rotation.roll

            relative_distance = math.sqrt((front_location_x - ego_location_x)**2 + (front_location_y - ego_location_y)**2)
            target_relative_distance = controller.set_target_distance()
            
            worksheet.write(row,11,front_speed)
            worksheet.write(row,12,front_acceleration)        
            worksheet.write(row,13,front_throttle)
            worksheet.write(row,14,front_steering)
            worksheet.write(row,15,front_brake)
            worksheet.write(row,16,front_location_x)
            worksheet.write(row,17,front_location_y)
            worksheet.write(row,18,front_rotation_yaw)
            worksheet.write(row,19,front_rotation_pitch)
            worksheet.write(row,20,front_rotation_roll)
            worksheet.write(row,21,relative_distance)
            worksheet.write(row,22,target_relative_distance)


        self.counter += 1
        wb.save(self.workbookname)

    def finish_recorder(self):
        wb = self.workbook
        wb.save()

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
        transform_2 = carla.Transform (carla.Location(x=2400, y=-8.25, z=0.4),carla.Rotation(pitch=0, yaw=180, roll=0))
        transform_3 = carla.Transform (carla.Location(x=-2410, y=5.7, z=0.4),carla.Rotation(pitch=0, yaw=0, roll=0))
        transform_4 = carla.Transform(carla.Location(x=-2405 ,y=12.25, z=0.40), carla.Rotation(pitch=0, yaw=0, roll=0))

        # carModel_1 = random.choice (blueprint.filter('vehicle.tesla.*')) 
        carModel_1 = world.world.get_blueprint_library().find('vehicle.tesla.model3')
        carModel_1.set_attribute('color','10,10,10')
        carActor_1 = world.world.try_spawn_actor(carModel_1,transform_2)
        carActor_1.set_autopilot (True)        
        supporting_actor_list.append(carActor_1)        
        
        
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
        # ACC_controller.set_destination((-2300,12.2,1.23))

        
        # if args.agent == "Roaming":
        #     agent = RoamingAgent(world.vehicle)
        # else:
        #     agent = BasicAgent(world.vehicle)
        #     spawn_point = world.world.get_map().get_spawn_points()[0]
        #     agent.set_destination((spawn_point.location.x,
        #                            spawn_point.location.y,
        #                            spawn_point.location.z))

        clock = pygame.time.Clock()
        
        # this vehicle list only has 1 member
        vehicle_list_1 = world.world.get_actors().filter("*vehicle*")
        # this vehicle list has 2 members
        # vehicle_list_2 = PID_contoller._world.get_actors().filter("*vehicle*")
        
        wb = xlwt.Workbook()
        ego_recorder = Recorder(hero,ACC_controller,wb)
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
            world.vehicle.apply_control(control)
            ego_recorder.start_recorder()
            # print (counter)
            
            # sheet1.write(counter,0,'haha')
            # counter = counter + 1 
            # wb.save('xlwt example_4.xlsx')
            # print (world.vehicle.get_location().x)
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
