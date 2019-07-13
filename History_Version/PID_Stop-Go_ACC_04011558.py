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
from agents.navigation.roaming_agent import *
from agents.navigation.basic_agent import *
from agents.tools.misc import distance_vehicle, get_speed
from agents.tools.misc import is_within_distance_ahead, compute_magnitude_angle



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
        blueprint.set_attribute('color', '120,0,0')
        
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
            # spawn_points = self.world.get_map().get_spawn_points()
            # spawn_point = spawn_points[1]
            # self.vehicle = self.world.spawn_actor(blueprint, spawn_point)
    
            # support_actor = random.choice(self.world.get_actors().filter("*vehicle*"))
            # support_actor_point = support_actor.get_transform()
            # spawn_point = support_actor_point
            # spawn_point.location.y = spawn_point.location.y - 10.0
            # spawn_point = carla.Transform(carla.Location(x=373.40, y=-8.7, z=0.40), carla.Rotation(pitch=0, yaw=-181.00004, roll=0))
            # Tesla spawn parameter
            spawn_point = carla.Transform(carla.Location(x=-2.10 ,y=-150, z=0.80), carla.Rotation(pitch=0, yaw=90.0, roll=0))


            # spawn_point.rotation.roll = 90.0
            # spawn_point.rotation.pitch = 90.0
                        
            # spawn_point = carla.Transform (carla.Location(x=232,y=160,z=2),carla.Rotation(roll=0,pitch=0,yaw=180))
            self.vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
        
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.vehicle, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.vehicle, self.hud)
        self.camera_manager = CameraManager(self.vehicle, self.hud)
        self.camera_manager._transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.vehicle)
        self.hud.notification(actor_type)
    
    # def record(self,vehicle):
    #     global counter
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
# -- PIDControl -----------------------------------------------------------
# ==============================================================================

class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, vehicle,
                 args_lateral={'K_P': 1.12, 'K_D': 0.005, 'K_I': 1.17},
                 args_longitudinal={'K_P': 1.0, 'K_D':0 , 'K_I': 1}):
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
        self._world = self._vehicle.get_world()
        self._lon_controller = PIDLongitudinalController(
            self._vehicle, **args_longitudinal)
        self._lat_controller = PIDLateralController(
            self._vehicle, **args_lateral)

    def run_step(self):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: distance (in meters) to the waypoint
        """
        throttle = self._lon_controller.run_step()
        steering = self._lat_controller.run_step()

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
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
        self._world = self._vehicle.get_world()
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=30)


    def speed_setting(self):
        global target_vehicle
        target_vehicle = None
        ego_vehicle = self._vehicle
        ego_vehicle_location = self._vehicle.get_location()
        # ego_vehicle_waypoint = self._world.get_map().get_waypoint(ego_vehicle_location)
        vehicle_list = self._world.get_actors().filter("*vehicle*")

        car_number = len(vehicle_list)
        
        if car_number > 1:
            # if target_vehicle is None:    
            #     for index in vehicle_list:
            #         if index.id != ego_vehicle.id:
            #             target_vehicle = index
            #             break
            #         else:
            #             pass
            # else:                 
            #     target_vehicle_location = target_vehicle.get_location()
            #     # target_vehicle_waypoint = self._world.get_map().get_waypoint(target_vehicle_location)
            #     # if target_vehicle_waypoint.road_id == ego_vehicle_waypoint.road_id or\
            #     #                 target_vehicle_waypoint.lane_id == ego_vehicle_waypoint.lane_id:
            #     offset = abs(ego_vehicle_location.y - target_vehicle_location.y)
            #     if offset < 1.7:
            #         return get_speed(target_vehicle)  
            #     else:
            #         return 60
            # target_vehicle = vehicle_list[0]

            for index in vehicle_list:
                    if index.id != ego_vehicle.id:
                        target_vehicle = index
                        break
                    else:
                        pass
            # target_vehicle_waypoint = self._world.get_map().get_waypoint(target_vehicle_location)
            # if target_vehicle_waypoint.road_id == ego_vehicle_waypoint.road_id or\
            #                 target_vehicle_waypoint.lane_id == ego_vehicle_waypoint.lane_id:
            target_vehicle_location = target_vehicle.get_location()
            offset = abs(ego_vehicle_location.x - target_vehicle_location.x)
            if offset < 1.77:
                return get_speed(target_vehicle)
            else:
                return 20            
        
        else:
            return 20
        


    def run_step(self, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in Km/h
        :return: throttle control in the range [0, 1]
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control( current_speed)


    def _pid_control(self, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        target_speed = self.speed_setting()
        _e = (target_speed - current_speed)
        self._e_buffer.append(_e)

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _e) + (self._K_D * _de / self._dt) + (self._K_I * _ie * self._dt), 0.0, 1.0)


class PIDLateralController():
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
        self._world = self._vehicle.get_world()
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=10)

    def waypoint_setting(self):
        target_vehicle = None
        ego_vehicle = self._vehicle
        ego_vehicle_location = self._vehicle.get_location()
        # ego_vehicle_waypoint = self._world.get_map().get_waypoint(ego_vehicle_location)
        vehicle_list = self._world.get_actors().filter("*vehicle*")

        car_number = len(vehicle_list)
        

            # if target_vehicle is None:    
            #     for index in vehicle_list:
            #         if index.id != ego_vehicle.id:
            #             target_vehicle = index
            #             break
            #         else:
            #             pass
            # else:                 
            #     target_vehicle_location = target_vehicle.get_location()
            #     # target_vehicle_waypoint = self._world.get_map().get_waypoint(target_vehicle_location)
            #     # if target_vehicle_waypoint.road_id == ego_vehicle_waypoint.road_id or\
            #     #                 target_vehicle_waypoint.lane_id == ego_vehicle_waypoint.lane_id:
            #     offset = abs(ego_vehicle_location.y - target_vehicle_location.y)
            #     if offset < 1.7:
            #         return get_speed(target_vehicle)  
            #     else:
            #         return 60
            # target_vehicle = vehicle_list[0]

        for index in vehicle_list:
                if index.id != ego_vehicle.id:
                    target_vehicle = index
                    break
                else:
                    pass
        # target_vehicle_waypoint = self._world.get_map().get_waypoint(target_vehicle_location)
        # if target_vehicle_waypoint.road_id == ego_vehicle_waypoint.road_id or\
        #                 target_vehicle_waypoint.lane_id == ego_vehicle_waypoint.lane_id:
        target_vehicle_location = target_vehicle.get_location()
        waypoint = self._world.get_map().get_waypoint(target_vehicle_location)
        
        return waypoint            
        


    def run_step(self):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control( self._vehicle.get_transform())

    def _pid_control(self, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """
        # print (waypoint)
        waypoint = self.waypoint_setting()
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


# ==============================================================================
# -- Recorder() ---------------------------------------------------------
# ==============================================================================
class Recorder(object):
    def __init__(self,vehicle,support_vehicle,controller,workbook):
        self.vehicle = vehicle
        self.support_vehicle = support_vehicle
        self.world = vehicle.get_world()
        self.workbook = workbook
        self.counter = 1
        self.sheetname = str(get_actor_display_name(vehicle, truncate=20))
        self.sheet = workbook.add_sheet(self.sheetname)
        self.vehicle_list = self.world.get_actors().filter("vehicle*")
        self.controller = controller
        self.workbookname =  str(time.strftime('%Y.%m.%d_%H%M%S',time.localtime(time.time()))) \
                                    + '_ID_' + str(vehicle.id) + '.xls'
        self.sheet.write(0,0,"Ego_Speed")
        self.sheet.write(0,1,"Supporting_Speed")        
        self.sheet.write(0,2,"Target_Speed")
        self.sheet.write(0,3,"Ego_Acceleration")
        self.sheet.write(0,4,"Supporting_Acceleration")
        self.sheet.write(0,5,"Ego_Location_y")
        self.sheet.write(0,6,"Support_Location_y")
        self.sheet.write(0,7,"Support_Yaw_Angle")
        self.sheet.write(0,8,"Ego_Yaw_Angle")
        self.sheet.write(0,9,"Throttle")
        self.sheet.write(0,10,"Steer")
        self.sheet.write(0,11,"Brake")
        self.sheet.write(0,12,"Relative_distance")

    
    def start_recorder(self):
        wb = self.workbook
        vehicle = self.vehicle
        row = self.counter
        worksheet = self.sheet
        supporting_actor = self.support_vehicle
        controller = self.controller

        control = vehicle.get_vehicle_control()
        ego_velocity = vehicle.get_velocity()   
        support_velocity = supporting_actor.get_velocity()
        target_speed = controller
        ego_acceleration_vector = vehicle.get_acceleration()
        sup_acceleration_vector = supporting_actor.get_acceleration()

        s_loc = supporting_actor.get_location()
        s_loc_x = s_loc.x
        s_loc_y = s_loc.y
        s_loc_z = s_loc.z
        
        support_speed = 3.6 * math.sqrt(support_velocity.x**2 + support_velocity.y**2 + support_velocity.z**2)
        ego_speed = 3.6 * math.sqrt(ego_velocity.x**2 + ego_velocity.y**2 + ego_velocity.z**2)
        target_speed = controller._lon_controller.speed_setting()

        ego_acceleration = math.sqrt(ego_acceleration_vector.x**2 + ego_acceleration_vector.y**2 + ego_acceleration_vector.z**2)
        sup_acceleration = math.sqrt(sup_acceleration_vector.x**2 + sup_acceleration_vector.y**2 + sup_acceleration_vector.z**2)
        # sup_acceleration = 10
        ego_location_x = vehicle.get_location().x
        ego_location_y = vehicle.get_location().y
        ego_location_z = vehicle.get_location().z
        
        ego_yaw = vehicle.get_transform().rotation.yaw
        support_yaw = supporting_actor.get_transform().rotation.yaw

        throttle = control.throttle
        steer = control.steer
        brake = control.brake

        distance = math.sqrt((s_loc_x - ego_location_x)**2 + (s_loc_y - ego_location_y)**2 + (s_loc_z - ego_location_z)**2)

        worksheet.write(row,0,ego_speed)
        worksheet.write(row,1,support_speed)        
        worksheet.write(row,2,target_speed)
        worksheet.write(row,3,ego_acceleration)
        worksheet.write(row,4,sup_acceleration)
        worksheet.write(row,5,ego_location_y)
        worksheet.write(row,6,s_loc_y)
        worksheet.write(row,7,support_yaw)
        worksheet.write(row,8,ego_yaw)
        worksheet.write(row,9,throttle)
        worksheet.write(row,10,steer)
        worksheet.write(row,11,brake)
        worksheet.write(row,12,distance)

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
        blueprint = world.world.get_blueprint_library()
        transform_1 = carla.Transform (carla.Location(x=-2.10, y=-145.30, z=0.4),carla.Rotation(pitch=0, yaw=90.0, roll=0))
        carModel_1 = random.choice (blueprint.filter('vehicle.tesla.*')) 
        carModel_1.set_attribute('color','10,10,10')
        carActor_1 = world.world.try_spawn_actor(carModel_1,transform_1)
        carActor_1.set_autopilot (True)        
        supporting_actor_list.append(carActor_1)        
        front_vehicle = carActor_1

        controller = KeyboardControl(world, False)
        PID_contoller = VehiclePIDController (world.vehicle)
        hero = world.vehicle
        
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
        vehicle_list_2 = PID_contoller._world.get_actors().filter("*vehicle*")
        
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
        way_point = world.world.get_map().get_waypoint(carla.Location(x=-2.1, y=180, z=0))
        # spawn_point = world.world.get_map().get_spawn_points()[0]
        # way_point = world.world.get_map().get_waypoint(spawn_point.location)
        wb = xlwt.Workbook()
        ego_recorder = Recorder(hero,front_vehicle,PID_contoller,wb)
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
            control = PID_contoller.run_step()
            world.vehicle.apply_control(control)
            ego_recorder.start_recorder()
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
