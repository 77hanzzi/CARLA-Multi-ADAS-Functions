
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