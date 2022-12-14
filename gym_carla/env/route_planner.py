import carla
import logging,random
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from enum import Enum
from collections import deque
from shapely.geometry import Polygon
from gym_carla.env.settings import ROADS,STRAIGHT,CURVE,JUNCTION
from gym_carla.env.util.misc import vector,compute_magnitude_angle,\
    is_within_distance_ahead,draw_waypoints,compute_distance,is_within_distance

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.

    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6

class GlobalPlanner:
    """
    class for generating chosen circuit's road topology,topology is saved with waypoints list
    vehicle always runs on the outer ring of chosen route
    """
    def __init__(self,map,sampling_resolution=1000.0) -> None:
        self._sampling_resolution=sampling_resolution
        self._wmap=map

        #code for simulation road generation
        self._route=[]
        self._topology = []

        #generate circuit topology
        self._build_topology()
        #print(len(self._topology))
        # self._build_graph()
        # nx.draw(self._graph,with_labels=True,font_weight='bold')
        # plt.draw()
        # plt.show()

        #generate route waypoints list
        self._build_route()


    def get_route(self,ego_waypoint):
        return self._compute_next_waypoints(ego_waypoint,len(self._route))

    def get_spawn_points(self):
        """Vehicle can only be spawned on straight road"""
        spawn_points=[]
        for wp in self._route:
            if wp.road_id in STRAIGHT:
                spawn_points.append(wp)
        
        return spawn_points

    def _build_route(self):
        begin=self._topology[0]
        self._route.append(begin['entry'])
        for wp in begin['path']:
            self._route.append(wp)
        #self._route.append(begin['exit'])
        indicator=begin['exit']
        iter=None
        for seg in self._topology:
            if seg['entry'].id==indicator.id:
                iter=seg
                break

        while(indicator.id!=begin['entry'].id):
            self._route.append(iter['entry'])
            for wp in iter['path']:
                self._route.append(wp)
            #self._route.append(iter['exit'])
            indicator=iter['exit']
            for seg in self._topology:
                if seg['entry'].id==indicator.id:
                    iter=seg
                    break
        
        #remove start
        #print(len(self._route))

    def _compute_next_waypoints(self,cur_wp,k=1):
        """
        Add new waypoints to the trajectory queue.

        :param cur_wp: current waypoint
        :param k: how many waypoints to compute
        :return: waypoint list
        """
        next_wps=[]
        iter=None
        for i,wp in enumerate(self._route):
            if wp.id==cur_wp.id:
                iter=i
        if iter is None:
            logging.error("Current waypoint on route not found!")
        if iter+k<len(self._route):
            for i in range(k):
                next_wps.append(self._route[iter+i+1])
        else:
            for i in range(len(self._route)-iter-1):
                next_wps.append(self._route[iter+i+1])
            for i in range(k-(len(self._route)-iter-1)):
                next_wps.append(self._route[i])
        
        return next_wps

    def _build_topology(self):
        """
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects with the following attributes

        - entry (carla.Waypoint): waypoint of entry point of road segment
        - entryxyz (tuple): (x,y,z) of entry point of road segment
        - exit (carla.Waypoint): waypoint of exit point of road segment
        - exitxyz (tuple): (x,y,z) of exit point of road segment
        - path (list of carla.Waypoint):  list of waypoints between entry to exit, separated by the resolution
        """
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            if self._test_wp(wp1) and self._test_wp(wp2):
                l1, l2 = wp1.transform.location, wp2.transform.location
                # Rounding off to avoid floating point imprecision
                x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
                wp1.transform.location, wp2.transform.location = l1, l2
                seg_dict = dict()
                seg_dict['entry'], seg_dict['exit'] = wp1, wp2
                seg_dict['entryxyz'], seg_dict['exitxyz'] = (x1, y1, z1), (x2, y2, z2)
                seg_dict['path'] = []
                endloc = wp2.transform.location
                if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                    w = wp1.next(self._sampling_resolution)[0]
                    while w.transform.location.distance(endloc) > self._sampling_resolution:
                        seg_dict['path'].append(w)
                        w = w.next(self._sampling_resolution)[0]
                    if self._test_wp(w):
                        seg_dict['path'].append(w)
                else:
                    next_wp=wp1.next(self._sampling_resolution)[0]
                    if self._test_wp(next_wp):
                        seg_dict['path'].append(next_wp)
                self._topology.append(seg_dict)      

    def _test_wp(self,wp):
        """Test if waypoint is on chosen route"""
        if (wp.road_id in STRAIGHT or wp.road_id in JUNCTION) and wp.lane_id==-1:
            return True
        if wp.road_id in CURVE and wp.lane_id==1:
            return True
        return False

    def _build_graph(self):
        """
        This function builds a networkx graph representation of topology, creating several class attributes:
        - graph (networkx.DiGraph): networkx graph representing the world map, with:
            Node properties:
                vertex: (x,y,z) position in world map
            Edge properties:
                entry_vector: unit vector along tangent at entry point
                exit_vector: unit vector along tangent at exit point
                net_vector: unit vector of the chord from entry to exit
                intersection: boolean indicating if the edge belongs to an  intersection
        - id_map (dictionary): mapping from (x,y,z) to node id
        - road_id_to_edge (dictionary): map from road id to edge in the graph
        """

        self._graph = nx.DiGraph()
        self._id_map = dict()  # Map with structure {(x,y,z): id, ... }
        self._road_id_to_edge = dict()  # Map with structure {road_id: {lane_id: edge, ... }, ... }

        for segment in self._topology:
            entry_xyz, exit_xyz = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            intersection = entry_wp.is_junction
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id

            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map
                if vertex not in self._id_map:
                    new_id = len(self._id_map)
                    self._id_map[vertex] = new_id
                    self._graph.add_node(new_id, vertex=vertex)
            n1 = self._id_map[entry_xyz]
            n2 = self._id_map[exit_xyz]
            if road_id not in self._road_id_to_edge:
                self._road_id_to_edge[road_id] = dict()
            if section_id not in self._road_id_to_edge[road_id]:
                self._road_id_to_edge[road_id][section_id] = dict()
            self._road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)

            entry_carla_vector = entry_wp.transform.rotation.get_forward_vector()
            exit_carla_vector = exit_wp.transform.rotation.get_forward_vector()

            # Adding edge with attributes
            self._graph.add_edge(
                n1, n2,
                length=len(path) + 1, path=path,
                entry_waypoint=entry_wp, exit_waypoint=exit_wp,
                entry_vector=np.array(
                    [entry_carla_vector.x, entry_carla_vector.y, entry_carla_vector.z]),
                exit_vector=np.array(
                    [exit_carla_vector.x, exit_carla_vector.y, exit_carla_vector.z]),
                net_vector=vector(entry_wp.transform.location, exit_wp.transform.location),
                intersection=intersection, type=RoadOption.LANEFOLLOW)

class LocalPlanner:
    def __init__(self, vehicle,buffer_size=12):
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()

        self._sampling_radius = 4
        self._min_distance = 1

        self._target_waypoint = None
        self._buffer_size = buffer_size
        self._waypoint_buffer = deque(maxlen=self._buffer_size)

        self._waypoints_queue = deque(maxlen=600)
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._target_road_option = RoadOption.LANEFOLLOW
        self._stop_waypoint_creation=False

        self._last_traffic_light = None
        self._proximity_threshold = self._sampling_radius*buffer_size

        self._waypoints_queue.append( (self._current_waypoint, RoadOption.LANEFOLLOW))
        #self._waypoints_queue.append( (self._current_waypoint.next(self._sampling_radius)[0], RoadOption.LANEFOLLOW))
        #self._compute_next_waypoints(k=200)

    def run_step(self):
        waypoints = self._get_waypoints()
        red_light, vehicle_front = self._get_hazard()
        # red_light = False
        return waypoints, red_light, vehicle_front
    
    def get_incoming_waypoint_and_direction(self,steps=3):
        """
        Returns direction and waypoint at a distance ahead defined by the user.

            :param steps: number of steps to get the incoming waypoint.
        """
        if len(self._waypoint_buffer)>steps:
            return self._waypoint_buffer[steps]
        else:
            try:
                wpt,direction=self._waypoint_buffer[-1]
                return wpt,direction
            except IndexError as i:
                return None,RoadOption.VOID

    def set_sampling_redius(self,sampling_resolution):
        self._sampling_radius=sampling_resolution
    
    def set_min_distance(self,min_distance):
        self._min_distance=min_distance

    def set_global_plan(self, current_plan, stop_waypoint_creation=True, clean_queue=True):
        """
        Adds a new plan to the local planner. A plan must be a list of [carla.Waypoint, RoadOption] pairs
        The 'clean_queue` parameter erases the previous plan if True, otherwise, it adds it to the old one
        The 'stop_waypoint_creation' flag stops the automatic creation of random waypoints

        :param current_plan: list of (carla.Waypoint, RoadOption)
        :param stop_waypoint_creation: bool
        :param clean_queue: bool
        :return:
        """
        if clean_queue:
            self._waypoints_queue.clear()

        # Remake the waypoints queue if the new plan has a higher length than the queue
        new_plan_length = len(current_plan) + len(self._waypoints_queue)
        if new_plan_length > self._waypoints_queue.maxlen:
            new_waypoint_queue = deque(maxlen=new_plan_length)
            for wp in self._waypoints_queue:
                new_waypoint_queue.append(wp)
            self._waypoints_queue = new_waypoint_queue

        for elem in current_plan:
            self._waypoints_queue.append((elem,RoadOption.LANEFOLLOW))

        self._stop_waypoint_creation = stop_waypoint_creation

    def _compute_next_waypoints(self, k=1):
        """
        Add new waypoints to the trajectory queue.

        :param k: how many waypoints to compute
        :return:
        """
        # check we do not overflow the queue
        available_entries = self._waypoints_queue.maxlen - len(self._waypoints_queue)
        k = min(available_entries, k)

        for _ in range(k):
            last_waypoint = self._waypoints_queue[-1][0]
            next_waypoints = list(last_waypoint.next(self._sampling_radius))

            if len(next_waypoints)==0:
                break
            elif len(next_waypoints) == 1:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                road_option = RoadOption.LANEFOLLOW
            else:
                road_options_list = self._retrieve_options(
                    next_waypoints, last_waypoint)

                # random choice between the possible options
                road_option = road_options_list[1]  
                #road_option = random.choice(road_options_list)
                next_waypoint = next_waypoints[road_options_list.index(road_option)]
                
                # idx=None
                # for i,wp in enumerate(next_waypoints):
                #     if wp.road_id in ROADS:
                #         next_waypoint=wp
                #         idx=i
                # road_option=road_options_list[idx]

            self._waypoints_queue.append((next_waypoint, road_option))

    def _get_waypoints(self):
        """
        Execute one step of local planning which involves running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

        :param debug: boolean flag to activate waypoints debugging
        :return:
        """

        # not enough waypoints in the horizon? => add more!
        if len(self._waypoints_queue) < int(self._waypoints_queue.maxlen * 0.5) and not self._stop_waypoint_creation:
            self._compute_next_waypoints(k=10)

        #   Buffering the waypoints
        while len(self._waypoint_buffer)<self._buffer_size:
            if self._waypoints_queue:
                self._waypoint_buffer.append(
                self._waypoints_queue.popleft())
            else:
                break

        waypoints=[]

        for i, (waypoint, _) in enumerate(self._waypoint_buffer):
            waypoints.append(waypoint)
            #waypoints.append([waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.rotation.yaw])

        # current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        # target waypoint
        self._target_waypoint, self._target_road_option = self._waypoint_buffer[0]

        # purge the queue of obsolete waypoints
        # vehicle_transform = self._vehicle.get_transform()
        # max_index = -1

        # for i, (waypoint, _) in enumerate(self._waypoint_buffer):
        #     if distance_vehicle(waypoint, vehicle_transform) < self._min_distance:
        #         max_index = i
        # if max_index >= 0:
        #     for i in range(max_index - 1):
        #         self._waypoint_buffer.popleft()
 
        veh_location=self._vehicle.get_location()
        num_waypoint_removed = 0
        for waypoint, _ in self._waypoint_buffer:

            if len(self._waypoints_queue) - num_waypoint_removed == 1:
                min_distance = 1  # Don't remove the last waypoint until very close by
            else:
                min_distance = self._min_distance

            if veh_location.distance(waypoint.transform.location) < min_distance:
                num_waypoint_removed += 1
            else:
                break

        if num_waypoint_removed > 0:
            for _ in range(num_waypoint_removed):
                self._waypoint_buffer.popleft()  
        
        return waypoints

    def _get_hazard(self):
        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")

        # check possible obstacles
        vehicle = self._vehicle_hazard(vehicle_list)

        # check for the state of the traffic lights
        light_state = self._is_light_red_us_style(lights_list)

        return light_state, vehicle

    def _vehicle_hazard(self, vehicle_list):
        """
        Check if a given vehicle is an obstacle in our way. To this end we take
        into account the road and lane the target vehicle is on and run a
        geometry test to check if the target vehicle is under a certain distance
        in front of our ego vehicle.

        WARNING: This method is an approximation that could fail for very large
        vehicles, which center is actually on a different lane but their
        extension falls within the ego vehicle lane.

        :param vehicle_list: list of potential obstacle to check
        :return:
            - vehicle is the blocker object itself
        """

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == self._vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint = self._map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                    target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()
            if is_within_distance_ahead(loc, ego_vehicle_location,
                            self._vehicle.get_transform().rotation.yaw,
                            self._proximity_threshold):
                return target_vehicle

        return None

    def _is_light_red_us_style(self, lights_list):
        """
        This method is specialized to check US style traffic lights.

        :param lights_list: list containing TrafficLight objects
        :return: a tuple given by (bool_flag, traffic_light), where
            - bool_flag is True if there is a traffic light in RED
            affecting us and False otherwise
            - traffic_light is the object itself or None if there is no
            red traffic light affecting us
        """
        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        if ego_vehicle_waypoint.is_intersection:
            # It is too late. Do not block the intersection! Keep going!
            return False

        if self._target_waypoint is not None:
            if self._target_waypoint.is_intersection:
                potential_lights = []
                min_angle = 180.0
                sel_magnitude = 0.0
                sel_traffic_light = None
                for traffic_light in lights_list:
                    loc = traffic_light.get_location()
                    magnitude, angle = compute_magnitude_angle(loc,
                                            ego_vehicle_location,
                                            self._vehicle.get_transform().rotation.yaw)
                    if magnitude < 80.0 and angle < min(25.0, min_angle):
                        sel_magnitude = magnitude
                        sel_traffic_light = traffic_light
                        min_angle = angle

                if sel_traffic_light is not None:
                    if self._last_traffic_light is None:
                        self._last_traffic_light = sel_traffic_light

                    if self._last_traffic_light.state == carla.libcarla.TrafficLightState.Red:
                        return True
                else:
                    self._last_traffic_light = None

        return False

    def _retrieve_options(self,list_waypoints, current_waypoint):
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
            link = self._compute_connection(current_waypoint, next_next_waypoint)
            options.append(link)

        return options

    def _compute_connection(self,current_waypoint, next_waypoint):
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

    def _vehicle_obstacle_detected(self, vehicle_list=None, max_distance=None, up_angle_th=90, low_angle_th=0, lane_offset=0):
        """
        Method to check if there is a vehicle in front of the agent blocking its path.

            :param vehicle_list (list of carla.Vehicle): list contatining vehicle objects.
                If None, all vehicle in the scene are used
            :param max_distance: max freespace to check for obstacles.
                If None, the base threshold value is used
        """
        if self._ignore_vehicles:
            return (False, None, -1)

        if not vehicle_list:
            vehicle_list = self._world.get_actors().filter("*vehicle*")

        if not max_distance:
            max_distance = self._sampling_radius

        ego_transform = self._vehicle.get_transform()
        ego_wpt = self._map.get_waypoint(self._vehicle.get_location())

        # Get the right offset
        if ego_wpt.lane_id < 0 and lane_offset != 0:
            lane_offset *= -1

        # Get the transform of the front of the ego
        ego_forward_vector = ego_transform.get_forward_vector()
        ego_extent = self._vehicle.bounding_box.extent.x
        ego_front_transform = ego_transform
        ego_front_transform.location += carla.Location(
            x=ego_extent * ego_forward_vector.x,
            y=ego_extent * ego_forward_vector.y,
        )

        for target_vehicle in vehicle_list:
            target_transform = target_vehicle.get_transform()
            target_wpt = self._map.get_waypoint(target_transform.location, lane_type=carla.LaneType.Any)

            # Simplified version for outside junctions
            if not ego_wpt.is_junction or not target_wpt.is_junction:

                if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id  + lane_offset:
                    next_wpt = self._local_planner.get_incoming_waypoint_and_direction(steps=3)[0]
                    if not next_wpt:
                        continue
                    if target_wpt.road_id != next_wpt.road_id or target_wpt.lane_id != next_wpt.lane_id  + lane_offset:
                        continue

                target_forward_vector = target_transform.get_forward_vector()
                target_extent = target_vehicle.bounding_box.extent.x
                target_rear_transform = target_transform
                target_rear_transform.location -= carla.Location(
                    x=target_extent * target_forward_vector.x,
                    y=target_extent * target_forward_vector.y,
                )

                if is_within_distance(target_rear_transform, ego_front_transform, max_distance, [low_angle_th, up_angle_th]):
                    return (True, target_vehicle, compute_distance(target_transform.location, ego_transform.location))

            # Waypoints aren't reliable, check the proximity of the vehicle to the route
            else:
                route_bb = []
                ego_location = ego_transform.location
                extent_y = self._vehicle.bounding_box.extent.y
                r_vec = ego_transform.get_right_vector()
                p1 = ego_location + carla.Location(extent_y * r_vec.x, extent_y * r_vec.y)
                p2 = ego_location + carla.Location(-extent_y * r_vec.x, -extent_y * r_vec.y)
                route_bb.append([p1.x, p1.y, p1.z])
                route_bb.append([p2.x, p2.y, p2.z])

                for wp, _ in self._local_planner.get_plan():
                    if ego_location.distance(wp.transform.location) > max_distance:
                        break

                    r_vec = wp.transform.get_right_vector()
                    p1 = wp.transform.location + carla.Location(extent_y * r_vec.x, extent_y * r_vec.y)
                    p2 = wp.transform.location + carla.Location(-extent_y * r_vec.x, -extent_y * r_vec.y)
                    route_bb.append([p1.x, p1.y, p1.z])
                    route_bb.append([p2.x, p2.y, p2.z])

                if len(route_bb) < 3:
                    # 2 points don't create a polygon, nothing to check
                    return (False, None, -1)
                ego_polygon = Polygon(route_bb)

                # Compare the two polygons
                for target_vehicle in vehicle_list:
                    target_extent = target_vehicle.bounding_box.extent.x
                    if target_vehicle.id == self._vehicle.id:
                        continue
                    if ego_location.distance(target_vehicle.get_location()) > max_distance:
                        continue

                    target_bb = target_vehicle.bounding_box
                    target_vertices = target_bb.get_world_vertices(target_vehicle.get_transform())
                    target_list = [[v.x, v.y, v.z] for v in target_vertices]
                    target_polygon = Polygon(target_list)

                    if ego_polygon.intersects(target_polygon):
                        return (True, target_vehicle, compute_distance(target_vehicle.get_location(), ego_location))

                return (False, None, -1)

        return (False, None, -1)
