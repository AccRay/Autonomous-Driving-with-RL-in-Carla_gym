import logging
import carla
import random
import time
import numpy as np
from queue import Queue
from gym_carla.env.util.pid_controller import *
from gym_carla.env.util.misc import draw_waypoints,get_speed,get_acceleration
from gym_carla.env.sensor import CollisionSensor,LaneInvasionSensor
from gym_carla.env.route_planner import GlobalPlanner,LocalPlanner

class CarlaEnv:
    def __init__(self,args) -> None:
        super().__init__()
        self.host=args.host
        self.port=args.port
        self.tm_port=args.tm_port
        self.sync=args.sync
        self.fps=args.fps
        self.ego_filter=args.filter
        self.loop=args.loop
        self.agent=args.agent
        self.behavior=args.behavior
        self.res=args.res
        self.num_of_vehicles=args.num_of_vehicles
        self.sampling_resolution=args.sampling_resolution
        self.hybrid=args.hybrid
        self.stride=args.stride
        self.buffer_size=args.buffer_size

        logging.info('listening to server %s:%s', args.host, args.port)
        self.client=carla.Client(self.host,self.port)
        self.client.set_timeout(5.0)
        self.world=self.client.load_world(args.map)
        self.map=self.world.get_map()
        self.origin_settings=self.world.get_settings()
        logging.info('Carla server connected')

        # Record the time of total steps
        self.reset_step = 0
        self.total_step = 0
        self.next_wps=None  #ego vehicle's following waypoint list
        self.vehicle_front=None #the vehicle in front of ego vehicle

        #generate ego vehicle spawn points on chosen route
        self.global_planner=GlobalPlanner(self.map,self.sampling_resolution)
        self.local_planner=None
        self.ego_spawn_waypoints=self.global_planner.get_spawn_points()
        #former_wp record the ego vehicle waypoint of former step

        #arguments for debug
        self.debug=args.debug
        self.seed=args.seed
        self.former_wp=None

        #arguments for caculating reward
        self.TTC_THRESHOLD=args.TTC_th
        self.speed_limit=args.speed_limit
        self.penalty=args.penalty
        self.last_acc=carla.Vector3D() #ego vehicle acceration in last step
        self.reward_info=None

        if self.debug:
            #draw_waypoints(self.world,self.global_panner.get_route())
            random.seed(self.seed)

        # Set fixed simulation step for synchronous mode
        if self.sync:
            settings=self.world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode=True
                settings.fixed_delta_seconds=1.0/self.fps
                self.world.apply_settings(settings)

        #Set weather
        #self.world.set_weather(carla.WeatherParamertes.ClearNoon)

        #Spawn surrounding vehicles
        self.companion_vehicles=[]
        self._spawn_companion_vehicles()

        #Ego vehicle
        self.ego_vehicle=None
        #the vehicle in front of ego vehicle
        self.vehicle_front=None

        #Collision sensor
        self.collision_sensor=None
        self.lane_invasion_sensor=None

        #thread blocker
        self.sensor_queue=Queue(maxsize=10)
        self.camera=None

    def __del__(self):
        logging.info('\n Destroying all vehicles')
        self.world.apply_settings(self.origin_settings)
        self._clear_actors(['vehicle.*','sensor.other.collison','sensor.camera.rgb','sensor.other.lane_invasion'])

    def reset(self):
        if self.ego_vehicle is not None:
            self._clear_actors(['sensor.other.collison',self.ego_filter,'sensor.camera.rgb','sensor.other.lane_invasion'])
            self.ego_vehicle=None
            self.collision_sensor=None
            self.lane_invasion_sensor=None
            self.camera=None
            self.sensor_queue.clear()

        #Get actors polygon list
        self.vehicle_polygons=[]
        vehicle_poly_dict=self._get_actor_polygons('vehicle.*')
        self.vehicle_polygons.append(vehicle_poly_dict)

        #try to spawn ego vehicle 
        while(self.ego_vehicle is None):
            spawn_waypoint=random.choice(self.ego_spawn_waypoints)
            self.former_wp=spawn_waypoint
            self.ego_vehicle=self._try_spawn_ego_vehicle_at(spawn_waypoint.transform)
        self.collision_sensor=CollisionSensor(self.ego_vehicle)
        self.lane_invasion_sensor=LaneInvasionSensor(self.ego_vehicle)

        #let the client interact with server
        if self.sync:
            self.world.tick()

            spectator=self.world.get_spectator()
            transform=self.ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location+carla.Location(z=50),
                carla.Rotation(pitch=-90)))
        else:
            self.world.wait_for_tick()
       
        """Attention:
        get_location() Returns the actor's location the client recieved during last tick. The method does not call the simulator.
        Hence, upon initializing, the world should first tick before calling get_location, or it could cause fatal bug"""
        #ego_wp=self.map.get_waypoint(self.ego_vehicle.get_location())

        #add route planner for ego vehicle
        self.local_planner=LocalPlanner(self.ego_vehicle,self.buffer_size)
        #self.local_planner.set_global_plan(self.global_planner.get_route(
        #    self.map.get_waypoint(self.ego_vehicle.get_location())))
        self.next_wps,_,self.vehicle_front=self.local_planner.run_step()

        #set ego vehicle controller
        #self.ego_vehicle.set_autopilot(self.debug,self.tm_port)
        if self.debug:
            self.controller=VehiclePIDController(self.ego_vehicle,
                {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 1.0 / 20.0},
                {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': 1.0 / 20.0})

        #test code for synchronous mode
        camera_bp=self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform=carla.Transform(carla.Location(x=1.5,z=2.4))
        self.camera=self.world.spawn_actor(camera_bp,camera_transform,attach_to=self.ego_vehicle)
        self.camera.listen(lambda image:self._sensor_callback(image,self.sensor_queue))
        # 

        #Update timesteps
        self.time_step=0
        self.reset_step+=1

        # return state information
        return self._get_state({'waypoints':self.next_wps,'vehicle_front':self.vehicle_front})

    def step(self,action):
        # Calculate acceleration and steering
        self.discrete=False
        if self.discrete:
            acc = self.discrete_act[0][action//self.n_steer]
            steer = self.discrete_act[1][action%self.n_steer]
        else:
            acc = action[0]
            steer = action[1]

        #route planner
        self.next_wps,_,self.vehicle_front=self.local_planner.run_step()
        ego_wp=self.map.get_waypoint(self.ego_vehicle.get_location(),project_to_road=False)

        if self.debug:
            if self.next_wps[0].id !=self.former_wp.id:
                self.former_wp=self.next_wps[0]

            draw_waypoints(self.world, [self.next_wps[0]], 10.0,z=1)
            if ego_wp:
                draw_waypoints(self.world, [ego_wp], 1.0)
            control=self.controller.run_step(self.speed_limit,self.next_wps[0])
        
            # Convert acceleration to throttle and brake
            if acc > 0:
                throttle = np.clip(acc/4,0,1)
                brake = 0
            else:
                throttle = 0
                brake = np.clip(-acc/3,0,1)
            #act=carla.VehicleControl(throttle=float(throttle),steer=float(steer),brake=float(brake))

        if self.sync:
            if self.debug:
                self.ego_vehicle.apply_control(control)

            self.world.tick()
            spectator=self.world.get_spectator()
            transform=self.ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location+carla.Location(z=50),
                carla.Rotation(pitch=-90)))

            camera_data=self.sensor_queue.get(block=True,timeout=1.0)
        else:
            temp=self.world.wait_for_tick()
            self.world.on_tick(lambda _:{})

        if self.ego_vehicle.get_location().distance(self.former_wp.transform.location)>=self.sampling_resolution:
            self.former_wp=self.next_wps[0]

        #Update timesteps
        self.time_step+=1
        self.total_step+=1

        reward=self._get_reward()
        self.last_acc=self.ego_vehicle.get_acceleration()

        return self._get_state({'waypoints':self.next_wps,'vehicle_front':self.vehicle_front}),\
            reward,self._terminal(),self._get_info()
        #return self._get_state(),self._get_reward(),self._terminal(),self._get_info()
        
    def seed(self,seed=None):
        return
    
    def render(self,mode):
        pass
    
    def _get_state(self,dict):
        wps=[]
        if dict['waypoints']:
            for wp in dict['waypoints']:
                wps.append((wp.transform.location.x, wp.transform.location.y, wp.s))
        
        if dict['vehicle_front']:
            vehicle_front=dict['vehicle_front']
            distance=self.ego_vehicle.get_location().distance(vehicle_front.get_location())
            relative_speed=abs(get_speed(self.ego_vehicle)-get_speed(vehicle_front))
            vfl=(distance, relative_speed, self.map.get_waypoint(vehicle_front.get_location()).s)
        else:
            #No vehicle front
            vfl=None
        
        return {'waypoints':wps,'vehicle_front':vfl}

    def _get_reward(self):
        """Calculate the step reward:
        TTC: Time to collide with front vehicle
        Eff: Ego vehicle efficiency, speed ralated
        Com: Ego vehicle comfort, ego vehicle acceration change rate 
        Lcen: Distance between ego vehicle location and lane center
        """
        ego_speed=get_speed(self.ego_vehicle)
        TTC=float('inf')
        if self.vehicle_front:
            distance=self.ego_vehicle.get_location().distance(self.vehicle_front.get_location())
            rel_speed=abs(ego_speed-get_speed(self.vehicle_front))
            TTC=distance/rel_speed
        fTTC=-math.exp(-TTC)
        # if TTC>=0 and TTC<=self.TTC_THRESHOLD:
        #     fTTC=np.log(TTC/self.TTC_THRESHOLD)
        # else:
        #     fTTC=0

        if ego_speed>self.speed_limit:
            fEff=0
        else:
            fEff=-(1-ego_speed/self.speed_limit)

        cur_acc=self.ego_vehicle.get_acceleration()
        jerk=math.sqrt((cur_acc.x-self.last_acc.x)**2+(cur_acc.y-self.last_acc.y)**2+(cur_acc.z-self.last_acc.z)**2)/(1.0/self.fps)
        # the maximum range is change from -3 to 3 in 0.1 s, then the jerk = 60
        fCom=-(jerk)/1200    

        lane_center=self.map.get_waypoint(self.ego_vehicle.get_location(),project_to_road=True)
        Lcen=lane_center.transform.location.distance(self.ego_vehicle.get_location())
        if Lcen>lane_center.lane_width/2:
            fLcen=-1
        else:
            fLcen=-Lcen/(lane_center.lane_width/2)

        self.reward_info={'TTC':fTTC, 'Comfort':fCom, 'Efficiency':fEff, 'Lane_center':fLcen}
        return fTTC+fEff+fCom+fLcen-self.penalty*self._terminal()

    def _terminal(self):
        """Calculate whether to terminate the current episode"""
        if len(self.collision_sensor.get_collision_history())!=0:
            logging.warn('collison happend')
            return True
        if self.map.get_waypoint(self.ego_vehicle.get_location()) is None:
            logging.warn('vehicle drive out of road')
            return True
        if self.lane_invasion_sensor.get_invasion_count()!=0:
            LaneInvasionSensor.count=0
            logging.warn('lane invasion occur')
            return True
        if len(self.next_wps)==0:
            logging.info('vehicle reach destination, simulation terminate')
            return True

        """A little bug yet to surface:
        Here we set vehicle reach destination to terminal, and this might cause trouble when caculating reward,
        since this would add penalty to reward. Normaly we should add another function as _truncated to differentiate
        between correct termination and premature truncation"""

        return False

    def _get_info(self):
        """Rerurn simulation running information"""
        return self.reward_info
    
    def _sensor_callback(self,sensor_data,sensor_queue):
        array=np.frombuffer(sensor_data.raw_data,dtype=np.dtype('uint8'))
        # image is rgba format
        array=np.reshape(array,(sensor_data.height,sensor_data.width,4))
        array=array[:,:,:3]
        sensor_queue.put((sensor_data.frame,array))

    def _create_vehicle_blueprint(self,actor_filter,ego=False,color=None,number_of_wheels=[4]):
        """Create the blueprint for a specific actor type.

        Args:
            actor_filter: a string indicating the actor type, e.g, 'vehicle.lincoln*'.

        Returns:
            bp: the blueprint object of carla.
        """
        blueprints=list(self.world.get_blueprint_library().filter(actor_filter))
        if not ego:
            for bp in blueprints:
                if bp.has_attribute(self.ego_filter):
                    blueprints.remove(bp)
    
        blueprint_library=[]
        for nw in number_of_wheels:
            blueprint_library=blueprint_library+[x for x in blueprints if int(x.get_attribute('number_of_wheels'))==nw]
        bp=random.choice(blueprint_library)
        if bp.has_attribute('color'):
            if color is None:
                color=random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color',color)
        if bp.has_attribute('driver_id'):
            driver_id=random.choice(bp.get_attribute('driver_id').recommended_values)
            bp.set_attribute('driver_id',driver_id)
        if not ego:
            bp.set_attribute('role_name','autopilot')
        else:
            bp.set_attribute('role_name','hero')
        return bp

    def _init_renderer(self):
        """Initialize the birdeye view renderer."""
        pass

    def _set_synchronous_mode(self,synchronous=True):
        """Set whether to use the synchronous mode."""
        pass

    def _try_spawn_ego_vehicle_at(self,transform):
        """Try to spawn a  vehicle at specific transform 
        Args:
            transform: the carla transform object.

        Returns:
            Bool indicating whether the spawn is successful.
        """   
        vehicle = None
        # Check if ego position overlaps with surrounding vehicles
        overlap = False
        for idx, poly in self.vehicle_polygons[-1].items():
            poly_center = np.mean(poly, axis=0)
            ego_center = np.array([transform.location.x, transform.location.y])
            dis = np.linalg.norm(poly_center - ego_center)
            if dis > 8:
                continue
            else:
                overlap = True
                break

        if not overlap:
            ego_bp=self._create_vehicle_blueprint(self.ego_filter,ego=True,color='0,255,0')
            vehicle = self.world.try_spawn_actor(ego_bp, transform)
            if vehicle is None:
                logging.warn("Ego vehicle generation fail")
        
        # if self.debug:
        #     vehicle.show_debug_telemetry()

        return vehicle         
    
    def _spawn_companion_vehicles(self):
        """
        Spawn surrounding vehcles of this simulation
        each vehicle is set to autopilot mode and controled by Traffic Maneger
        """
        traffic_manager=self.client.get_trafficmanager(self.tm_port)
        # every vehicle keeps a distance of 3.0 meter
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        # Set physical mode only for cars around ego vehicle to save computation
        if self.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)

        """Attention:Do not set global_percentage_speed_difference parameter as 100,
        all vehicles' velocity would be 0 in this way"""
        traffic_manager.global_percentage_speed_difference(-0)

        if self.sync:
            traffic_manager.set_synchronous_mode(True)

        spawn_points=self.map.get_spawn_points()
        num_of_spawn_points=len(spawn_points)

        if self.num_of_vehicles<num_of_spawn_points:
            random.shuffle(spawn_points)
        else:
            msg='requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg,self.num_of_vehicles,num_of_spawn_points)
            self.num_of_vehicles=num_of_spawn_points-1
        
        # Use command to apply actions on batch of data
        SpawnActor=carla.command.SpawnActor
        SetAutopilot=carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor #FutureActor is eaqual to 0
        command_batch=[]

        for i,transform in enumerate(spawn_points):
            if i>=self.num_of_vehicles:
                break

            blueprint=self._create_vehicle_blueprint('vehicle.*',number_of_wheels=[4])
            #Spawn the cars and their autopilot all together
            command_batch.append(SpawnActor(blueprint,transform).
                then(SetAutopilot(FutureActor,True,self.tm_port)))
            
        #execute the command batch
        for (i,response) in enumerate(self.client.apply_batch_sync(command_batch,self.sync)):
            if response.has_error():
                logging.error(response.error)
            else:
                #print("Future Actor",response.actor_id)
                self.companion_vehicles.append(self.world.get_actor(response.actor_id))
                #set vehicles to ignore traffic lights
                traffic_manager.ignore_lights_percentage(
                    self.world.get_actor(response.actor_id),0)
                #print(self.world.get_actor(response.actor_id).attributes)
        
        #Set car as dangerous car to test collison sensor
        #crazy car ignore traffic light, do not keep safe distance and very fast
        for i in range(1):
            danger_car=self.companion_vehicles[i]
            traffic_manager.ignore_lights_percentage(danger_car,100)
            traffic_manager.distance_to_leading_vehicle(danger_car,0)
            traffic_manager.vehicle_percentage_speed_difference(danger_car,-100)

        msg='requested %d vehicles, generate %d vehicles, press Ctrl+C to exit.'
        logging.info(msg,self.num_of_vehicles,len(self.companion_vehicles))

    def _try_spawn_random_walker_at(self,transform):
        """Try to spawn a walker at specific transform with random bluprint.

        Args:
            transform: the carla transform object.

        Returns:
            Bool indicating whether the spawn is successful.
        """
        pass

    def _get_actor_polygons(self,filt):
        """Get the bounding box polygon of actors.
        Args:
            filt: the filter indicating what type of actors we'll look at.

        Returns:
            actor_poly_dict: a dictionary containing the bounding boxes of specific actors.
        """
        actor_poly_dict={}
        for actor in self.world.get_actors().filter(filt):
            # Get x, y and yaw of the actor
            trans=actor.get_transform()
            x=trans.location.x  
            y=trans.location.y
            yaw=trans.rotation.yaw/180*np.pi
              # Get length and width
            bb=actor.bounding_box
            l=bb.extent.x
            w=bb.extent.y
            # Get bounding box polygon in the actor's local coordinate
            poly_local=np.array([[l,w],[l,-w],[-l,-w],[-l,w]]).transpose()
            # Get rotation matrix to transform to global coordinate
            R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
            # Get global bounding box polygon
            poly=np.matmul(R,poly_local).transpose()+np.repeat([[x,y]],4,axis=0)
            actor_poly_dict[actor.id]=poly
        return actor_poly_dict
    
    def _clear_actors(self,actor_filters):
        """Clear specific actors"""
        for actor_filter in actor_filters:
            self.client.apply_batch([carla.command.DestroyActor(x) 
                for x in self.world.get_actors().filter(actor_filter)])
        # for actor_filter in actor_filters:
        #     for actor in self.world.get_actors().filter(actor_filter):
        #         if actor.is_alive:
        #             if actor.type_id =='controller.ai.walker':
        #                 actor.stop()
        #             actor.destroy()