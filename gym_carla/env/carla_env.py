from queue import Queue
import gym
import logging
import carla
import random
import time
import numpy as np
from gym_carla.env.util import *
from gym_carla.env.util.sensor import CollisionSensor

class CarlaEnv:
    def __init__(self,args) -> None:
        super().__init__()
        self.roads=args.roads 
        self.host=args.host
        self.port=args.port
        self.tm_port=args.tm_port
        self.sync=args.sync
        self.fps=args.fps
        self.ego_filter=args.filter
        self.loop=args.loop
        self.agent=args.agent
        self.behavior=args.behavior
        self.seed=args.seed
        self.debug=args.debug
        self.res=args.res
        self.num_of_vehicles=args.num_of_vehicles
        self.seed=args.seed
        self.sampling_resolution=args.sampling_resolution

        logging.info('listening to server %s:%s', args.host, args.port)
        self.client=carla.Client(self.host,self.port)
        self.client.set_timeout(5.0)
        self.world=self.client.load_world(args.map)
        self.map=self.world.get_map()
        logging.info('Carla server connected')

        #generate ego vehicle spawn points in chosen route
        self.ego_spawn_points=[]
        spawn_points=list(self.map.get_spawn_points())
        for spawn_point in spawn_points:
            if self.map.get_waypoint(spawn_point.location).road_id in self.roads:
                self.ego_spawn_points.append(spawn_point)
        #random.seed(self.seed)

        # Set fixed simulation step for synchronous mode
        self.origin_settings=self.world.get_settings()
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

        #Get actors polygon list
        self.vehicle_polygons=[]
        vehicle_poly_dict=self._get_actor_polygons('vehicle.*')
        self.vehicle_polygons.append(vehicle_poly_dict)

        #Ego vehicle
        self.ego_vehicle=None

        #Collision sensor
        self.collision_sensor=None

        #thread blocker
        self.sensor_queue=Queue(maxsize=10)
        self.camera=None

    def __del__(self):
        logging.info('\n Destroying all vehicles')
        self._clear_actors(['vehicle.*','sensor.other.collison'])

    def reset(self):
        if self.ego_vehicle is not None:
            self._clerar_actors(['sensor.other.collison',self.ego_filter,'sensor.camera.rgb'])
            self.ego_vehicle=None
            self.collision_sensor=None
            self.camera=None
            self.sensor_queue.clear()

        #try to spawn ego vehicle 
        while(self.ego_vehicle is None):
            transform=random.choice(self.ego_spawn_points)
            self.ego_vehicle=self._try_spawn_ego_vehicle_at(transform)
        self.collision_sensor=CollisionSensor(self.ego_vehicle)
        self.ego_vehicle.set_autopilot(True,self.tm_port)

        #test code for synchronous mode
        camera_bp=self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform=carla.Transform(carla.Location(x=1.5,z=2.4))
        self.camera=self.world.spawn_actor(camera_bp,camera_transform,attach_to=self.ego_vehicle)
        self.camera.listen(lambda image:self._sensor_callback(image,self.sensor_queue))
        # 

        if self.sync:
            self.world.tick()
            spectator=self.world.get_spectator()
            transform=self.ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location+carla.Location(z=50),
                carla.Rotation(pitch=-90)))
        else:
            self.world.wait_for_tick()
        return self._get_state()

    def step(self, action):
        if self.sync:
            self.world.tick()
            spectator=self.world.get_spectator()
            transform=self.ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location+carla.Location(z=50),
                carla.Rotation(pitch=-90)))

            camera_data=self.sensor_queue.get(block=True,timeout=1.0)
        else:
            temp=self.world.wait_for_tick()
            self.world.on_tick(lambda _:{})

        return 
        #return self._get_state(),self._get_reward(),self._terminal(),self._get_info()

    def close(self):
        return
        
    def seed(self,seed=None):
        return
    
    def render(self,mode):
        pass

    def _get_state(self):
        """Get the current simulation state"""
        pass

    def _get_reward(self):
        """Calculate the step reward"""
        pass

    def _terminal(self):
        """Calculate whether to terminate the current episode"""
        pass

    def _get_info(self):
        """Rerurn simulation running information"""
        pass
    
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

        if vehicle is not None:
            return vehicle
        
        return None         
    
    def _spawn_companion_vehicles(self):
        """
        Spawn surrounding vehcles of this simulation
        each vehicle is set to autopilot mode and controled by Traffic Maneger
        """
        traffic_manager=self.client.get_trafficmanager(self.tm_port)
        # every vehicle keeps a distance of 3.0 meter
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        # Set physical mode only for cars around ego vehicle to save computation
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_hybrid_physics_radius(70.0)
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
                print("Future Actor",response.actor_id)
                self.companion_vehicles.append(self.world.get_actor(response.actor_id))
                #set vehicles to ignore traffic lights
                traffic_manager.ignore_lights_percentage(
                    self.world.get_actor(response.actor_id),0)
                #print(self.world.get_actor(response.actor_id).attributes)
        
        #Set all cars as dangerous car to test collison sensor
        #crazy car ignore traffic light, do not keep safe distance and very fast
        for i in range(len(self.companion_vehicles)):
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