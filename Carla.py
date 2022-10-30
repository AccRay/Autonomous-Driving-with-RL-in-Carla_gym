import queue
import random,glob,os,sys
import argparse
from queue import Empty
try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

argparser = argparse.ArgumentParser(
        description=__doc__)
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
args=argparser.parse_args()

# Connect to the client and retrieve the world object
# client=carla.Client('0.0.0.0',2000)
client=carla.Client(args.host,args.port)
client.set_timeout(2.0)
client.load_world('Town01')
world=client.get_world()

#change world weather
weather=carla.WeatherParameters(cloudiness=10.0,precipitation=10.0,fog_density=10.0)
world.set_weather(weather)

# Set the spectator with an empty transform
#spectator.set_transform(carla.Transform())
# This will set the spectator at the origin of the map, with 0 degrees
# pitch, yaw and roll - a good way to orient yourself in the map

# Get the blueprint library and filter for the vehicle blueprints
vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
blueprint_library=world.get_blueprint_library()    #拿到这个世界所有物体的蓝图
ego_vehicle_bp=blueprint_library.find('vehicle.audi.a2')
ego_vehicle_bp.set_attribute('color','0,0,0')

# Get the map's spawn points
transform = random.choice(world.get_map().get_spawn_points())   # 找到所有可以作为初始点的位置并随机选择一个
ego_vehicle=world.spawn_actor(ego_vehicle_bp,transform) # 在这个位置生成汽车

#改变车车辆位w
location=ego_vehicle.get_location()
location.x+=10.0
ego_vehicle.set_location(location)
ego_vehicle.set_autopilot(True,8000) # 把它设置成自动驾驶模式
#actor.set_simulate_physics(False)   # 我们可以甚至在中途将这辆车“冻住”，通过抹杀它的物理仿真

#设置同步模式
def sensor_callback(sensor_data,sensor_queue,sensor_name):
    if 'lidar' in sensor_name:
        sensor_data.save_to_disk(os.path.join('./out/output_synchronized','%06d.ply'%sensor_data.frame))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join('./out/output_synchronized','%06d.png'%sensor_data.frame))
    sensor_queue.put((sensor_data,sensor_name))
original_settings=world.get_settings()
settings=world.get_settings()
settings.synchronous_mode=True
settings.fixed_delta_seconds=0.05 #20 fps
world.apply_settings(settings)

#基于汽车构建相机
camera_bp=blueprint_library.find('sensor.camera.rgb')
camera_transform=carla.Transform(carla.Location(x=1.5,z=2.4))
camera=world.spawn_actor(camera_bp,camera_transform,attach_to=ego_vehicle)
camera_queue=queue.Queue()
camera.listen(lambda image:sensor_callback(image,camera_queue,'camera'))

#基于汽车构建激光雷达
lidar_bp=blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels',str(32))
lidar_bp.set_attribute('points_per_second',str(90000))
lidar_bp.set_attribute('rotation_frequency',str(40))
lidar_bp.set_attribute('range',str(20))

lidar_location=carla.Location(0,0,2)
lidar_rotation=carla.Rotation(0,0,0)
lidar_transform=carla.Transform(lidar_location,lidar_rotation)
lidar=world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)
lidar_queue=queue.Queue()
lidar.listen(lambda point_cloud:\
        sensor_callback(point_cloud,lidar_queue,'lidar'))

traffic_manager=client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)

try:
    while True:
        world.tick()

        # Retrieve the spectator object
        spectator = world.get_spectator()
        # Get the location and rotation of the spectator through its transform
        transform = ego_vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location+carla.Location(z=20),carla.Rotation(pitch=-90)))

        try:
            camera_data=camera_queue.get(block=True,timeout=1.0)
            lidar_data=lidar_queue.get(block=True,timeout=1.0)
        except Empty:
            print("Some of the sensor information is missed")

        #location = transform.location
        #rotation = transform.rotation
finally:
    world.apply_settings(original_settings)
    print('destroy actors')
    client.apply_batch(carla.DestroyActor(ego_vehicle))
    lidar.destroy()
    camera.destroy()
    print('done')

# Spawn 50 vehicles randomly distributed throughout the map 
# for each spawn point, we choose a random vehicle from the blueprint library
"""for i in range(0,50):
    world.try_spawn_actor(random.choice(vehicle_blueprints), transform)

for vehicle in world.get_actors().filter('vehicle'):
    vehicle.set_autopilot(True)"""

#ego_vehicle.destroy()   # 如果注销单个Actor
#client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])   # 如果你有多个Actor 存在list里，想一起销毁。

"""camera_init_trans=carla.Transform(carla.Locatsssssssion(z=1.5))
camera_bp=world.get_blueprint_library().find('sensor.camera.rgb')
camera=world.spawn_actor(camera_bp,camera_init_trans,attach_to=ego_vehicle)
camera.listen(lambda image: image.save_to_disk('out/%06d.png'%image.frame))"""

"""settings=world.get_settings()
settings.synchronous_mode=True
settings.fixed_delta_seconds=0.05
world.apply_settings(settings)

blueprint=world.get_blueprint_library()
camera=world.spawn_actor(blueprint,transform)
image_queue=queue.Queue()
camera.listen(image_queue.put)

while True:
    world.tick()
    image=image_queue.get()"""