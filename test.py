import gym
import argparse
import logging
from gym_carla.env.carla_env import CarlaEnv
from temp import Temp

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        default=True,
        help='Print debug information')
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
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        default=True,
        help='Synchronous mode execution')
    argparser.add_argument(
        '--fps',metavar='FPS',
        default=20,type=int,
        help="The fps of server running speed")
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.audi.a2',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        #
        default='True',
        #
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)
    argparser.add_argument(
        '-m','--map',type=str,
        choices=['Town01','Town02'],
        help='Choose one the possible world map',
        default='Town01')
    argparser.add_argument(
        '-n','--num_of_vehicles',type=int,
        help='Total vehicles number which run in simulation',
        default=30)
    argparser.add_argument(
        '-sa','--sampling_resolution',type=float,
        help='Distance between generated two waypoints',
        default=4.0)
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with traffic manager (default: 8000)')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        default=True,
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')
    argparser.add_argument(
        '--stride',type=int,
        default=10,
        help='The number of upfront waypoints each state should include')
    argparser.add_argument(
        '--buffer-size',type=int,
        default=10,
        help='The number of look-ahead waypoints in each step')
    args=argparser.parse_args()

    log_level=logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    #env=gym.make('CarlaEnv-v0')
    env=CarlaEnv(args)
    #e=Temp(args)
    
    done=False
    with open('./out/state_test.txt','w') as f:
        pass
    env.reset()

    try:
        while(True):
            action=[2.0,0.0]
            state,reward,done,info=env.step(action)
            
            with open('./out/state_test.txt','a') as f:
                for loc in state['waypoints']:
                    f.write(str(loc)+'\n')
                f.write('\n')
                f.write(str(state['vehicle_front'])+'\n')
                f.write('\n')

            if done:
                break
            #e.step()
    except KeyboardInterrupt:
        pass
    finally:
        env.__del__()
        logging.info('\nDone.')
    
    # for i in range(100):
    #     env.reset()
    #     while not done:
    #         next_state,reward,done,info=env.step([0.3,0])
        
    #     done=False
    # while(True):
    #     continue

if __name__=='__main__':
    main()