import gym
import logging
from gym_carla.env.carla_env import CarlaEnv
from gym_carla.env.settings import ARGS
from temp import Temp

def main():
    args=ARGS.parse_args()

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
                f.write(str(reward)+'\t'+str(info)+'\n')
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