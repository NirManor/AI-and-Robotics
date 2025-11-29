
import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from building_blocks import Building_Blocks
import time
import matplotlib.pyplot as plt

#our imports
from pathlib import Path

def main():
    inflation_factors = np.linspace(1.0, 1.8, 9)
    times = []
    is_collision_instances = []

    ground_truth = []
    for inflation_factor in inflation_factors:
        ur_params = UR5e_PARAMS(inflation_factor=inflation_factor)
        env = Environment(env_idx=0)
        transform = Transform(ur_params)
        bb = Building_Blocks(transform=transform, ur_params=ur_params, env=env, resolution=0.1, p_bias=0.03) 
        # change the path
        file_path = Path.joinpath(Path.cwd(), 'random_samples\\random_samples_100k.npy')
        random_samples = np.load(file_path)
        
        # TODO
        # ADD YOUR CODE HERE
        start_time = time.time()
        #false_negative_count = 0
        false_positive_count = 0

        for index, sample in enumerate(random_samples):
            if index>10000:
                test=index
            is_collision = bb.is_in_collision(sample)
            if inflation_factor == 1.0:
                if is_collision:
                    ground_truth.append(1)
                else:
                    ground_truth.append(0)
            else:
                #the CD returns that there isn't a collision while there is in the real world.
                """if not is_collision and ground_truth[index]:
                    false_negative_count += 1"""
                #the CD returns that there is a collision while there isn't in the real world.
                if is_collision and not ground_truth[index]:
                    false_positive_count += 1
                

        #is_collision_instances.append(false_negative_count)
        is_collision_instances.append(false_positive_count)

        end_time = time.time()
        times.append(end_time - start_time)

    fig = plt.figure()
    ax1 = fig.add_subplot()
    ax1.set_xlabel('min radii factor')
    ax2 = ax1.twinx()
    ax1.set_ylabel('time (s)', color='blue')
    ax2.set_ylabel('False Negative Instances', color='red') 
    ax1.scatter(inflation_factors, times, c='blue')
    ax2.scatter(inflation_factors, is_collision_instances, c='red')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax2.tick_params(axis='y', labelcolor='red')
    fig.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()



