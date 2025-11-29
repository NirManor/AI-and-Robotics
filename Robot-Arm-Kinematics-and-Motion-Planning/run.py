import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR


def main():
    ur_params = UR5e_PARAMS(inflation_factor=1)
    env = Environment(env_idx=1)
    transform = Transform(ur_params)
    
    bb = Building_Blocks(transform=transform, 
                        ur_params=ur_params, 
                        env=env,
                        resolution=0.1, 
                        p_bias=0.50,)

    # conf1_rad = np.deg2rad([80, -72, 101, -120, -90, -10])
    conf2_rad = np.deg2rad([20, -90, 90, -90, -90, -10])

    conf1_rad = np.deg2rad([90, -56, 101, -120, -90, -10])

    # Test the local planner with the given configurations
    resolution_values = [0.1, 0.05]  # Add more values if needed
    visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)

    for resolution in resolution_values:
        bb.resolution = resolution

        print(f"Testing with resolution: {resolution}")
        path_ok=bb.local_planner(conf1_rad, conf2_rad,visualizer)
        # print(f"Local Planner Result: {bb.local_planner(conf1_rad, conf2_rad,visualizer)}")
    resolution = 0.1
    test_cnonfig=bb.sample(np.deg2rad([0, -90, 0, -90, 0,0 ]))
    # visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)




    num_intermediate_configs = max(int(np.ceil(np.linalg.norm(conf2_rad - conf1_rad) / resolution)), 2)
    intermediate_configs = np.linspace(conf1_rad, conf2_rad, num_intermediate_configs)

    # Check for collisions in intermediate configurations
    for conf in intermediate_configs:
        visualizer.show_conf(conf)


    # # --------- configurations-------------
    # home = np.deg2rad([0, -90, 0, -90, 0,0 ])
    #
    # # ---------------------------------------
    #
    # visualizer.show_conf(test_cnonfig)
    
   

if __name__ == '__main__':
    main()



