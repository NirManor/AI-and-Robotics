import numpy as np
from environment import Environment
from kinematics import UR5e_PARAMS, Transform
from planners import RRT_STAR
from building_blocks import Building_Blocks
from visualizer import Visualize_UR

def main():
    ur_params = UR5e_PARAMS(inflation_factor=1.3)
    env = Environment(env_idx=1)
    transform = Transform(ur_params)
    
    bb = Building_Blocks(transform=transform, 
                        ur_params=ur_params, 
                        env=env,
                        resolution=0.1, 
                        p_bias=0.05,)
    
    visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)

    # --------- configurations-------------
    home = np.deg2rad([0, -90, 0, -90, 0, 0])
    conf2 = np.deg2rad([20, -90, 90, -90, -90, -10])
    test=bb.is_in_collision(conf2)
   
    # ---------------------------------------

    visualizer.show_conf(home)

    #Question 2.3
    """home_rad = np.array([-0.694, -1.376, -2.212, -1.122, 1.570, -2.26])"""
    
    #Question 3.1
    """collision_free_conf_3_1 = np.array([0.694, -1.376, -2.212, -1.122, 1.570, -2.26])
    
    in_collision_conf_3_1 = np.array([-0.694, -1.376, 2.55, -1.122, -1.570, 2.26])"""

    #Question 3.2
    """collision_free_conf_3_2 = np.array([-0.5, -2, -0.5, 1, 1, -1])

    in_collision_conf_3_2 = np.array([-0.5, -1, -0.5, 1, 1, -1])"""
    
    #Question 4
    """conf1 = np.deg2rad([80, -72, 101, -120, -90, -10])
    conf2 = np.deg2rad([20, -90, 90, -90, -90, -10])
    resolutions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
    for resolution in resolutions:
        bb.resolution = resolution
        bb.local_planner(conf1, conf2)
        print("\n")"""

    
       
if __name__ == '__main__':
    main()



