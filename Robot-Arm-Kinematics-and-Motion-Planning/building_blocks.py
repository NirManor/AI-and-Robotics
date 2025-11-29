import numpy as np
from visualizer import Visualize_UR

class Building_Blocks(object):
    '''
    @param resolution determines the resolution of the local planner(how many intermidiate configurations to check)
    @param p_bias determines the probability of the sample function to return the goal configuration
    '''
    def __init__(self, transform, ur_params, env, resolution=0.1, p_bias=0.05):
        self.transform = transform
        self.ur_params = ur_params
        self.env = env
        self.resolution = resolution
        self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3 ,0.2 ,0.1 ,0.07 ,0.05])
        
    def sample(self, goal_conf) -> np.array:
        """
        sample random configuration
        @param goal_conf - the goal configuration
        """
        # TODO 
        # hint - use self.ur_params.mechamical_limits
        
        # return np.array(conf)
        if np.random.rand() < self.p_bias:
            return goal_conf
        else:
            conf = np.zeros(len(goal_conf))
            for i, joint_name in enumerate(self.ur_params.mechamical_limits.keys()):
                # Sample random values within mechanical limits for each joint
                conf[i] = np.random.uniform(self.ur_params.mechamical_limits[joint_name][0],
                                            self.ur_params.mechamical_limits[joint_name][1])
            return conf

    def check_if_sphere_intersect(self,sphere_1, sphere_radius1,sphere_2, sphere_radius2):
        distance_between_spheres = np.linalg.norm(sphere_1[:3] - sphere_2[:3])
        distance_between_radiuses = sphere_radius1 + sphere_radius2
        if (distance_between_spheres < distance_between_radiuses):
            return True


    def is_in_collision(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        # TODO 
        # hint: use self.transform.conf2sphere_coords(), self.ur_params.sphere_radius, self.env.obstacles
        # arm - arm collision
        
        # arm - obstacle collision

        # self.transform.conf2sphere_coords - returns the coordinates of the spheres along the manipulator's links
        # for a given configuration, in the base_link frame
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        global_sphere_coords_copy = global_sphere_coords.copy()
        keys_list = list(global_sphere_coords_copy.keys())
        del global_sphere_coords_copy[keys_list[0]]

        sphere_radius = self.ur_params.sphere_radius

        is_collision = False
        # arm - arm collision
        for joint_1, spheres_1 in global_sphere_coords.items():
            if len(global_sphere_coords_copy) <= 1:
                break
            keys_list = list(global_sphere_coords_copy.keys())
            del global_sphere_coords_copy[keys_list[0]]

            for joint_2, spheres_2 in global_sphere_coords_copy.items():
                for sphere_1 in spheres_1:
                    for sphere_2 in spheres_2:
                        distance_between_spheres=np.linalg.norm(sphere_1[:3] - sphere_2[:3])
                        distance_between_radiuses =sphere_radius[joint_1]+sphere_radius[joint_2]
                        if (distance_between_spheres<distance_between_radiuses):
                            print("arm - arm collision detected:\n {} -> {}" .format(joint_1,joint_2))
                            return True
                        #     return True

        # arm - obstacle collision
        obstacles = self.env.obstacles
        is_collision_floor=False
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                for obs in obstacles:

                    distance_between_spheres = np.linalg.norm(sphere[:3] - obs[:3])
                    distance_between_radiuses = sphere_radius[joint] + self.env.radius
                    if (distance_between_spheres < distance_between_radiuses):
                        print("arm - obstacle collision:\n {} ".format(joint))
                        is_collision_floor=True

                    distance_between_sphere_and_floor= sphere[2]-0
                    if sphere[2]<0:
                        test=0
                    if (sphere[2] + sphere_radius['shoulder_link'] - sphere_radius[joint]) < 0:
                    # if sphere[2]< 0:
                        print("arm -floor collision:\n {} ".format(joint))
                        is_collision_floor=True

                    # is_collision = check_if_sphere_intersect(sphere, sphere_radius[joint], obs, self.env.radius)
                    # is_collision_floor = check_if_sphere_intersect(sphere[:3], sphere_radius[joint],
                    #                                                (obs[0], obs[1], 0), self.env.radius)
                    if is_collision or is_collision_floor:
                        return True

        return False

    # def local_planner(self, prev_conf, current_conf) -> bool:
    #     '''check for collisions between two configurations - return True if trasition is valid
    #     @param prev_conf - some configuration
    #     @param current_conf - current configuration
    #     '''
    #     # TODO
    #     if self.is_in_collision(prev_conf) or self.is_in_collision(current_conf):
    #         return False
    #
    #     num_intermediate_conf = 2 / self.resolution
    #     intermediate_conf = prev_conf
    #     # step_size = (current_conf - prev_conf) / (num_intermediate_conf + 1)
    #     for i in range(num_intermediate_conf):
    #         intermediate_conf = self.sample(current_conf)
    #         if self.is_in_collision(intermediate_conf):
    #             return False
    #         # intermediate_conf += step_size
    #
    #     return True
        
    
    def local_planner(self, prev_conf ,current_conf,visualizer) -> bool:
        '''check for collisions between two configurations - return True if trasition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration
        '''
        # TODO 
        # hint: use self.is_in_collision()
        # Generate intermediate configurations between prev_conf and current_conf
        num_intermediate_configs = max(int(np.ceil(np.linalg.norm(current_conf - prev_conf) / self.resolution)), 2)
        intermediate_configs = np.linspace(prev_conf, current_conf, num_intermediate_configs)

        # Check for collisions in intermediate configurations
        for conf in intermediate_configs:
            visualizer.show_conf(conf)
            if self.is_in_collision(conf):
                return False  # Collision detected, transition is invalid

        return True  # No collision detected, transition is valid
    
    def edge_cost(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1-conf2,2)) ** 0.5
    
    

    
    
    