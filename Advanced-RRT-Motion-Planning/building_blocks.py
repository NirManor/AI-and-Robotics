import numpy as np
import math

X = 0
Y = 1
Z = 2
MIN_RESOLUTION = 3

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
        #samples a value for the joint. If the random number is less than p_bias, 
        #it samples from the goal configuration for that joint. 
        #Otherwise, it samples randomly within the joint limits.
    
        if np.random.rand() < self.p_bias:
            return goal_conf
        else:# With probability 1 - p_bias, sample randomly within joint limits
            conf = []
            for joint, joint_limits in self.mechamical_limits.items():            
                lower_limit, upper_limit = joint_limits[0], joint_limits[1]
                conf[joint] = np.random.uniform(lower_limit, upper_limit)
        
        return np.array(conf)
           
    
    def is_in_collision(self, conf) -> bool:
        """
        check for collision in given configuration, arm-arm and arm-obstacle
        return True if in collision
        @param conf - some configuration 
        """
        # TODO         
        #self.transform.conf2sphere_coords - returns the coordinates of the spheres along the manipulator's links 
        #for a given configuration, in the base_link frame
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        global_sphere_coords_copy = global_sphere_coords.copy()
        keys_list = list(global_sphere_coords_copy.keys())
        del global_sphere_coords_copy[keys_list[0]]

        sphere_radius = self.ur_params.sphere_radius

        is_collision = False
        # arm - arm collision
        for joint_1, spheres_1 in global_sphere_coords.items():
            any_greater_than_0_4 = any(arr[0] > 0.4 for arr in spheres_1)
            if any_greater_than_0_4:
                return True
            if len(global_sphere_coords_copy) <= 1:
                break
            
            keys_list = list(global_sphere_coords_copy.keys())
            del global_sphere_coords_copy[keys_list[0]]
            for joint_2, spheres_2 in global_sphere_coords_copy.items():
                for sphere_1 in spheres_1:
                    for sphere_2 in spheres_2:
                        is_collision = check_if_sphere_intersect(sphere_1[:3], sphere_radius[joint_1], 
                                                                sphere_2[:3], sphere_radius[joint_2])
                        if is_collision:
                            return True

        # arm - obstacle collision 
        obstacles = self.env.obstacles
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                for obs in obstacles:
                    is_collision = check_if_sphere_intersect(sphere[:3], sphere_radius[joint], 
                                                            obs, self.env.radius)
                    if is_collision:
                        return True
                    
        # arm - floor collision
        for joint, spheres in global_sphere_coords.items():
            for sphere in spheres:
                if joint == 'shoulder_link':
                    continue
                if (sphere[2] - sphere_radius[joint]) <= 0:
                    return True

        return False
    
    def local_planner(self, prev_conf ,current_conf) -> bool:
        '''check for collisions between two configurations - return True if trasition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration
        '''
        # TODO 
        # Generate intermediate configurations between prev_conf and current_conf
        dist_prev_curr = np.linalg.norm(current_conf - prev_conf)
        num_intermediate_configs = max(int(np.ceil(dist_prev_curr / self.resolution)), MIN_RESOLUTION)
        intermediate_configs = np.linspace(prev_conf, current_conf, num_intermediate_configs)

        # Check for collisions in intermediate configurations
        for intermediate_conf in intermediate_configs:
            if self.is_in_collision(intermediate_conf):
                #print_resolution_numConfig(self.resolution, num_intermediate_configs, False)
                return False  # Collision detected, transition is invalid
        
        #print_resolution_numConfig(self.resolution, num_intermediate_configs, True)
        return True   # No collision detected, transition is valid
    
    def edge_cost(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1-conf2,2)) ** 0.5
    
    
#our function
def check_if_sphere_intersect(sphere_1, r_1, sphere_2, r_2) -> bool:
    """
    Calculates the distance between the centers of the spheres. 
    If this distance is less than or equal to the sum of the radii of the spheres, 
    then the spheres intersect and returns True"""
      
    distance = np.linalg.norm(sphere_1 - sphere_2)
    
    sum_of_radii = r_1 + r_2

    if distance <= sum_of_radii:
        return True
    else:
        return False

 #our function
"""def print_resolution_numConfig(resolution, num_intermediate_configs, is_collision):
    print("For self.resolution of {0} the number of intermediate "
        "configuration that were checked is: {1}, and the local "
        "planner returns {2}".format(resolution, num_intermediate_configs, is_collision))"""

#our function
def check_is_floor_collision(z_coord, radii, curr_joint):
    if curr_joint != 'shoulder_link':
         if (z_coord - radii) <= 0:
            return True
    return False
    