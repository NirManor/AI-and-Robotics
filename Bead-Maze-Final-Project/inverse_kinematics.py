import numpy as np
from scipy.spatial.transform import Rotation as R
from math import pi, cos, sin, atan2, acos, sqrt, asin

class Inverse_Kinematics(object):
    def __init__(self, tx, ty, tz, tangent, normal_option, ur_params, env, bb):
        self.tool_length = 0.135 + 0.075  # meters, adjust if different + adding 7.5 cm for the gripper
        self.DH_matrix_UR5e = np.matrix([
            [0, pi / 2.0, 0.1625],
            [-0.425, 0, 0],
            [-0.3922, 0, 0],
            [0, pi / 2.0, 0.1333],
            [0, -pi / 2.0, 0.0997],
            [0, 0, 0.0996 + self.tool_length]
        ])
        self.env_idx = 2
        self.ur_params = ur_params
        self.env = env
        self.bb = bb
        self.update_target_and_tangent(tx, ty, tz, tangent, normal_option)

    def update_target_and_tangent(self, tx, ty, tz, tangent, normal_option):
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.tangent = tangent
        # Compute the normal vector from the tangent
        normal_vector = self.calculate_normal_vector(tangent, normal_option)
        # Calculate the rotation matrix based on the tangent, normal, and binormal vectors
        self.R_matrix = self.calculate_rotation_matrix(tangent, normal_vector)

    def normalize_vector(self, vector):
        """Normalize the vector to unit length."""
        norm = np.linalg.norm(vector)
        if norm == 0:
            raise ValueError("Cannot normalize the zero vector")
        return vector / norm

    def calculate_normal_vector(self, tangent, normal_option):
        """Calculate a vector normal to the provided tangent vector."""
        # Ensure the tangent is normalized
        normalized_tangent = self.normalize_vector(tangent)
        arbitrary_vector = self.normalize_vector(normal_option)
        normal_vector = np.cross(normalized_tangent, arbitrary_vector)
        if normal_vector[1] > 0:
            normal_vector = -normal_vector
        return self.normalize_vector(normal_vector)

    def calculate_rotation_matrix(self, tangent, normal_vector):
        """Calculate the rotation matrix based on the tangent and normal vectors."""
        tangent = self.normalize_vector(tangent)
        normal_vector = self.normalize_vector(normal_vector)
        binormal = np.cross(tangent, normal_vector)

        # Adjust the rotation matrix to ensure the Z-axis points in the negative Y and negative Z directions
        # if binormal[2] > 0:
        #     binormal = -binormal
        rotation_matrix = np.column_stack((tangent, binormal, -normal_vector))
        return rotation_matrix

    def compute_transformation_matrix(self, roll_angle):
        """Compute the transformation matrix based on position and orientation."""
        # Rotation matrix for the roll around the binormal (tangent) vector
        R_roll = R.from_euler('z', roll_angle, degrees=False).as_matrix()
        R_final = self.R_matrix @ R_roll  # Combine with the rotation matrix
        T = np.matrix([[self.tx], [self.ty], [self.tz]])
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R_final
        transformation_matrix[:3, 3] = T.flatten()
        return transformation_matrix

    def mat_transform_DH(self, DH_matrix, n, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
        n = n - 1
        t_z_theta = np.matrix([[cos(edges[n]), -sin(edges[n]), 0, 0],
                               [sin(edges[n]), cos(edges[n]), 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], copy=False)
        t_zd = np.matrix(np.identity(4), copy=False)
        t_zd[2, 3] = DH_matrix[n, 2]
        t_xa = np.matrix(np.identity(4), copy=False)
        t_xa[0, 3] = DH_matrix[n, 0]
        t_x_alpha = np.matrix([[1, 0, 0, 0],
                               [0, cos(DH_matrix[n, 1]), -sin(DH_matrix[n, 1]), 0],
                               [0, sin(DH_matrix[n, 1]), cos(DH_matrix[n, 1]), 0],
                               [0, 0, 0, 1]], copy=False)
        transform = t_z_theta * t_zd * t_xa * t_x_alpha
        return transform

    def forward_kinematic_solution(self, DH_matrix, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
        t01 = self.mat_transform_DH(DH_matrix, 1, edges)
        t12 = self.mat_transform_DH(DH_matrix, 2, edges)
        t23 = self.mat_transform_DH(DH_matrix, 3, edges)
        t34 = self.mat_transform_DH(DH_matrix, 4, edges)
        t45 = self.mat_transform_DH(DH_matrix, 5, edges)
        t56 = self.mat_transform_DH(DH_matrix, 6, edges)
        answer = t01 * t12 * t23 * t34 * t45 * t56
        return answer

    def inverse_kinematic_solution(self, DH_matrix, transform_matrix):
        theta = np.matrix(np.zeros((6, 8)))
        # theta 1
        T06 = transform_matrix

        P05 = T06 * np.matrix([[0], [0], [-DH_matrix[5, 2]], [1]])
        psi = atan2(P05[1], P05[0])
        phi_cos = (DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2]) / sqrt(P05[0] ** 2 + P05[1] ** 2)
        phi = acos(np.clip(phi_cos, -1.0, 1.0))  # Use np.clip to avoid domain error
        theta[0, 0:4] = psi + phi + pi / 2
        theta[0, 4:8] = psi - phi + pi / 2

        # theta 5
        for i in {0, 4}:
            th5cos = (T06[0, 3] * sin(theta[0, i]) - T06[1, 3] * cos(theta[0, i]) - (
                        DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2])) / DH_matrix[5, 2]
            if 1 >= th5cos >= -1:
                th5 = acos(th5cos)
            else:
                th5 = 0
            theta[4, i:i + 2] = th5
            theta[4, i + 2:i + 4] = -th5

        # theta 6
        for i in {0, 2, 4, 6}:
            T60 = np.linalg.inv(T06)
            th = atan2((-T60[1, 0] * sin(theta[0, i]) + T60[1, 1] * cos(theta[0, i])),
                       (T60[0, 0] * sin(theta[0, i]) - T60[0, 1] * cos(theta[0, i])))
            theta[5, i:i + 2] =(th)

        # theta 3
        for i in {0, 2, 4, 6}:
            T01 = self.mat_transform_DH(DH_matrix, 1, theta[:, i])
            T45 = self.mat_transform_DH(DH_matrix, 5, theta[:, i])
            T56 = self.mat_transform_DH(DH_matrix, 6, theta[:, i])
            T14 = np.linalg.inv(T01) * T06 * np.linalg.inv(T45 * T56)
            P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])
            costh3 = ((P13[0] ** 2 + P13[1] ** 2 - DH_matrix[1, 0] ** 2 - DH_matrix[2, 0] ** 2) /
                      (2 * DH_matrix[1, 0] * DH_matrix[2, 0]))
            if 1 >= costh3 >= -1:
                th3 = acos(costh3)
            else:
                th3 = 0
            theta[2, i] = th3
            theta[2, i + 1] = -th3

        # theta 2, 4
        for i in range(8):
            T01 = self.mat_transform_DH(DH_matrix, 1, theta[:, i])
            T45 = self.mat_transform_DH(DH_matrix, 5, theta[:, i])
            T56 = self.mat_transform_DH(DH_matrix, 6, theta[:, i])
            T14 = np.linalg.inv(T01) * T06 * np.linalg.inv(T45 * T56)
            P13 = T14 * np.matrix([[0], [-DH_matrix[3, 2]], [0], [1]])

            theta[1, i] = atan2(-P13[1], -P13[0]) - asin(
                -DH_matrix[2, 0] * sin(theta[2, i]) / sqrt(P13[0] ** 2 + P13[1] ** 2)
            )
            T32 = np.linalg.inv(self.mat_transform_DH(DH_matrix, 3, theta[:, i]))
            T21 = np.linalg.inv(self.mat_transform_DH(DH_matrix, 2, theta[:, i]))
            T34 = T32 * T21 * T14
            theta[3, i] = atan2(T34[1, 0], T34[0, 0])
        return theta

    def find_possible_configs(self):
        tx = self.tx
        ty = self.ty
        tz = self.tz

        candidate_sols = []
        for roll_angle in np.linspace(0, 2 * pi, 3):  # Try different roll angles for the last joint
            transform = self.compute_transformation_matrix(roll_angle)
            IKS = self.inverse_kinematic_solution(self.DH_matrix_UR5e, transform)
            for i in range(IKS.shape[1]):
                candidate_sols.append(IKS[:, i])
        candidate_sols = np.array(candidate_sols)

        # Check for collisions and angle limits
        sols = []
        for candidate_sol in candidate_sols:
            candidate_sol_array = candidate_sol.flatten()
            if self.bb.is_in_collision(candidate_sol_array):
                continue
            for idx, angle in enumerate(candidate_sol):
                if 2 * np.pi > angle > np.pi:
                    candidate_sol[idx] = -(2 * np.pi - angle)
                if -2 * np.pi < angle < -np.pi:
                    candidate_sol[idx] = -(2 * np.pi + angle)
            if np.max(candidate_sol) > np.pi or np.min(candidate_sol) < -np.pi:
                continue
            sols.append(candidate_sol)

        # Verify solution:
        final_sol = []
        for sol in sols:
            transform = self.forward_kinematic_solution(self.DH_matrix_UR5e, sol)
            diff = np.linalg.norm(np.array([transform[0, 3], transform[1, 3], transform[2, 3]]) - np.array([tx, ty, tz]))
            if diff < 0.05:
                final_sol.append(sol)
        final_sol = np.array(final_sol)
        return final_sol
