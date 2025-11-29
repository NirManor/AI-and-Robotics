import numpy as np
import json
import os
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from kinematics import UR5e_PARAMS, Transform
from matplotlib.widgets import RectangleSelector

class Path_Model(object):
    def __init__(self, json_file, load_modified=True, x_offset=0, z_offset=0.2):
        # Check if json file exists and load
        json_path = os.path.join(os.getcwd(), json_file)
        if not os.path.isfile(json_path):
            raise ValueError('Json file does not exist!')
        with open(json_path) as f:
            json_dict = json.load(f)

        self.ur_params = UR5e_PARAMS()
        self.transform = Transform(self.ur_params)

        if load_modified and 'WAY_POINTS_MODIFIED' in json_dict:
            self.waypoints_coords = np.array(json_dict['WAY_POINTS_MODIFIED'])
        else:
            self.waypoints_confs = np.array(json_dict['WAY_POINTS'])
            waypoints = []
            for conf in self.waypoints_confs:
                coords = self.transform.get_end_effector_position(conf)
                waypoints.append(coords)
            self.waypoints_coords = np.array(waypoints)

        # Apply translation to the waypoints
        self.waypoints_coords = self.translate(self.waypoints_coords, np.array([x_offset, 0, z_offset]))

        self.new_start = self.waypoints_coords[0]
        self.new_end = self.waypoints_coords[-1]

    def get_waypoints_coords(self):
        return self.waypoints_coords

    def fit_spline(self, waypoints, s=0):
        """Fit smoothing splines to provided 3D waypoints."""
        # Ensure all points are finite
        if not np.all(np.isfinite(waypoints)):
            raise ValueError("Waypoints contain non-finite values")

        # Ensure we have enough points
        if waypoints.shape[0] < 4:
            raise ValueError("Not enough points to fit a spline")

        try:
            tck, u = splprep([waypoints[:, 0], waypoints[:, 1], waypoints[:, 2]], s=s)
        except Exception as e:
            raise
        return tck

    def evaluate_spline(self, tck, num_points=100):
        """Evaluate splines to generate smooth path."""
        u_new = np.linspace(0, 1, num_points)
        smooth_path = splev(u_new, tck)
        return np.vstack(smooth_path).T

    def compute_tangents(self, tck, num_points=100):
        """Compute the tangents (derivatives) of the spline at regular intervals."""
        u_new = np.linspace(0, 1, num_points)
        derivatives = splev(u_new, tck, der=1)
        return np.vstack(derivatives).T

    def normalize_vector(self, vector):
        """Normalize the vector to unit length."""
        norm = np.linalg.norm(vector)
        if norm == 0:
            raise ValueError("Cannot normalize the zero vector")
        return vector / norm

    def calculate_normal_vector(self, tangent, arbitrary_option):
        """Calculate a vector normal to the provided tangent vector and pointing towards positive Y axis."""
        # Ensure the tangent is normalized
        normalized_tangent = self.normalize_vector(tangent)
        # Use the Z axis for the cross product to ensure the normal vector is in the XY plane
        z_axis_vector = self.normalize_vector(np.array(arbitrary_option))
        normal_vector = np.cross(normalized_tangent, z_axis_vector)
        # Ensure the normal vector is also normalized
        normal_vector = self.normalize_vector(normal_vector)
        # Adjust the normal vector to point towards positive Y axis if necessary
        if normal_vector[1] > 0:
            normal_vector = -normal_vector
        return normal_vector

    def rotate_about_z(self, points, theta):
        """Rotate points around the Z-axis by theta degrees."""
        theta = np.deg2rad(theta)
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        return np.dot(points, rotation_matrix.T)

    def translate(self, points, offset):
        """Translate points by a given offset."""
        return points + offset

    def adjust_path(self):
        """Return the waypoints without any adjustments for debugging."""
        adjusted_points = self.waypoints_coords  # No translation or rotation
        return adjusted_points

    def plot_original_waypoints_3d(self):
        """Plot the original waypoints in 3D."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 1], self.waypoints_coords[:, 2], color='b', s=10)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Original Waypoints')

        # Set equal scaling
        max_range = np.array([self.waypoints_coords[:, 0].max() - self.waypoints_coords[:, 0].min(),
                              self.waypoints_coords[:, 1].max() - self.waypoints_coords[:, 1].min(),
                              self.waypoints_coords[:, 2].max() - self.waypoints_coords[:, 2].min()]).max() / 2.0

        mid_x = (self.waypoints_coords[:, 0].max() + self.waypoints_coords[:, 0].min()) * 0.5
        mid_y = (self.waypoints_coords[:, 1].max() + self.waypoints_coords[:, 1].min()) * 0.5
        mid_z = (self.waypoints_coords[:, 2].max() + self.waypoints_coords[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        plt.show()

    def plot_2d_views(self):
        """Plot 2D views (XY, XZ, and YZ) for point removal with consistent axis scaling."""
        margin = 0.1  # 10% margin
        x_limits = (np.min(self.waypoints_coords[:, 0]), np.max(self.waypoints_coords[:, 0]))
        y_limits = (np.min(self.waypoints_coords[:, 1]), np.max(self.waypoints_coords[:, 1]))
        z_limits = (np.min(self.waypoints_coords[:, 2]), np.max(self.waypoints_coords[:, 2]))

        x_range = x_limits[1] - x_limits[0]
        y_range = y_limits[1] - y_limits[0]
        z_range = z_limits[1] - z_limits[0]

        x_limits = (x_limits[0] - margin * x_range, x_limits[1] + margin * x_range)
        y_limits = (y_limits[0] - margin * y_range, y_limits[1] + margin * y_range)
        z_limits = (z_limits[0] - margin * z_range, z_limits[1] + margin * z_range)

        self.fig_xy, self.ax_xy = plt.subplots()
        self.ax_xy.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 1], color='b', s=5)
        self.ax_xy.set_xlabel('X')
        self.ax_xy.set_ylabel('Y')
        self.ax_xy.set_title('XY View')
        self.ax_xy.set_xlim(x_limits)
        self.ax_xy.set_ylim(y_limits)
        self.rect_selector_xy = RectangleSelector(self.ax_xy, self.on_select_xy, drawtype='box', useblit=True, button=[1], minspanx=5, minspany=5, spancoords='pixels', interactive=True)

        self.fig_xz, self.ax_xz = plt.subplots()
        self.ax_xz.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 2], color='b', s=5)
        self.ax_xz.set_xlabel('X')
        self.ax_xz.set_ylabel('Z')
        self.ax_xz.set_title('XZ View')
        self.ax_xz.set_xlim(x_limits)
        self.ax_xz.set_ylim(z_limits)
        self.rect_selector_xz = RectangleSelector(self.ax_xz, self.on_select_xz, drawtype='box', useblit=True, button=[1], minspanx=5, minspany=5, spancoords='pixels', interactive=True)

        self.fig_yz, self.ax_yz = plt.subplots()
        self.ax_yz.scatter(self.waypoints_coords[:, 1], self.waypoints_coords[:, 2], color='b', s=5)
        self.ax_yz.set_xlabel('Y')
        self.ax_yz.set_ylabel('Z')
        self.ax_yz.set_title('YZ View')
        self.ax_yz.set_xlim(y_limits)
        self.ax_yz.set_ylim(z_limits)
        self.rect_selector_yz = RectangleSelector(self.ax_yz, self.on_select_yz, drawtype='box', useblit=True, button=[1], minspanx=5, minspany=5, spancoords='pixels', interactive=True)

        plt.show()

    def on_select_xy(self, eclick, erelease):
        """Callback for rectangle selection in XY view."""
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        selected_indices = np.where((self.waypoints_coords[:, 0] >= min(x1, x2)) & (self.waypoints_coords[:, 0] <= max(x1, x2)) &
                                    (self.waypoints_coords[:, 1] >= min(y1, y2)) & (self.waypoints_coords[:, 1] <= max(y1, y2)))[0]
        self.waypoints_coords = np.delete(self.waypoints_coords, selected_indices, axis=0)
        self.replot()

    def on_select_xz(self, eclick, erelease):
        """Callback for rectangle selection in XZ view."""
        x1, z1 = eclick.xdata, eclick.ydata
        x2, z2 = erelease.xdata, erelease.ydata
        selected_indices = np.where((self.waypoints_coords[:, 0] >= min(x1, x2)) & (self.waypoints_coords[:, 0] <= max(x1, x2)) &
                                    (self.waypoints_coords[:, 2] >= min(z1, z2)) & (self.waypoints_coords[:, 2] <= max(z1, z2)))[0]
        self.waypoints_coords = np.delete(self.waypoints_coords, selected_indices, axis=0)
        self.replot()

    def on_select_yz(self, eclick, erelease):
        """Callback for rectangle selection in YZ view."""
        y1, z1 = eclick.xdata, eclick.ydata
        y2, z2 = erelease.xdata, erelease.ydata
        selected_indices = np.where((self.waypoints_coords[:, 1] >= min(y1, y2)) & (self.waypoints_coords[:, 1] <= max(y1, y2)) &
                                    (self.waypoints_coords[:, 2] >= min(z1, z2)) & (self.waypoints_coords[:, 2] <= max(z1, z2)))[0]
        self.waypoints_coords = np.delete(self.waypoints_coords, selected_indices, axis=0)
        self.replot()

    def replot(self):
        self.ax_xy.cla()
        self.ax_xz.cla()
        self.ax_yz.cla()
        self.ax_xy.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 1], color='b', s=5)
        self.ax_xz.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 2], color='b', s=5)
        self.ax_yz.scatter(self.waypoints_coords[:, 1], self.waypoints_coords[:, 2], color='b', s=5)
        self.ax_xy.set_xlim((np.min(self.waypoints_coords[:, 0]), np.max(self.waypoints_coords[:, 0])))
        self.ax_xy.set_ylim((np.min(self.waypoints_coords[:, 1]), np.max(self.waypoints_coords[:, 1])))
        self.ax_xz.set_xlim((np.min(self.waypoints_coords[:, 0]), np.max(self.waypoints_coords[:, 0])))
        self.ax_xz.set_ylim((np.min(self.waypoints_coords[:, 2]), np.max(self.waypoints_coords[:, 2])))
        self.ax_yz.set_xlim((np.min(self.waypoints_coords[:, 1]), np.max(self.waypoints_coords[:, 1])))
        self.ax_yz.set_ylim((np.min(self.waypoints_coords[:, 2]), np.max(self.waypoints_coords[:, 2])))
        self.fig_xy.canvas.draw()
        self.fig_xz.canvas.draw()
        self.fig_yz.canvas.draw()

        adjusted_splines = self.fit_spline(self.waypoints_coords)
        adjusted_smooth_path = self.evaluate_spline(adjusted_splines)
        adjusted_tangents = self.compute_tangents(adjusted_splines)
        self.plot_3d_path_with_tangents(adjusted_smooth_path, adjusted_tangents, scale=0.05)

    def plot_3d_path_with_tangents(self, path, tangents, scale=0.01):
        """Plot the path with tangents in a 3D plot."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(path[:, 0], path[:, 1], path[:, 2], label='Adjusted Path')

        for i in range(len(path)):
            x, y, z = path[i]
            dx, dy, dz = tangents[i] * scale * 2
            ax.quiver(x, y, z, dx, dy, dz, color='r', length=scale, normalize=False, label='Tangent' if i == 0 else "")

        # Set equal scaling
        max_range = np.array([path[:, 0].max() - path[:, 0].min(),
                              path[:, 1].max() - path[:, 1].min(),
                              path[:, 2].max() - path[:, 2].min()]).max() / 2.0

        mid_x = (path[:, 0].max() + path[:, 0].min()) * 0.5
        mid_y = (path[:, 1].max() + path[:, 1].min()) * 0.5
        mid_z = (path[:, 2].max() + path[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()

    def plot_3d_path_with_multiple_normals(self, path, tangents, scale=0.01):
        """Plot the path with tangents and multiple normal vectors in a 3D plot."""
        self.fig_3d = plt.figure()
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        self.ax_3d.plot(path[:, 0], path[:, 1], path[:, 2], label='Adjusted Path')

        # Define the arbitrary vectors
        arbitrary_options = [
            np.array([0, 0, 1]),
            np.array([0, 1, 1]),
            np.array([0, 1, 2]),
            np.array([0, 2, 1]),
            np.array([0, -1, -1]),
            np.array([0, -1, -2]),
            np.array([0, -2, -1]),
            np.array([0, -1, 0]),
            np.array([0, 1, 0])
        ]

        colors = ['g', 'c', 'm', 'y', 'b', 'orange', 'purple', 'brown', 'pink']  # Different colors for different normal vectors
        labels = ['Normal (0,0,1)', 'Normal (0,1,1)', 'Normal (0,1,2)', 'Normal (0,2,1)',
                  'Normal (0,-1,-1)', 'Normal (0,-1,-2)', 'Normal (0,-2,-1)', 'Normal (0,-1,0)', 'Normal (0,1,0)']

        for i in range(len(path)):
            x, y, z = path[i]
            dx, dy, dz = tangents[i] * scale
            self.ax_3d.quiver(x, y, z, dx, dy, dz, color='r', length=scale, normalize=False)

            for arb_opt, color, label in zip(arbitrary_options, colors, labels):
                # Calculate normal vector and plot it
                normal_vector = self.calculate_normal_vector(tangents[i], arb_opt)
                nx, ny, nz = normal_vector * scale * 10
                self.ax_3d.quiver(x, y, z, nx, ny, nz, color=color, length=scale, normalize=False, label=label if i == 0 else "")

        # Set equal scaling
        max_range = np.array([path[:, 0].max() - path[:, 0].min(),
                              path[:, 1].max() - path[:, 1].min(),
                              path[:, 2].max() - path[:, 2].min()]).max() / 2.0

        mid_x = (path[:, 0].max() + path[:, 0].min()) * 0.5
        mid_y = (path[:, 1].max() + path[:, 1].min()) * 0.5
        mid_z = (path[:, 2].max() + path[:, 2].min()) * 0.5

        self.ax_3d.set_xlim(mid_x - max_range, mid_x + max_range)
        self.ax_3d.set_ylim(mid_y - max_range, mid_y + max_range)
        self.ax_3d.set_zlim(mid_z - max_range, mid_z + max_range)

        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.legend()
        plt.show()

    def process_path(self, plot=False, plot_multiple_normals=False):
        """Process the path adjustments and optionally plot them. Return adjusted path and tangents."""
        adjusted_waypoints = self.adjust_path()
        adjusted_tck = self.fit_spline(adjusted_waypoints)
        adjusted_smooth_path = self.evaluate_spline(adjusted_tck)
        adjusted_tangents = self.compute_tangents(adjusted_tck)

        if plot:
            self.plot_3d_path_with_tangents(adjusted_smooth_path, adjusted_tangents, scale=0.05)

        if plot_multiple_normals:
            self.plot_3d_path_with_multiple_normals(adjusted_smooth_path, adjusted_tangents, scale=0.05)

        return adjusted_tck, adjusted_smooth_path, adjusted_tangents

    def save_modified_waypoints(self, output_file):
        """Save the modified waypoints to a JSON file."""
        with open(output_file, 'w') as f:
            json.dump({'WAY_POINTS_MODIFIED': self.waypoints_coords.tolist()}, f, indent=4)

# Example usage
if __name__ == "__main__":
    path_model = Path_Model('3D_curve_waypoint.json', load_modified=True, x_offset=-0.04, z_offset=0.078)
    path_model.plot_original_waypoints_3d()
    path_model.process_path(plot=True, plot_multiple_normals=True)
    path_model.plot_2d_views()

    # Save modified waypoints to a new JSON file
    path_model.save_modified_waypoints('3D_curve_waypoint.json')
