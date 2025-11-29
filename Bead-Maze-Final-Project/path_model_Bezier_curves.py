import numpy as np
import json
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.special import comb
from kinematics import UR5e_PARAMS, Transform


class Path_Model(object):
    def __init__(self, json_file, first_y_order=5, last_y_order=5, load_modified=False):
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

            # Sort waypoints
            self.first_y_order = first_y_order
            self.last_y_order = last_y_order
            self.sort_waypoints()

        self.new_start = self.waypoints_coords[0]
        self.new_end = self.waypoints_coords[-1]

    def sort_waypoints(self):
        """Sort waypoints: first few by increasing Z, last few by decreasing Z, and middle by increasing X."""
        if self.first_y_order > 0:
            first_part = self.waypoints_coords[:self.first_y_order]
            first_part = first_part[np.argsort(first_part[:, 2])]
        else:
            first_part = np.empty((0, 3))

        if self.last_y_order > 0:
            last_part = self.waypoints_coords[-self.last_y_order:]
            last_part = last_part[np.argsort(last_part[:, 2])[::-1]]
        else:
            last_part = np.empty((0, 3))

        middle_part = self.waypoints_coords[self.first_y_order:-self.last_y_order]
        middle_part = middle_part[np.argsort(middle_part[:, 0])]

        self.waypoints_coords = np.vstack((first_part, middle_part, last_part))

    def get_waypoints_coords(self):
        return self.waypoints_coords

    def bezier_curve(self, points, num_points=100):
        """Evaluate a cubic Bezier curve."""

        def bernstein_poly(i, n, t):
            return comb(n, i) * (t ** i) * (1 - t) ** (n - i)

        n = len(points) - 1
        t = np.linspace(0.0, 1.0, num_points)
        curve = np.zeros((num_points, points.shape[1]))

        for i in range(n + 1):
            curve += np.outer(bernstein_poly(i, n, t), points[i])
        return curve

    def fit_bezier(self, waypoints):
        """Fit cubic Bezier curves to provided 3D waypoints."""
        bezier_curves = []
        num_points = len(waypoints)

        for i in range(0, num_points - 3, 3):
            control_points = waypoints[i:i + 4]
            bezier_curve = self.bezier_curve(control_points, num_points=100)
            bezier_curves.append(bezier_curve)

        return np.vstack(bezier_curves)

    def adjust_path(self):
        """Adjust the path to align with new start and new end points."""
        translated_points = self.translate(self.waypoints_coords, self.new_start - self.waypoints_coords[0])
        orig_vector = self.waypoints_coords[-1] - self.waypoints_coords[0]
        new_vector = self.new_end - self.new_start
        orig_angle = np.arctan2(orig_vector[1], orig_vector[0])
        new_angle = np.arctan2(new_vector[1], new_vector[0])
        rotation_angle = new_angle - orig_angle
        return self.rotate_about_z(translated_points, np.rad2deg(rotation_angle))

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

    def plot_original_waypoints_3d(self):
        """Plot the original waypoints in 3D."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 1], self.waypoints_coords[:, 2], color='b',
                   s=10)
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
        """Plot 2D views (XY and XZ) for point removal."""
        self.fig_xy, self.ax_xy = plt.subplots()
        self.ax_xy.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 1], color='b', s=5)
        self.ax_xy.set_xlabel('X')
        self.ax_xy.set_ylabel('Y')
        self.ax_xy.set_title('XY View')
        self.equalize_axes(self.ax_xy, self.waypoints_coords[:, 0], self.waypoints_coords[:, 1])
        self.rect_selector_xy = RectangleSelector(self.ax_xy, self.on_select_xy, drawtype='box', useblit=True,
                                                  button=[1], minspanx=5, minspany=5, spancoords='pixels',
                                                  interactive=True)

        self.fig_xz, self.ax_xz = plt.subplots()
        self.ax_xz.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 2], color='b', s=5)
        self.ax_xz.set_xlabel('X')
        self.ax_xz.set_ylabel('Z')
        self.ax_xz.set_title('XZ View')
        self.equalize_axes(self.ax_xz, self.waypoints_coords[:, 0], self.waypoints_coords[:, 2])
        self.rect_selector_xz = RectangleSelector(self.ax_xz, self.on_select_xz, drawtype='box', useblit=True,
                                                  button=[1], minspanx=5, minspany=5, spancoords='pixels',
                                                  interactive=True)

        plt.show()

    def equalize_axes(self, ax, x_data, y_data):
        """Set equal scaling for the axes."""
        max_range = np.array([x_data.max() - x_data.min(), y_data.max() - y_data.min()]).max() / 2.0

        mid_x = (x_data.max() + x_data.min()) * 0.5
        mid_y = (y_data.max() + y_data.min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)

    def on_select_xy(self, eclick, erelease):
        """Callback for rectangle selection in XY view."""
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        selected_indices = \
        np.where((self.waypoints_coords[:, 0] >= min(x1, x2)) & (self.waypoints_coords[:, 0] <= max(x1, x2)) &
                 (self.waypoints_coords[:, 1] >= min(y1, y2)) & (self.waypoints_coords[:, 1] <= max(y1, y2)))[0]
        self.waypoints_coords = np.delete(self.waypoints_coords, selected_indices, axis=0)
        self.replot()

    def on_select_xz(self, eclick, erelease):
        """Callback for rectangle selection in XZ view."""
        x1, z1 = eclick.xdata, eclick.ydata
        x2, z2 = erelease.xdata, erelease.ydata
        selected_indices = \
        np.where((self.waypoints_coords[:, 0] >= min(x1, x2)) & (self.waypoints_coords[:, 0] <= max(x1, x2)) &
                 (self.waypoints_coords[:, 2] >= min(z1, z2)) & (self.waypoints_coords[:, 2] <= max(z1, z2)))[0]
        self.waypoints_coords = np.delete(self.waypoints_coords, selected_indices, axis=0)
        self.replot()

    def replot(self):
        self.ax_xy.cla()
        self.ax_xz.cla()
        self.ax_xy.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 1], color='b', s=5)
        self.ax_xz.scatter(self.waypoints_coords[:, 0], self.waypoints_coords[:, 2], color='b', s=5)
        self.equalize_axes(self.ax_xy, self.waypoints_coords[:, 0], self.waypoints_coords[:, 1])
        self.equalize_axes(self.ax_xz, self.waypoints_coords[:, 0], self.waypoints_coords[:, 2])
        self.fig_xy.canvas.draw()
        self.fig_xz.canvas.draw()

        bezier_path = self.fit_bezier(self.waypoints_coords)
        tangents = self.compute_tangents(bezier_path)
        self.plot_3d_path_with_tangents(bezier_path, tangents, scale=0.01)

    def compute_tangents(self, path, num_points=100):
        """Compute the tangents of the path at regular intervals."""
        tangents = np.zeros_like(path)
        tangents[1:-1] = (path[2:] - path[:-2]) / 2.0
        tangents[0] = path[1] - path[0]
        tangents[-1] = path[-1] - path[-2]
        return tangents

    def plot_3d_path_with_tangents(self, path, tangents, scale=0.01):
        """Plot the path with tangents in a 3D plot."""
        self.fig_3d = plt.figure()
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        self.ax_3d.plot(path[:, 0], path[:, 1], path[:, 2], label='Adjusted Path')

        for i in range(len(path)):
            x, y, z = path[i]
            dx, dy, dz = tangents[i] * scale
            self.ax_3d.quiver(x, y, z, dx, dy, dz, color='r', length=scale, normalize=False)

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

    def create_sine_wave_waypoints(self, num_points=100, amplitude=5, frequency=1, length=10):
        """Generate waypoints along a sine wave in 3D."""
        x = np.linspace(0, length, num_points)
        y = amplitude * np.sin(frequency * x)
        z = np.zeros_like(x)  # Keep z constant for simplicity
        return np.column_stack((x, y, z))

    def process_path(self, plot=True):
        """Process the path adjustments and optionally plot them. Return adjusted path and tangents."""
        adjusted_waypoints = self.adjust_path()
        bezier_path = self.fit_bezier(adjusted_waypoints)
        tangents = self.compute_tangents(bezier_path)

        if plot:
            self.plot_3d_path_with_tangents(bezier_path, tangents, scale=0.01)

        return bezier_path, tangents

    def save_modified_waypoints(self, output_file):
        """Save the modified waypoints to a JSON file."""
        with open(output_file, 'w') as f:
            json.dump({'WAY_POINTS_MODIFIED': self.waypoints_coords.tolist()}, f, indent=4)


# Additional imports for RectangleSelector
from matplotlib.widgets import RectangleSelector

# Example usage
if __name__ == "__main__":
    path_model = Path_Model('modified_path_data.json', first_y_order=13, last_y_order=10, load_modified=True)
    path_model.plot_original_waypoints_3d()
    path_model.process_path(plot=True)
    path_model.plot_2d_views()

    # Save modified waypoints to a new JSON file
    path_model.save_modified_waypoints('modified_path_data.json')
