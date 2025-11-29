import numpy as np
import argparse
import os
from inverse_kinematics import Inverse_Kinematics
from building_blocks import Building_Blocks
from kinematics import UR5e_PARAMS, Transform
from environment import Environment
from visualizer_gif import Visualize_Gif

from layered_graph import Layered_Graph
from path_model import Path_Model
from dijkstra import Dijkstra
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_axes(ax, origin, rotation_matrix, length=0.1):
    """Plot the XYZ axes at the given origin with the given rotation matrix."""
    x_axis = rotation_matrix[:, 0] * length
    y_axis = rotation_matrix[:, 1] * length
    z_axis = rotation_matrix[:, 2] * length

    ax.quiver(*origin, *x_axis, color='r', label='X-axis')
    ax.quiver(*origin, *y_axis, color='g', label='Y-axis')
    ax.quiver(*origin, *z_axis, color='b', label='Z-axis')

def visualize_end_effector(tx, ty, tz, R_matrix, smooth_path):
    """Visualize the end effector position and orientation along with the smooth path."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the smooth path
    smooth_path_points = np.array(smooth_path)
    ax.plot(smooth_path_points[:, 0], smooth_path_points[:, 1], smooth_path_points[:, 2], 'k-', label='Path')

    # Plot the end effector orientation
    ax.set_xlim([tx - 0.2, tx + 0.2])
    ax.set_ylim([ty - 0.2, ty + 0.2])
    ax.set_zlim([tz - 0.2, tz + 0.2])
    plot_axes(ax, [tx, ty, tz], R_matrix)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script for bead maze')
    parser.add_argument('-waypoints', '--waypoints', type=str, default='3D_curve_waypoint.json',
                        help='Json file name containing all way points')
    parser.add_argument('-threshold', '--threshold', type=int, default=10,
                        help='Threshold for the number of solutions in a layer to stop further calculations')
    args = parser.parse_args()

    path_model = Path_Model(json_file=args.waypoints)
    splines, smooth_path, tangents = path_model.process_path()
    waypoints_coords = path_model.get_waypoints_coords()

    ur_params = UR5e_PARAMS()
    transform = Transform(ur_params)
    env = Environment(2, smooth_path)
    bb = Building_Blocks(transform, ur_params, env, None)

    # Define arbitrary options
    normal_options = [
        np.array([0, 0, 1]) #,
        # np.array([0, 1, 1]),
        # np.array([0, 1, 2]),
        # np.array([0, 2, 1])
        # np.array([0, -1, -1]),
        # np.array([0, -1, -2]),
        # np.array([0, -2, -1]),
        # np.array([0, -1, 0]),
        # np.array([0, 1, 0])
    ]
    threshold = 5
    ik_solutions_per_layer = []
    # Process each waypoint and its corresponding tangent
    i = 0
    j = 0
    for waypoint, tangent in zip(smooth_path, tangents):
        print("potential layer number: ", i)

        tx, ty, tz = waypoint
        layer_solutions = []
        for normal_opt in normal_options:
            # Update IK solver target and tangent        
            ik_solver = Inverse_Kinematics(tx, ty, tz, tangent, normal_opt, ur_params, env, bb)

            # Compute IK solutions for the current waypoint
            possible_configs = ik_solver.find_possible_configs()
            possible_configs_flatten = np.array([np.squeeze(sub_array) for sub_array in possible_configs])
            layer_solutions.extend(possible_configs_flatten)
            if len(layer_solutions) > threshold:
                print(f"Threshold of {threshold} solutions reached. Moving to next layer.")
                break
        if len(layer_solutions) != 0:
            ik_solutions_per_layer.append(layer_solutions)#TODO - layer_solutions can be empty?
            j += 1
            if i > 5:
                visualize_end_effector(tx, ty, tz, ik_solver.R_matrix, smooth_path)
                j = 0
        else:
            print(f"\nfailed with IK in layer: {i} \n")

        i += 1
    bb.inverse_kinematics = ik_solver
    
    # Building the graph using valid configurations for each waypoint
    graph = Layered_Graph(ur_params, env, bb, splines)
    graph.build_graph(ik_solutions_per_layer)

    first_layer = 0
    last_layer = len(graph.layers)
    nodes_in_first_layer = graph.get_nodes_by_layer(first_layer)    
    nodes_in_last_layer = graph.get_nodes_by_layer(last_layer - 1)
    min_bottle_neck = np.inf
    shortest_path = None
    
    for i in range(len(nodes_in_first_layer)):
        dijkstra = Dijkstra(graph, (first_layer, i), last_layer - 1)
        curr_bottleneck, path = dijkstra.find_shortest_path()
        if curr_bottleneck != None:
            if curr_bottleneck < min_bottle_neck:
                min_bottle_neck = curr_bottleneck
                shortest_path = path
                print("shortest path:", shortest_path)
            else:
                print("Did not found a new shortest path")
            
    # Create the directory if it doesn't exist
    directory = "final_path"
    if not os.path.exists(directory):
        os.makedirs(directory)

    file_path = os.path.join(directory, 'path')
    np.save(file_path, np.array(shortest_path))

    visualizer_gif = Visualize_Gif(ur_params, env, transform, bb, path_model.waypoints_coords)#TODO - need to check if it works
    file_path_vg = os.path.join(directory, 'visualize_path')

    try:
        path = np.load(file_path + '.npy')
        visualizer_gif.save_path_to_gif(path, file_path_vg)
    except Exception as X:
        print(X)
    
    
    

