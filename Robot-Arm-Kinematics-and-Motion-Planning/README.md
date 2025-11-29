# Robot Arm Kinematics & Motion Planning Basics

Implementation of forward/inverse kinematics for the UR5e robotic arm and introduction to sampling-based motion planning using RRT-Star algorithm.

## Problem Description

**Objective:** Plan collision-free motion for a 6-DOF robotic arm from a start configuration to a goal configuration while avoiding obstacles in the workspace.

**Robot:** UR5e Collaborative Manipulator
- 6 rotational joints with ±π range
- Industrial-grade precision and reliability
- Real-time control capabilities

**Workspace:** 3D environment with obstacles
- Static obstacle avoidance
- Sphere-based collision detection
- Complex geometric constraints

**Challenges:**
- High-dimensional configuration space (6D)
- Non-linear kinematics equations
- Collision detection efficiency
- Path feasibility verification

---

## Key Concepts

### Forward Kinematics

**Purpose:** Compute end-effector position and orientation from joint angles

**Method:** Denavit-Hartenberg (D-H) Parameters
- Standard robotics convention for link transformations
- Parameters describe relationship between consecutive joints
- Homogeneous transformation matrices

**UR5e D-H Parameters:**
```
Joint    Alpha    A        D        Theta
1        0        0        0.1625   0
2        π/2      0        0        0
3        0        -0.425   0        0
4        0        -0.3922  0.133    0
5        π/2      0        0.0997   0
6        -π/2     0        0.0996   0
```

**Forward Kinematics Equation:**
```
T = T1 * T2 * T3 * T4 * T5 * T6
```

Where Ti is the homogeneous transformation from frame i-1 to frame i.

**Complexity:** O(1) - direct computation from joint angles

### Inverse Kinematics

**Purpose:** Find joint angles for target end-effector position

**Challenge:** Non-linear, multiple solutions possible

**Methods:**
- Numerical iterative approaches
- Jacobian-based methods
- Analytical solutions (when available)

**Properties:**
- May have 0, 1, or multiple solutions
- Singularities where solutions don't exist
- Configuration-dependent feasibility

### Configuration Space (C-Space)

**Definition:** Space of all possible robot configurations (joint angles)

**UR5e C-Space:** 6-dimensional
- Each dimension represents one joint angle
- Range: [-π, π] for each joint
- Feasible region: avoiding self-collisions and workspace bounds

**Obstacles in C-Space:**
- Workspace obstacles map to C-space obstacles
- Shape and location depend on obstacle geometry
- Used for collision detection during planning

### Collision Detection

**Sphere-Based Approximation:**
- Each robot link approximated by multiple spheres
- Spheres have center positions and radii
- Efficient collision checking with workspace obstacles

**Sphere Parameters:**
```
Link Sphere Radius (inflated by safety factor)
Shoulder: 0.06m
Upper Arm: 0.05m
Forearm: 0.05m
Wrist 1: 0.04m
Wrist 2: 0.04m
Wrist 3: 0.04m
```

**Collision Check:**
- For each link sphere, check distance to obstacles
- Overhead obstacle (walls, boxes)
- Early termination if collision detected

**Complexity:** O(n_spheres × n_obstacles)

---

## Algorithms Implemented

### Local Motion Planner

**Purpose:** Validate straight-line motion between two configurations

**Algorithm:**
```
Input: start_config, goal_config, resolution
Output: collision_free (boolean)

num_steps = ceil(distance(start, goal) / resolution)
for i in 0 to num_steps:
    intermediate = interpolate(start, goal, i/num_steps)
    if collision_check(intermediate) == True:
        return False
return True
```

**Key Parameters:**
- **Resolution:** Step size for path discretization
  - Smaller = more accurate but slower
  - Larger = faster but might miss collisions
  - Typical: 0.05-0.1 radians

**Properties:**
- Deterministic: same result for same input
- Not complete: might miss narrow passages
- Efficient: single straight-line check

### RRT-Star Algorithm

**Purpose:** Find collision-free, optimized paths in configuration space

**Algorithm Overview:**
```
Initialize: Tree ← {start_config}

while iterations < max_iterations:
    1. Sample random configuration
       x_random ← random_sample()

    2. Find nearest configuration in tree
       x_nearest ← nearest_neighbor(Tree, x_random)

    3. Extend toward sample
       x_new ← extend(x_nearest, x_random, step_size)

    4. Check collision
       if collision_free(x_nearest, x_new):

    5. Find k-nearest neighbors
       neighbors ← k_nearest(Tree, x_new, k)

    6. Find best parent
       best_parent ← minimum cost neighbor

    7. Connect with best parent
       if collision_free(best_parent, x_new):
           connect(best_parent, x_new)

    8. Rewire neighbors through new node
       for neighbor in neighbors:
           if cost(x_new→neighbor) < cost(current_parent→neighbor):
               if collision_free(x_new, neighbor):
                   rewire(x_new, neighbor)

    9. Check goal
       if near_goal(x_new):
           return extract_path()
```

**Key Components:**

**Sampling Strategy:**
```python
if random() < goal_bias:
    return goal_config
else:
    return random_configuration()
```

**Extension Method:**
```python
direction = normalize(x_random - x_nearest)
x_new = x_nearest + step_size * direction
```

**k-Nearest Strategy:**
```python
k = constant  # e.g., k=5 for fixed
# or
k = log(num_vertices)  # For asymptotic optimality
```

**Rewiring Condition:**
```python
cost_via_new = cost(start→x_new) + cost(x_new→neighbor)
cost_via_current = cost(start→current_parent→neighbor)
if cost_via_new < cost_via_current:
    rewire neighbor to x_new
```

**Complexity:**
- Per iteration: O(log n) with spatial indexing
- Total: O(n log n) for n iterations
- Space: O(n) for tree storage

---

## Implementation Details

### Files Overview

**kinematics.py**
- `UR5e_PARAMS`: Robot parameters (D-H, geometry, collision spheres)
- `Transform`: Forward kinematics computation
- Methods:
  - `forward_kinematics(joint_angles)`: Compute end-effector pose
  - `get_link_positions(joint_angles)`: Get all link sphere positions

**building_blocks.py**
- `Building_Blocks`: Core planning functionality
- Methods:
  - `sample()`: Generate random/goal-biased sample
  - `local_planner()`: Validate straight-line motion
  - `collision_check()`: Sphere-obstacle collision detection

**planners.py**
- `RRT_STAR`: RRT-Star implementation
- Methods:
  - `find_path()`: Main planning algorithm
  - `extend()`: Tree extension method
  - `rewire()`: Path optimization via rewiring
  - `get_k_num()`: Dynamic k-nearest calculation
  - `get_shortest_path()`: Extract final solution path

**environment.py**
- `Environment`: Workspace obstacle definition
- Creates obstacle sets for collision detection
- Methods:
  - `wall_x_const()`, `wall_y_const()`, `wall_z_const()`: Wall creation
  - `box()`: Box obstacle definition
  - `sphere_num()`: Calculate sphere density for obstacles

**visualizer.py**
- `Visualize_UR`: 3D visualization
- Methods:
  - `show_conf()`: Display robot configuration
  - `visualize_path()`: Plot planned path
  - `show_tree()`: Display planning tree

**RRTTree.py**
- Tree data structure for storing nodes
- Methods:
  - `AddVertex()`: Add new node to tree
  - `nearest_neighbor()`: Find closest node
  - `get_path()`: Backtrack path from goal

---

## Experimental Results

### Test Scenarios

**Scenario 1: Obstacle-Free Environment**
- Workspace with no obstacles
- RRT* should find near-straight paths
- Convergence: typically 50-100 iterations

**Scenario 2: Single Wall Obstacle**
- Wall blocking direct path
- Requires path planning around obstacle
- Convergence: 200-300 iterations

**Scenario 3: Complex Obstacle Field**
- Multiple walls and boxes
- Narrow passages
- Convergence: 500+ iterations

### Performance Metrics

**Success Rate:**
- Percentage of planning attempts that find solutions
- Depends on problem difficulty and max iterations

**Path Cost:**
- Total distance traveled in configuration space
- RRT: suboptimal, longer paths
- RRT*: improves over iterations, approaches optimal

**Computation Time:**
- Wall-clock time for complete planning
- Depends on collision checking overhead
- Typically 1-30 seconds for UR5e

**Convergence:**
- How quickly path cost improves
- RRT*: significant improvement in first 100-200 iterations
- Diminishing returns after 500+ iterations

---

## Practical Considerations

### Collision Sphere Sizing

**Inflation Factor:** Safety margin for collision detection
- Default: 1.0 (no inflation)
- Recommended: 1.1-1.3 for safety
- Higher values: more conservative, slower planning

### Resolution Parameter

**Impact on Planning:**
- Too high: fast but misses collisions
- Too low: accurate but very slow
- Sweet spot: 0.05-0.1 for UR5e

### Goal Biasing

**Sampling Strategy:**
- Default: 50% uniform, 50% goal-biased
- Higher bias: faster convergence but worse exploration
- Lower bias: thorough search but slower convergence

---

## Key Learnings

### Robot Kinematics
- D-H parameters provide systematic approach to kinematics
- Forward kinematics is straightforward and deterministic
- Inverse kinematics is harder, often numerical

### Motion Planning
- Configuration space makes collision detection efficient
- Sampling-based methods scale to high dimensions
- RRT* provides both feasibility and optimality

### Implementation
- Sphere-based collision detection is practical and fast
- Tree structures enable efficient nearest-neighbor queries
- Visualization critical for debugging and understanding

---

## Usage

```bash
cd Robot-Arm-Kinematics-and-Motion-Planning
python run.py
```

The script will:
1. Initialize UR5e robot with parameters
2. Create workspace environment
3. Test local motion planning
4. Run RRT* algorithm
5. Visualize results

---

## Files

- `assignment.pdf` - Full problem specification
- `kinematics.py` - Robot kinematics
- `planners.py` - RRT-STAR implementation
- `building_blocks.py` - Planning components
- `environment.py` - Obstacle definitions
- `visualizer.py` - Visualization utilities
- `RRTTree.py` - Tree data structure
- `run.py` - Main execution script

---

**Status:** ✅ Complete with kinematics and RRT-Star implementation
**Last Updated:** 2025
**Topics:** Robot Kinematics, Motion Planning, RRT-Star, Collision Detection
