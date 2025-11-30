# AI and Robotics - Motion Planning for Robotic Manipulators

Complete implementations of motion planning algorithms for the UR5e collaborative robotic arm, covering kinematics, collision detection, and advanced path planning techniques.

## Course Information

- **Course Number:** 236630
- **Course Name:** AI and Robotics
- **Institution:** Technion - Israel Institute of Technology
- **Faculty:** Computer Science
- **Semester:** Winter 2023
- **Language:** Python 3

---

## Repository Overview

This repository contains practical implementations of motion planning for industrial robotic arms with a focus on:
- Forward and inverse kinematics for 6-DOF manipulators
- Collision detection in 3D workspaces
- Sampling-based motion planning algorithms (RRT, RRT*)
- Task-space planning and path optimization
- Real-world robotic manipulation challenges

All implementations use the **UR5e** collaborative robot and emphasize practical application of planning algorithms to real robot constraints.

---

## Projects

### 1. Robot Arm Kinematics & Motion Planning Basics
**Path:** `Robot-Arm-Kinematics-and-Motion-Planning/`

**Topic:** Fundamentals of robot kinematics and local motion planning

**Key Concepts:**
- **Forward Kinematics:** Computing end-effector position from joint angles
  - Denavit-Hartenberg (D-H) parameters
  - Homogeneous transformations
  - Link geometry and frame definitions

- **Inverse Kinematics:** Finding joint angles for target end-effector positions
  - Numerical solutions
  - Configuration space representation

- **Local Motion Planning:** Collision-free straight-line motion
  - Resolution-based path validation
  - Sphere-based collision detection
  - Workspace obstacle representation

- **RRT-Star Introduction:** Sampling-based global planning
  - Tree construction and growth
  - Nearest-neighbor search
  - Basic path extension

**UR5e Robot:**
- 6-DOF industrial collaborative manipulator
- Joint range limits: ±π for all joints
- Link collision spheres for safety and planning
- Tool configuration with gripper

**Files:**
- `kinematics.py` - UR5e parameters, forward/inverse kinematics
- `planners.py` - RRT-STAR algorithm implementation
- `building_blocks.py` - Fundamental components (sampling, collision checking)
- `environment.py` - 3D workspace with obstacles
- `visualizer.py` - 3D visualization utilities
- `assignment.pdf` - Problem specification

---

### 2. Advanced RRT Motion Planning
**Path:** `Advanced-RRT-Motion-Planning/`

**Topic:** Enhanced RRT variants and algorithm optimization

**Algorithms Implemented:**
- **RRT* (Asymptotically Optimal):** Path rewiring for cost improvement
  - k-nearest neighbor rewiring
  - Iterative path optimization
  - Convergence to near-optimal solutions

- **Path Optimization Techniques:**
  - Cost function analysis
  - Parameter sensitivity study
  - Trade-offs between planning time and solution quality

- **Comparative Analysis:**
  - RRT vs RRT* performance metrics
  - Success rate and convergence behavior
  - Computational efficiency

**Experimental Results:**
- Success rate analysis across different configurations
- Path cost comparison
- Planning time and iteration efficiency

**Files:**
- `kinematics.py` - UR5e kinematics
- `planners.py` - Enhanced RRT and RRT* implementations
- `building_blocks.py` - Advanced collision detection
- `environment.py` - Complex workspace scenarios
- `visualizer.py` - Result visualization
- `assignment.pdf` - Problem specification
- `report.pdf` - Solution with experimental analysis

---

### 3. Complex Motion Planning Tasks
**Path:** `Complex-Motion-Planning-Tasks/`

**Topic:** Advanced planning scenarios beyond point-to-point navigation

**Task Types:**
- **Multi-Target Planning:** Visiting multiple waypoints sequentially
- **Constrained Motion:** Planning under kinematic and workspace constraints
- **Task Coordination:** Combining multiple objectives
- **Obstacle-Rich Environments:** Navigation through complex scenarios

**Advanced Techniques:**
- Adaptive sampling strategies
- Task-aware goal biasing
- Configuration-space analysis
- Path quality metrics specific to tasks

**Implementation Highlights:**
- Task-specific planner variants
- Result visualization with cost analysis
- Performance metrics for complex tasks

**Files:**
- `building_blocks.py` - Task-aware planning components
- `environment.py` - Complex scenario definitions
- `kinematics.py` - Robot model
- `planners.py` - Task-specific planners
- `visualizer.py` - Advanced visualization
- `assignment.pdf` - Problem specification

---

### 4. Bead Maze Final Project
**Path:** `Bead-Maze-Final-Project/`

**Topic:** Complex 3D motion planning for real-world manipulation

**Challenge:**
Navigate a robotic arm through a 3D bead maze puzzle, planning collision-free paths that thread beads onto wire structures with complex 3D geometry.

**Advanced Concepts:**
- **Task-Space Planning:** Planning in end-effector space (position/orientation)
- **Frechet Distance:** Metric for path similarity and quality
- **Multi-Objective Optimization:**
  - Minimize path length
  - Ensure geometric feasibility
  - Balance smoothness and directness
  - Handle complex constraints

- **Constraint Handling:**
  - Bead position constraints
  - Wire collision avoidance
  - End-effector orientation requirements
  - Workspace limitations

**Team Project:**
- Collaborators: Nir Manor (305229627), Ortal Cohen (308524875)
- Comprehensive solution with multiple planning approaches
- Detailed analysis of feasibility and optimality

**Key Achievements:**
- Successful bead maze navigation
- Optimized path solutions
- Robust collision detection
- Visualization of planning process

**Files:**
- `report.pdf` - Complete project report with results

---

## Technical Stack

**Robot Platform:**
- UR5e Collaborative Robot Arm
- 6 rotational joints
- Workspace: ~850mm radius
- Joint torque limits and speed constraints

**Software:**
- **Language:** Python 3.7+
- **Libraries:**
  - NumPy (numerical computation)
  - Matplotlib (visualization)
  - SciPy (scientific computing)

**Algorithms:**
- Forward/Inverse Kinematics (Denavit-Hartenberg)
- RRT (Rapidly-Exploring Random Trees)
- RRT* (Asymptotically Optimal)
- Sampling-based motion planning
- Collision detection (sphere-based)

---

## Key Concepts

### Configuration Space (C-Space)
- High-dimensional representation of robot joint angles
- Obstacles map to forbidden regions in C-Space
- Planning = finding collision-free path in C-Space

### Kinematics
- **Forward Kinematics:** Joint angles → End-effector position
- **Inverse Kinematics:** End-effector position → Joint angles
- Enables planning in both task space and configuration space

### Motion Planning
- Finding collision-free, optimized paths
- Handling constraints: joint limits, workspace boundaries
- Trade-offs: optimality vs. computation time

### Sampling-Based Planning
- Probabilistically complete exploration
- Efficient for high-dimensional spaces
- RRT and variants provide asymptotic optimality

### Collision Detection
- Sphere-based approximation of robot links
- Efficient spatial queries
- Supports continuous path validation

---

## Installation

### Prerequisites
```bash
Python 3.7+
pip
```

### Install Dependencies
```bash
pip install numpy matplotlib scipy
```

### Clone Repository
```bash
git clone https://github.com/NirManor/AI-and-Robotics.git
cd AI-and-Robotics
```

---

## Usage

### Project 1: Robot Arm Kinematics
```bash
cd Robot-Arm-Kinematics-and-Motion-Planning
python run.py
```

### Project 2: Advanced RRT Planning
```bash
cd Advanced-RRT-Motion-Planning
python run.py
```

### Project 3: Complex Tasks
```bash
cd Complex-Motion-Planning-Tasks
python task3_run.py
```

### Project 4: Bead Maze
See `Bead-Maze-Final-Project/report.pdf` for complete implementation details.

---

## Repository Structure

```
AI-and-Robotics/
├── README.md (this file)
│
├── Robot-Arm-Kinematics-and-Motion-Planning/
│   ├── README.md
│   ├── assignment.pdf
│   ├── kinematics.py
│   ├── planners.py
│   ├── building_blocks.py
│   ├── environment.py
│   ├── visualizer.py
│   ├── RRTTree.py
│   ├── run.py
│   └── data/
│
├── Advanced-RRT-Motion-Planning/
│   ├── README.md
│   ├── assignment.pdf
│   ├── report.pdf
│   ├── kinematics.py
│   ├── planners.py
│   ├── building_blocks.py
│   ├── environment.py
│   ├── visualizer.py
│   ├── RRTTree.py
│   ├── run.py
│   └── data/
│
├── Complex-Motion-Planning-Tasks/
│   ├── README.md
│   ├── assignment.pdf
│   ├── kinematics.py
│   ├── planners.py
│   ├── building_blocks.py
│   ├── environment.py
│   ├── visualizer.py
│   ├── RRTTree.py
│   ├── task3_run.py
│   └── data/
│
└── Bead-Maze-Final-Project/
    ├── README.md
    ├── report.pdf
    └── data/
```

---

## Learning Outcomes

### Robotics
- Robot kinematics (forward and inverse)
- Configuration space representation
- Collision detection and avoidance
- Real-world robot constraints

### Motion Planning
- Sampling-based algorithms (RRT, RRT*)
- Path optimization techniques
- Multi-objective planning
- Constraint handling

### Implementation Skills
- Python scientific computing
- 3D geometric algorithms
- Algorithm performance analysis
- Visualization and debugging

### Problem Solving
- Real-world robotic challenges
- Algorithm selection and tuning
- Trade-off analysis
- Solution validation

---

## Key Algorithms

| Algorithm | Application | Optimality | Completeness |
|-----------|-------------|-----------|--------------|
| **Forward Kinematics** | End-effector positioning | - | Yes |
| **Inverse Kinematics** | Target configuration | Numerical | Local |
| **Local Planner** | Straight-line motion | Optimal | Yes |
| **RRT** | Global path planning | Non-optimal | Probabilistic |
| **RRT\*** | Optimized planning | Asymptotic | Probabilistic |

---

## References

1. **Robot Kinematics:**
   - Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot Modeling and Control* (2nd ed.). John Wiley & Sons.
   - Craig, J. J. (2009). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson Education.

2. **Motion Planning:**
   - LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
   - Karaman, S., & Frazzoli, E. (2011). "Sampling-based Algorithms for Optimal Motion Planning."

3. **Collision Detection:**
   - Ericson, C. (2004). *Real-Time Collision Detection*. Morgan Kaufmann Publishers.

4. **UR Robots:**
   - Universal Robots. (2023). UR5e Technical Specifications.
   - UR Robot Programming Manual.

---

## License

Educational use. Course materials property of Technion.

---

**Repository:** https://github.com/NirManor/AI-and-Robotics
**Last Updated:** 2025
**Status:** Complete implementations with comprehensive documentation
