# Bead Maze Final Project

Advanced 3D motion planning for a robotic arm navigating a complex bead maze puzzle, combining task-space planning, path optimization, and real-world constraints.

## Project Overview

**Challenge:** Navigate a UR5e robotic arm through a 3D bead maze puzzle where beads must be threaded onto wire structures.

**Team:** Nir Manor (305229627), Ortal Cohen (308524875)

**Complexity:** Advanced application of multiple planning paradigms

---

## Problem Description

### The Bead Maze Puzzle

**Physical Setup:**
- 3D wire structure with complex geometry
- Multiple beads positioned on wire segments
- Threading paths with narrow passages
- Gravity-based constraints

**Robotic Task:**
```
Sequence:
1. Position end-effector to bead location
2. Maneuver bead along wire path
3. Navigate around wire turns and obstacles
4. Place bead at destination on wire
5. Repeat for all beads (if applicable)
```

**Challenges:**
- **3D Geometry:** Complex wire routing in 3D space
- **Narrow Passages:** Tight clearances require precision
- **End-Effector Orientation:** Must align with wire geometry
- **Contact Constraints:** Maintain contact during threading
- **Collision Avoidance:** Robot arm avoidance while manipulating bead
- **Real-world Physics:** Gravity, friction, bead mass

---

## Advanced Planning Concepts

### Task-Space Planning

**Definition:** Planning in end-effector space (position and orientation)

**Advantages:**
- More intuitive for manipulation tasks
- Easier to specify geometric constraints
- Direct connection to physical task

**Challenges:**
- Non-linear transformation to configuration space
- Singularities where mapping is undefined
- Multiple configuration solutions per task-space point

**Implementation:**
```
1. Specify task in end-effector coordinates
2. Generate task-space trajectory
3. Solve inverse kinematics at each point
4. Validate configuration-space path
5. Refine if needed for smoothness
```

### Frechet Distance

**Definition:** Similarity metric between two paths

**Mathematical Formulation:**
```
F(P,Q) = inf max ||p(u) - q(v)||
         γ  (u,v)∈γ

where γ is a reparameterization of both paths
```

**Intuition:** Minimum "leash length" for dog-person following paths

**Applications:**
- Comparing planned path with teacher demonstration
- Path similarity analysis
- Quality metric for path optimization

**Computation:**
- O(mn log(mn)) for paths with m,n segments
- Requires careful implementation for accuracy
- Useful for validation and iterative refinement

---

## Planning Approach

### Phase 1: Global Path Planning

**Objective:** Find collision-free path from start to goal

**Method:** RRT* in task space
```
1. Sample end-effector targets
2. Plan path between targets
3. Validate via inverse kinematics
4. Rewire for optimization
```

**Output:** Sequence of end-effector poses

### Phase 2: Inverse Kinematics Solution

**Problem:** Convert end-effector poses to joint angles

**Challenges:**
- Multiple solutions per pose (6 solutions for UR5e)
- Some poses unreachable (singularities)
- Need continuous solution across path

**Approach:**
```
1. Compute multiple IK solutions
2. Choose solution minimizing joint jumps
3. Verify collision-free in configuration space
4. Smooth joint trajectory
```

### Phase 3: Path Refinement

**Goal:** Optimize path for task requirements

**Refinements:**
1. **Smoothing:** Reduce joint jerks
2. **Collision-Free Validation:** Verify at fine resolution
3. **Contact Maintenance:** Ensure bead stays on wire
4. **Energy Optimization:** Minimize joint torques

**Iterative Process:**
```
while not optimized:
    validate_path()
    if collisions:
        fix_collision()
    if not_smooth:
        smooth_path()
    improve_cost()
```

---

## Technical Implementation

### Wire Representation

**Geometry:**
```
Wire segments represented as line curves
Junction points where segments meet
Bead position: parametric location on wire segment

Example:
Wire = [Segment1, Segment2, Segment3, ...]
Segment = {start_point, end_point, radius}
Bead = {wire_segment_index, parameter_t}
```

**Collision Checking:**
```
Robot link distance to wire geometry
Bead position relative to wire path
End-effector approach angles
```

### End-Effector Orientation

**Constraints:**
```
Maintain specific orientation relative to wire
Enable bead threading
Allow passage through narrow sections

Representation:
- Quaternions (singularity-free)
- Roll-Pitch-Yaw angles (intuitive but singular)
- Rotation matrices (computational)
```

### Path Quality Metrics

**Feasibility:**
```
All configurations reachable (no singularities)
Collision-free throughout path
Constraints satisfied
```

**Optimality:**
```
Minimize path length in task space
Minimize joint motion in configuration space
Minimize required end-effector acceleration
```

**Robustness:**
```
Maintain safety margins from obstacles
Smooth trajectory for control
Recovery strategies for disturbances
```

---

## Experimental Results

### Solution Approach

**Method 1: Direct Task-Space Planning**
- Pros: Intuitive, directly solves task
- Cons: Inverse kinematics can be problematic
- Result: Works for accessible configurations

**Method 2: Configuration-Space Planning**
- Pros: Guaranteed smooth solutions
- Cons: Difficult to incorporate task constraints
- Result: Good for collision avoidance

**Method 3: Hybrid Approach**
- Pros: Combines advantages of both
- Cons: More complex implementation
- Result: Best overall performance

### Performance Metrics

**Path Quality:**
- Successfully threads bead through maze
- Smooth motion suitable for real robot
- Minimum path length (within 10-20% of optimal)

**Computation:**
- Planning time: <60 seconds
- Memory usage: acceptable for lab computer
- Scalability: handles multiple beads

**Validation:**
- Collision-free path verified at multiple resolutions
- Joint limits satisfied throughout
- Singularity avoidance confirmed

---

## Key Challenges Addressed

### 1. Complex 3D Geometry
- **Solution:** Accurate wire representation
- **Validation:** Point-to-wire distance computation

### 2. End-Effector Orientation
- **Solution:** Quaternion-based orientation planning
- **Validation:** Rotation matrix consistency checks

### 3. Inverse Kinematics Multiple Solutions
- **Solution:** Solution selection based on path continuity
- **Validation:** Verification of continuity in joint space

### 4. Real-world Uncertainties
- **Solution:** Robust path with safety margins
- **Validation:** Simulation with uncertainty ranges

### 5. Optimization Under Constraints
- **Solution:** Constrained optimization framework
- **Validation:** Constraint satisfaction verification

---

## Files

- `report.pdf` - Complete project report with:
  - Problem analysis
  - Approach description
  - Mathematical formulations
  - Experimental results
  - Conclusions and future work

---

## Report Contents

The comprehensive project report includes:

**1. Introduction**
- Problem motivation
- Bead maze description
- Project objectives

**2. Technical Approach**
- Kinematic modeling
- Planning algorithms
- Constraint handling
- Optimization methods

**3. Implementation**
- System architecture
- Data structures
- Algorithm details
- Numerical considerations

**4. Experimental Results**
- Solution visualizations
- Performance metrics
- Comparison of approaches
- Statistical analysis

**5. Analysis**
- Strengths of approach
- Limitations encountered
- Robustness considerations
- Scalability discussion

**6. Conclusion**
- Key findings
- Practical insights
- Future improvements
- Broader applications

---

## Key Contributions

### Novel Aspects
- Hybrid task-space and configuration-space planning
- Frechet distance application to path optimization
- Constraint-aware RRT* variant for manipulation

### Technical Insights
- Importance of smooth joint trajectories
- End-effector orientation challenges
- Trade-offs in planning strategies

### Practical Results
- Successfully planned complex manipulation task
- Demonstrated scalability to multi-segment paths
- Validated on UR5e robot constraints

---

## Lessons Learned

### Problem Complexity
- Real-world problems far more complex than textbook examples
- Multiple conflicting objectives require careful trade-offs
- Constraint satisfaction is often the bottleneck

### Algorithm Selection
- No single algorithm works for all problems
- Hybrid approaches often superior
- Problem-specific tuning essential

### Validation and Testing
- Simulation inadequate alone
- Real robot testing critical for validation
- Safety and robustness paramount in manipulation

### Team Collaboration
- Clear communication on algorithmic choices
- Division of work across implementation and testing
- Iterative refinement through discussion

---

## Future Work

**Extensions:**
- Real robot implementation and testing
- Learning-based approaches for parameter tuning
- Multi-arm coordination
- Dynamic obstacle avoidance

**Improvements:**
- Faster planning algorithms
- Better inverse kinematics solutions
- Adaptive constraint handling
- Uncertainty quantification

**Applications:**
- Generalization to other manipulation tasks
- Integration with computer vision
- Autonomous manipulation systems
- Human-robot collaboration

---

## References

See main README for comprehensive references on:
- Robot kinematics and dynamics
- Motion planning algorithms
- Path optimization techniques
- Real-time control systems

---

**Status:** ✅ Complete final project with comprehensive report
**Last Updated:** 2025
**Topics:** Task-Space Planning, Manipulation, Constraint Optimization, Real-world Robotics
