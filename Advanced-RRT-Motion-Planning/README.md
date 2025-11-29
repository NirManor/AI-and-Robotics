# Advanced RRT Motion Planning

Enhanced implementations of RRT-Star and advanced motion planning techniques with comprehensive experimental analysis and optimization strategies.

## Problem Description

**Objective:** Implement and analyze advanced variants of RRT-Star algorithm with focus on:
- Path optimization through rewiring
- Parameter sensitivity analysis
- Comparative algorithm performance
- Real-world planning scenarios

**Focus Areas:**
- Asymptotic optimality proofs and empirical validation
- Convergence rate analysis
- Parameter tuning for different problem types
- Scalability to complex environments

---

## Advanced Algorithms

### RRT-Star with k-Nearest Rewiring

**Enhanced Features:**

**1. Smart Neighbor Selection**
```
k = log(n) for asymptotic optimality
or
k = constant for practical speedup

Benefits:
- Logarithmic growth reduces rewiring overhead
- Balances optimality and computation time
- Proven asymptotically optimal convergence
```

**2. Rewiring Strategy**

**Parent Selection:**
```python
# Find best parent among k-nearest neighbors
best_cost = infinity
best_parent = nearest_neighbor

for neighbor in k_nearest_neighbors:
    new_cost = cost(start → neighbor) + distance(neighbor, x_new)
    if new_cost < best_cost and collision_free(neighbor, x_new):
        best_cost = new_cost
        best_parent = neighbor

connect(best_parent, x_new)
```

**Neighbor Rewiring:**
```python
# Rewire existing neighbors through new node
for neighbor in k_nearest_neighbors:
    cost_via_new = cost(start → x_new) + distance(x_new, neighbor)
    cost_via_current = cost(start → current_parent → neighbor)

    if cost_via_new < cost_via_current:
        if collision_free(x_new, neighbor):
            change_parent(neighbor, x_new)
            cost(start → neighbor) = cost_via_new
```

**Properties:**
- Asymptotic optimality: lim(n→∞) cost → optimal
- Convergence rate: proportional to 1/n^(1/d)
- Practical: significant improvement in first 200-500 iterations

### Cost Calculation

**Edge Cost:**
```
distance = L2 norm in configuration space
cost = Euclidean distance in joint angle space
units = radians
```

**Path Cost:**
```
total_cost = sum of all edge costs along path from start to goal
```

---

## Experimental Analysis

### Experiment 1: Convergence Rate Analysis

**Objective:** Characterize how path cost improves over time

**Setup:**
- Multiple planning runs (20+ trials)
- Same start/goal configuration
- Track best path cost vs iteration count

**Results:**
- **Phase 1 (0-100 iterations):** Rapid improvement, 30-50% cost reduction
- **Phase 2 (100-500 iterations):** Moderate improvement, 15-25% reduction
- **Phase 3 (500+ iterations):** Asymptotic convergence, <10% improvement

**Key Finding:** Most benefit achieved in first 200 iterations

### Experiment 2: k-Nearest Parameter Sensitivity

**Objective:** Compare k=constant vs k=log(n)

**Configurations Tested:**
```
k = 5 (constant)
k = 10 (constant)
k = log(n) (logarithmic)
```

**Metrics:**
| k Strategy | Iteration Time | Path Quality | Memory | Asymptotic |
|-----------|----------------|-------------|--------|-----------|
| k=5 | ~50ms | Near-optimal | Low | No |
| k=10 | ~100ms | Good | Low | No |
| k=log(n) | ~150ms | Optimal | Moderate | Yes |

**Analysis:**
- Constant k: faster but bounded path quality
- Logarithmic k: slower but provably optimal
- Trade-off: accept 2-3× slowdown for optimality guarantee

### Experiment 3: Problem Difficulty Impact

**Scenarios:**
1. **Obstacle-Free:** Straight-line path
2. **Single Obstacle:** One wall to navigate around
3. **Multiple Obstacles:** Complex narrow passages
4. **High-Dimensional:** Full 6-DOF complexity

**Results:**
```
Obstacle-Free:        20-50 iterations for near-optimal
Single Obstacle:      100-200 iterations
Multiple Obstacles:   300-500 iterations
High-Dimensional:     1000+ iterations needed
```

**Observation:** Difficulty strongly affects convergence rate

### Experiment 4: Comparison with RRT

**RRT vs RRT-Star:**

| Metric | RRT | RRT-Star |
|--------|-----|----------|
| **Solution Quality** | Non-optimal | Asymptotic optimal |
| **Optimality Gap** | 20-50% | Converges to 0% |
| **Computation Time** | Fast | 1.5-2× slower |
| **Scalability** | Excellent | Good |
| **Best Use** | Quick feasibility | Quality paths |

---

## Performance Optimization

### Spatial Indexing

**Nearest-Neighbor Search:**
```
Naive: O(n) comparison with all nodes
KD-Tree: O(log n) with tree structure
Ball Tree: O(log n) with good constant factors
```

**Implementation Impact:**
- Improves per-iteration time from O(n) to O(log n)
- Critical for problems with 1000+ nodes
- NumPy vectorization provides additional speedup

### Collision Checking Optimization

**Early Termination:**
```python
# Check coarse collision first (fast)
if coarse_collision(path):
    return False

# Detailed collision only if coarse passes
for segment in path:
    if detailed_collision(segment):
        return False
return True
```

**Benefits:**
- 70-80% of checks can be terminated early
- Reduces average collision check time
- Minimal impact on correctness

### Memory Management

**Issue:** Storing large trees consumes memory

**Solutions:**
1. Store only necessary node data
2. Prune non-promising branches
3. Limit maximum tree size
4. Use memory-mapped data structures

---

## Path Quality Metrics

### Cost-Based Metrics

**Path Length:**
```
L = sum of edge lengths along path
Units: radians (configuration space)
```

**Smoothness:**
```
Smoothness = measure of path curvature
Lower = smoother (less jerky motion)
Higher = more direct but potentially infeasible
```

### Robot-Specific Metrics

**Joint Displacement:**
```
For each joint, sum of angle changes
Indicator of joint wear and energy consumption
```

**Workspace Deviation:**
```
How far end-effector deviates from straight line
Important for tasks requiring linear end-effector motion
```

---

## Advanced Techniques

### Informed Sampling (Optional Extension)

**Idea:** Bias sampling toward improving region

```python
if has_solution():
    # Sample from ellipse connecting start/goal through current best
    x_sample = sample_from_informed_set()
else:
    # Standard uniform sampling
    x_sample = sample_uniform()
```

**Benefits:**
- Faster convergence in later iterations
- Maintains probabilistic completeness
- Especially useful for optimization phase

### Bidirectional Planning (Optional Extension)

**Approach:** Plan from start and goal simultaneously

```python
Tree_start = {start}
Tree_goal = {goal}

while not connected:
    extend_from_smaller_tree()
    if trees_overlap():
        return merge_paths()
```

**Advantages:**
- Faster for long-distance problems
- Better exploration near obstacles
- Can reduce iteration count significantly

---

## Implementation Details

### Key Classes

**RRT_STAR (Enhanced):**
- `find_path()` - Main algorithm with rewiring
- `extend()` - Tree expansion
- `rewire()` - Neighbor optimization
- `get_k_num()` - Dynamic k calculation
- `get_cost()` - Path cost computation

**Building_Blocks (Advanced):**
- Optimized collision detection
- Spatial indexing for nearest-neighbor
- Advanced sampling strategies

---

## Files

- `assignment.pdf` - Problem specification
- `report.pdf` - Detailed analysis and results
- `kinematics.py` - UR5e parameters
- `planners.py` - RRT-STAR implementation
- `building_blocks.py` - Advanced components
- `environment.py` - Test scenarios
- `visualizer.py` - Result visualization
- `RRTTree.py` - Tree data structure
- `run.py` - Main execution script

---

## Usage

```bash
cd Advanced-RRT-Motion-Planning
python run.py
```

Runs multiple experiments:
1. Convergence analysis
2. Parameter sensitivity study
3. Algorithm comparison
4. Problem difficulty analysis

---

## Key Learnings

### Algorithm Design
- Asymptotic optimality requires careful parameter choice
- k=log(n) strategy provides best theoretical properties
- Trade-offs between optimality and computation time unavoidable

### Experimental Validation
- Multiple trials essential for statistical significance
- Problem type strongly affects algorithm performance
- Parameters must be tuned per problem class

### Real-World Application
- Theory doesn't always match practice
- Implementation details matter (collision checking, data structures)
- Visualization critical for algorithm understanding

---

**Status:** ✅ Complete with advanced RRT-Star and experimental analysis
**Last Updated:** 2025
**Topics:** Asymptotic Optimality, Path Optimization, Experimental Analysis, Algorithm Comparison
