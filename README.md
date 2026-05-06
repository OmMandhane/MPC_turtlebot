# Nav2 MPC Tracker with Dynamic Obstacle Avoidance

This repository contains a custom local planner for ROS 2, implementing Model Predictive Control (MPC) with a B-Spline path smoother. The system is designed to handle complex navigation scenarios, including dynamic obstacle avoidance and tight hairpin turns, using mathematically continuous cost functions.

## System Architecture

The navigation stack consists of two primary custom nodes:

1. **`bspline` Node**: Subscribes to sparse CSV waypoints and generates a mathematically smooth, highly dense $C^2$ continuous reference path.

2. **`mpc` Node**: A Model Predictive Controller utilizing `CasADi` and `IPOPT`. It features:
   - **Monotonic Progress Tracking**: Prevents path-jumping at sharp U-turns.
   - **Inverse Barrier Functions**: Creates dynamic repelling fields around LiDAR-detected obstacle surfaces.
   - **Cruise Control Cost**: Forces forward progression to prevent local minimum stalling.
   - **Solver Warm Starting**: Injects inertia to prevent saddle-point oscillations.

## Usage

Launch the simulation environment, followed by the path generator and the MPC tracker.

**Terminal 1: Bring up the Simulation**
```bash
ros2 launch nav sim_bringup.launch.py
```

**Terminal 2: Launch the Path Smoother**
```bash
ros2 run nav bspline
```

**Terminal 3: Launch the MPC Tracker**
```bash
ros2 run nav mpc --ros-args -p use_sim_time:=true
```


## The Mathematical Breakdown

### 1. The B-Spline Node (`bspline`)

The MPC solver requires gradients (derivatives) to optimize the path. If raw waypoints are used, the sharp corners cause the gradients to explode, crashing the solver.

We fit a Cubic (Degree 3) B-Spline to the waypoints to guarantee $C^2$ continuity (smooth acceleration/steering). Instead of calculating headings using noisy point-to-point geometry, we use exact calculus by taking the first derivative of the continuous spline to extract the precise tangent vectors for the robot's target heading.

### 2. The MPC Tracker Node (`mpc`)

The MPC formulates a constrained non-linear optimization problem, minimizing a total "Cost" ($J$) over a prediction horizon of $N=25$ steps while obeying physical boundaries.

#### A. Kinematic Constraints (The Laws of Physics)

The solver uses the Unicycle Model, enforcing differential-drive physics to ensure the predicted trajectory is physically possible:

$$x_{k+1} = x_k + v_k \cos(\theta_k) dt$$

$$y_{k+1} = y_k + v_k \sin(\theta_k) dt$$

$$\theta_{k+1} = \theta_k + \omega_k dt$$

#### B. The Cost Function ($J$)

The total objective function is the sum of four competing mathematical forces. The solver continuously balances these to find the optimal steering and velocity commands.

**1. Position Tracking:** Standard Euclidean squared error pulling the robot to the path.

$$J_{pos} = Q_x(x_k - x_{ref})^2 + Q_y(y_k - y_{ref})^2$$

**2. Continuous Heading Tracking (The Cosine Trick):** Using standard squared error $(\theta_k - \theta_{ref})^2$ causes a mathematical singularity when crossing West (the $+180^\circ$ to $-180^\circ$ boundary), resulting in violent $360^\circ$ steering oscillations. We replace it with trigonometry. Based on Taylor Series expansion, this perfectly matches squared error for small angles but seamlessly absorbs the $\pm \pi$ jump:

$$J_{heading} = Q_\theta \cdot 2.0 \cdot (1 - \cos(\theta_k - \theta_{ref}))$$

**3. Cruise Control (Forward Motivation):** Standard MPC penalizes energy usage ($v_k^2$), which causes the robot to freeze and park when faced with an obstacle. Instead, we heavily penalize the robot for not driving at its maximum speed. This forces the solver to confidently swerve around boxes to maintain velocity.

$$J_{vel} = Q_v(v_{max} - v_k)^2 + R_w(\omega_k)^2$$

**4. Inverse Barrier Obstacle Avoidance:** Instead of drawing hard bounding boxes, we divide our obstacle weight by the squared distance to create an infinite, repelling magnetic field. The penalty is negligible at a distance but ramps aggressively to infinity as the robot approaches the surface. (The $0.001$ prevents a divide-by-zero crash).

$$J_{obs} = \sum_{i=1}^{10} \frac{W_{obs}}{(x_k - obs_{x,i})^2 + (y_k - obs_{y,i})^2 + 0.001}$$

#### C. Advanced Engineering Techniques

To make the MPC robust in complex, dynamic environments, several advanced techniques were implemented outside of the core solver math:

**Monotonic Progress Tracking:** Standard path trackers naively search for the absolute closest point on the global path. In tight U-turns, this causes the robot to "snap" to the opposite lane. We mitigate this by giving the robot a memory index (_progress_idx) and utilizing a narrow 50-point forward-search window, ensuring the trajectory remains monotonically directional.

**LiDAR Surface Downsampling:** Feeding a raw 360-degree laser scan into a CasADi solver causes immediate computation failure. The node processes the LiDAR data by downsampling (reading every 15th ray) and dynamically extracting the absolute 10 closest physical surface coordinates. This allows the MPC to route around the exact geometry of the obstacle rather than a generic center-point.

**Solver Warm Starting (Memory):** When an obstacle is perfectly centered on the path, the cost of dodging left vs. dodging right is identical (a "Saddle Point"). This can cause the solver to oscillate back and forth indefinitely. By caching the optimized output arrays from time $T-1$ and injecting them as the initial guess for time $T$, the solver gains mathematical "inertia," securely locking into an avoidance decision without stuttering.