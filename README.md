# Nav2 MPC Tracker with Dynamic Obstacle Avoidance

This repository contains a custom local planner for ROS 2, implementing Model Predictive Control (MPC) with a B-Spline path smoother. The system is designed to handle complex navigation scenarios, including dynamic obstacle avoidance and tight hairpin turns, using mathematically continuous cost functions.

## System Architecture

The navigation stack consists of two primary custom nodes:
1. **`bspline` Node**: Subscribes to sparse CSV waypoints and generates a mathematically smooth, highly dense $C^2$ continuous reference path.
2. **`mpc` Node**: A Model Predictive Controller utilizing `CasADi` and `IPOPT`. It features:
   * **Monotonic Progress Tracking**: Prevents path-jumping at sharp U-turns.
   * **Inverse Barrier Functions**: Creates dynamic repelling fields around LiDAR-detected obstacle surfaces.
   * **Cruise Control Cost**: Forces forward progression to prevent local minimum stalling.

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

---

### The Mathematical Breakdown

If the reviewers ask you how your code works under the hood, here is the technical explanation of the formulas you implemented.

#### 1. The B-Spline Node (`bspline`)
The goal of this node is to convert raw, jagged waypoints into a perfectly smooth mathematical curve  . 

*   **The Problem:** The MPC solver calculates derivatives (gradients). If the path has sharp corners, the gradient explodes to infinity, and the solver crashes.
*   **Heading Extraction:** Instead of using geometry between two points to find the robot's target heading (which can be noisy), we use exact calculus. We ask the B-spline for its first derivative (`splev(..., der=1)`) to get the tangent vectors $dx$ and $dy$  . The target heading ($\psi$) is simply:
    $$\psi = \arctan\left(\frac{dy}{dx}\right)$$

#### 2. The MPC Tracker Node (`mpc`)
The MPC formulates a constrained non-linear optimization problem. It tries to minimize a total "Cost" ($J$) while obeying the laws of physics .

**A. Kinematic Constraints (The Laws of Physics)**
The solver uses the Unicycle Model. It proves to the solver that the robot cannot move sideways .
$$x_{k+1} = x_k + v_k \cos(\theta_k) dt$$
$$y_{k+1} = y_k + v_k \sin(\theta_k) dt$$
$$\theta_{k+1} = \theta_k + \omega_k dt$$

**B. The Cost Function ($J$)**
The total penalty cost is calculated by summing up several distinct mathematical forces :

*   **Position Tracking:** Standard Euclidean squared error to keep the robot on the path.
    $$J_{pos} = Q_x(x_k - x_{ref})^2 + Q_y(y_k - y_{ref})^2$$
*   **Continuous Heading Tracking (The Cosine Trick):** Instead of using $(\theta_k - \theta_{ref})^2$, which causes a massive imaginary error when crossing the West boundary ($+180^\circ$ to $-180^\circ$), we use trigonometry . The Cosine trick mathematically absorbs the $\pm \pi$ jump seamlessly.
    $$J_{heading} = Q_\theta \cdot 2.0 \cdot (1 - \cos(\theta_k - \theta_{ref}))$$
*   **Actuation / Cruise Control:** Instead of penalizing the robot for using its motors (which causes it to park in front of obstacles), we penalize it for *not* driving at maximum speed . This forces the solver to confidently steer around boxes rather than stopping.
    $$J_{vel} = Q_v(v_{max} - v_k)^2$$
*   **Inverse Barrier Obstacle Avoidance:** Rather than drawing a hard boundary circle, we iterate over a downsampled array of LiDAR points representing the physical surface of the obstacle . We divide our weight by the squared distance to create an infinite, repelling magnetic field that ramps up aggressively as the robot gets close . The $0.001$ prevents a divide-by-zero crash .
    $$J_{obs} = \sum_{i=1}^{10} \frac{W_{obs}}{(x_k - obs_{x,i})^2 + (y_k - obs_{y,i})^2 + 0.001}$$
