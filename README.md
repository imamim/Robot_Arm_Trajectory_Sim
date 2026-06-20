# Task Space Trajectory Generation and Tracking Simulation of a 2DoF Robot

🚀 This repository is explicitly designed to serve as an open framework to accelerate and facilitate development for students and researchers looking to model, simulate, and control various multi-joint robotic systems.

> ### 🤝 Open for Contributions!
> 
> 
> 💡 **Want to see your favorite robot in action?** Whether you want to implement a new manipulator (e.g., a 6-DoF arm like the Staubli TX90 or a 7-DoF redundant robot), optimize a trajectory generation pipeline, or experiment with alternative nonlinear control laws, your ideas are welcome!
> ✨ Feel free to branch out, innovate, and **open a Pull Request (PR) whenever you wish!** Let's build a highly modular robotics playground together! 🛠️🤖

## 3. Methodology

### Kinematics

Kinematics models the spatial transitions of the manipulator independent of the actuation forces.

* **Forward Kinematics (FK):** Maps the joint configuration vector $q = [\theta_1, \theta_2]^T$ to Cartesian end-effector coordinates $x = [x_{pos}, y_{pos}]^T$:

$$x_{pos} = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2)$$


$$y_{pos} = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2)$$


* **Differential Kinematics:** Establishes the relationship between joint-space velocities $\dot{q}$ and task-space velocities $\dot{x}$ using the analytical space Jacobian $J(q)$:

$$\dot{x} = J(q)\dot{q}$$


* **Singularity Avoidance:** To guarantee bounded control signals near kinematic singular boundaries, the workspace solver relies on a dynamic Damped Least Squares (DLS) pseudo-inverse:

$$J^{\dagger} = J^T (J J^T + \lambda^2 I)^{-1}$$



where the damping factor $\lambda$ scales actively based on the Yoshikawa Manipulability Measure.

### Dynamics

The rigid-body equations of motion are derived using the energy-based Lagrange-Euler formulation:


$$M(\theta)\ddot{\theta} + c(\theta, \dot{\theta}) + g(\theta) = \tau$$

* **$M(\theta) \in \mathbb{R}^{2\times2}$:** Symmetric, positive-definite mass/inertia matrix.
* **$c(\theta, \dot{\theta}) \in \mathbb{R}^2$:** Coriolis and centrifugal force vector.
* **$g(\theta) \in \mathbb{R}^2$:** Gravity force vector pulling links downwards.
* **$\tau \in \mathbb{R}^2$:** Vector of applied joint torques.

### Reference Path & Cubic Spline Formulation

The task-space reference inputs are derived from a horizontally oriented figure-eight Lissajous curve:


$$x_{ref}(t) = x_c + A_x \sin\left(\frac{2\pi t}{T}\right)$$

$$y_{ref}(t) = y_c + A_y \sin\left(\frac{4\pi t}{T}\right)$$


To match physical constraint limits, discrete path waypoints are interpolated via piecewise cubic splines:


$$p_i(t) = a_i(t - t_i)^3 + b_i(t - t_i)^2 + c_i(t - t_i) + d_i$$


This formulation ensures $C^2$ continuity, providing smooth, analytical position ($x_d$), velocity ($\dot{x}_d$), and acceleration ($\ddot{x}_d$) reference profiles across spline boundaries.


## 4. Features

* **Custom Simulation Engine:** A continuous-time forward dynamics simulation environment built entirely in MATLAB ('ode45').
* **Actuator Constraints:** Actuator upper and lower torque limit applied to virtual servos.
* **Operational Space Control Architecture:** Implementation of Proportional-Integral-Derivative (PID) state-feedback (NDI) structures with feedforward trajectory acceleration.
* **Scalable Matrix Formulations:** As long as the system definition method given in the dynamic section is followed, the robot model can be updated, and simulations for different robots can be performed without changing the controller and spline production.



## 7. Results and Validation

### Figure 1: Position Tracking Performance

The Operational Space Controller eliminates cross-coupling issues, guiding the end-effector with sub-millimeter precision along the continuous cubic spline target. Both $X$ and $Y$ states exhibit zero steady-state error and zero transient overshoot.

<p align="center">
  <img src="assets/OSC Analysis - Cartesian Tracking.png" alt="Workspace Figure-Eight Position Tracking" width="480" height="720">
</p>

### Figure 2: Applied Joint Torques

The motor effort satisfies physical system properties. The required control torque peaks safely at **3.5 Nm** on Joint 1, operating comfortably within the actuator hardware limit bounds of **$\pm10$ Nm**.

<p align="center">
  <img src="assets/OSC Analysis - Torque Saturation.png" alt="Workspace Figure-Eight Position Tracking" width="480" height="720">
</p>


*The exact overlay between commanded and applied torques verifies that no control saturation clipping occurs.*

---

## 8. References

* [1] O. Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation," *IEEE Journal of Robotics and Automation*, vol. RA-3, no. 1, Feb. 1987.
* [2] J. Z. Kolter and A. Y. Ng, "Task-Space Trajectories via Cubic Spline Optimization," *Computer Science Department, Stanford University*, 2009.
* [3] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press, 2017.
