# Environment Design

This document describes the proposed reinforcement learning environment for the suspension control simulation.

## Simulation Objectives
- **Minimize chassis oscillation** when driving over randomly generated speed bumps.
- **Maintain vehicle stability** by reducing pitch and roll motion.
- **Encourage efficient control** so that motors apply only the torque necessary for smooth traversal.

## Observation Vector
The agent receives the following observations each simulation step:
1. Vertical acceleration of the chassis (`accel_z`).
2. Pitch rate from the gyroscope (`gyro_pitch`).
3. Wheel angular velocities (one per wheel).
4. Motor torques currently applied (one per wheel).
5. Vehicle forward velocity.

All observations are concatenated into a single vector.

## Action Space
The agent outputs a torque command for each wheel motor.
- **Dimension:** 4 actions (front-left, front-right, rear-left, rear-right).
- **Range:** continuous values between `-1.0` and `1.0`, scaled to the motor's maximum torque capability.

## Reward Function
At each step, the reward encourages minimal chassis movement while penalizing large control signals:

```
reward = - (w_acc * |accel_z| + w_pitch * |gyro_pitch| + w_ctrl * sum(|action|))
```

where `w_acc`, `w_pitch`, and `w_ctrl` are weighting factors chosen to balance comfort and energy use. Larger negative rewards correspond to undesirable oscillation or high control effort.

## Episode Start
An episode begins with the vehicle positioned before a sequence of speed bumps generated via `bump_generator.py` and inserted using `update_mjcf.py`. The vehicle is at rest with zero initial torque applied.

## Termination Conditions
An episode terminates when one of the following occurs:
1. The vehicle reaches the end of the track.
2. The simulation time exceeds a specified limit.
3. The vehicle flips or becomes unstable (e.g., large roll or pitch beyond safe bounds).

These conditions ensure that learning focuses on traversing bumps smoothly without indefinite runtimes.

