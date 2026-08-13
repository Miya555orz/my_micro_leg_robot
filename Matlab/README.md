# Mini Wheel-Leg MATLAB Model

Author: Miya Zheng  
Created: 2026-07-30

This folder contains the MATLAB model used to calculate first-pass balance gains for the mini single-link wheel-leg robot.

Start with `README_single_link_model.md`, then run:

```matlab
result = single_link_wheel_leg_lqr();
sim = single_link_wheel_leg_sim();
```

The exported firmware macro is written to `single_link_lqr_output.txt`.

The old `HGC_LQR_*.m` files are kept as reference only. The current robot is treated as a single-link wheel inverted pendulum, not a five-link VMC wheel-leg model.
