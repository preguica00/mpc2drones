# mpc2drones


This example presents two quadcopter models controlled by an MPC controller. To run the program, use `simulate.m`.

The MPC controller corresponds to the optimize_trajectory.m.
The controller is implemented with fmincon, which has a cost function and a corresponding nonlinear constraints function 
called `discretization.m`. This last function uses `timestep_euler.m` to implement the dynamics constraint.

The `quadcopter_ode.m` corresponds to the models equations.
The simulation is made with the `simulate_timestep.m` that uses the `quadcopter_ode.m`

To see a plot of the states and control use `simulate_states.m`, that runs states.mat and control.mat.
