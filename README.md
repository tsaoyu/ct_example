## CT example

This repository contains a minimal example to implement advanced controller in [control toolbox](https://github.com/ethz-adrl/control-toolbox) with ROS. Most of material is derived from the official tutorial expect that the main objective begin showing the basics in design a controller for ROS integration close to real application. 


### Pre-requisite 

Control toolbox need to be installed and sourced first. 

Optional: [rqt_multiplot](https://github.com/ANYbotics/rqt_multiplot_plugin)

### Control problem

The problem used in this example is a simple second order system dynamics control. A physical example of the such system can be a damped oscillator shown in below.

![](/images/system.png)

Dynamics of the system follows the following ODE and velocity and position of the frictionless car are the state of the system. 

![](/images/system_ode.png)

where y is the position, y dot is the velocity of the car. 

![](/images/system_ode_terms.png)

The state space model of the system is 

![](/images/system_ss_model.png)

with the parameter of k_{dc} = 1, omega_n = 0.1 and zeta = 0.5. The control problem is to track the set point x_ref with optimal control u. 




### Example 1: Create a dynamic simulator

`oscillator_dynamics.cpp` shows how to create a simulator in ROS environment for the control problem, allows the user to control the system via joystick. Run the example with: 

```
roslaunch ct_example oscillator_dynamic_manual.launch
```
You can use any `rosjoy` compatiable joystick controller to manuipulate the system. To view current state of the system try `rostopic echo /position` in command line or `rqt` for visulisation. Two configuration files `multiplot.xml` and `view.perspective` are provided under `rqt_settings` folder.

![](/images/rqt_view.png)




### Example 2: LQR controller 
`oscillator_lqr_controller.cpp` demonstrates the design of LQR controller. The optimal control problem is to find the control input u to minimise the total cost.

```
roslaunch ct_example oscillator_lqr_control.launch
```

You can play around by publishing different reference position in `rqt`. Note that the `x` in the message represents the postion while `y` in the message represents the velocity of the car.

![](/images/lqr_view.png)



### Example 3: iLQR controller

`oscillator_ilqr_controller.cpp` illustrates the implementation of ILQR controller. 

```
roslaunch ct_example oscillator_ilqr_control.launch
```
Similarly, you can change the reference point in the `rqt` GUI.

### Example 4: MPC controller

The final example `oscillator_mpc_controller.cpp` is the Model Predictive Controller for the damped oscillator system.

```
roslaunch ct_example  oscillator_mpc_control.launch
```

