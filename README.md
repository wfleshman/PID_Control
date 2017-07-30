# PID-Control of OpenAI LunarLander-v2 (continuous)

## Dependencies
````
gym version 0.9.2
numpy 1.13.1
matplotlib 2.0.0
````

## PID Control
From [wikipedia](https://en.wikipedia.org/wiki/PID_controller):
A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism (controller) commonly used in industrial control systems. A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (sometimes denoted P, I, and D respectively) which give their name to the controller type.

## LunarLander
[OpenAI](https://gym.openai.com/envs/LunarLanderContinuous-v2)

This is an environment from OpenAI gym. The goal is to land the craft safely between the goal posts. 

At each timestep the craft has access to its current state which consists of the x,y coordinate, x,y velocity, angle and angular velocity, and a touch sensor on each leg. 

Also at each step, the craft will be given an action from the control loop. The action is a 2-vector corresponding to the thrust output of the main booster, and the thrust output of the side boosters. 

## PID Solution
For this environment, the process variables to be controlled are the amounts of thrust to the main and secondary boosters, which in normal situations are respectively used for controlling the altitude and angle of the craft. 

The first step in PID control is figuring out what the setpoints should be. 

I decided my target altitude for a given state would be equal to the horizontal distance from the target as shown below. Therefore, if I was too low and off-target the craft should increase it's height back into the cone, and when the craft is above the cone it should decrease it's height. So my altitude setpoint is equal to:
````
|x_position|
````
![Altitude Setpoint](/imgs/altitude.png)

Similarly, the craft should always be angled toward the goal since the angle drives what direction the main boosters will send the craft. I figured if I was above the target I would want a perfect vertical angle, and if I was at the extreme edges I would want 45 degrees. 
So my angle setpoint is:
````
pi/4 * (x_position + x_velocity)
````
![Angle Setpoint](/imgs/angle.png)

The altitude and angle (as well as the velocities for each component) are known at each step. Therefore, the error can be calculated as the difference between our setpoints and the current altitude/angle. 

To add proportional control to our control system, we adjust the boosters by some scalar multiple of the errors(i.e. a correction proportional to the error).
```` 
# P-Control
Adj = k_p*error
````

To add derivative control we add the derivative of each control value to our adjustment. The derivative of position (the variables we're trying to control for) is velocity. Luckily the y velocity and angular velocities are given to us as part of the sensor readings in our state. We add the term to our adjustment with an additional parameter.
````
PD-Control
Adj = k_p*error + k_d*derivative
````
The final component of a PID controller is the Integral or sum of errors. This portion of a controller corrects for any biases in the system (like a misaligned component, or uncalibrated sensor). Since we have 100% sensor values in this problem the I component is not needed for the solution. 

So we have everything we need, except that we don't know the values to use for the four parameters (k_p/k_d for both altitude and angle). Tuning parameters by hand can be difficult and time consuming, so I used a random optimization technique called hill climbing which is very simple. Essentially, we start off by assuming the parameters should be all 0 (no control). We then try to land the rocket and observer our score. Then at each step we alter the parameters by some small and random amount. If the rocket gets a better score we keep the new values and repeat. Otherwise we throw out the new values and try adding random noise again. This technique allows the optimizer to slowly move the values of the parameters "up the hill" toward a solution. 

With the optimized values the rocket is able to land safely, and we can see that our PID controller is doing what we want:
![PID Controlled](/imgs/pid.png)
![Demo](/imgs/lander.gif)

