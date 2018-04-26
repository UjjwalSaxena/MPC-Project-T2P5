## Model predictive control project

### Overview:

Model predictive control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints. It reframes the task of following a trajectory to a cost optimization problem. We define the costs of not maintaining a reference velocity, the cost of cross track error etc. and add this up to the total cost the car has to minimize. This cost is minimized once the car is in optimal driving position. MPC has the ability to anticipate future events and take actions accordingly.

### MPC Algorithm:

There are two steps in an MPC algorithm.
1. Initializing the error cost.
2. Minimizing it in continuously according to the new waypoints.

The msgic is done by the solver which minimizes the cost of the trajectory. It takes in all the constraints and returns the set of control actions needed to be prformed.

### State variables:

 **x:** x coordinate of the current location of the car in the global map coordinate system.
 
 **y:** y coordinate of the current location of the car in the global map coordinate system.
 
 **psi:** orientation of the car or the angle of the car from x axis in the global map coordinate system.
 
 **v:** velocity of the car. 
 
These are given as inputs to the MPC algorithm. In addition to this, we are also given a series of waypoints which are points with respect to an arbitrary global map coordinate system we can use to fit a polynomial which is a function that estimates the curve of the road ahead. We can use any order polynomial to fit to the curve of the road but using 3rd order polynomial is beneficial as it generally approximates the road curves better.

### Errors:

**CTE:** Cross track error is the difference between the actual and the desired position of the car.

**EPSI:** Refers to the error in psi or the orientation. This is the arctan of the derivative of the fitted polynomial.

### Actuations:

**delta:** This is the steering angle, the car turns after each timestep. For Sanity check this angle is restricted to +25 and -25 degrees. I have converted this value to values between +1.0 and -1.0 because the simulator expects the values in this range.

**a:** This is the throttle or the acceleration value which is also restricted between +1 and -1. Here negative throttle values are analogous to brakes.

### Next state equations:

    px` = px + v * cos(psi) * dt
    py` = py + v * sin(psi) ( dt)
    psi` = psi + v / Lf * (-delta) * dt
    v` = v + a * dt
    
### Next state errors:

    cte` = cte - v * sin(epsi) * dt
    epsi` = epsi +  v / Lf * (-delta) * dt

**Note: Lf is distance of the front of vehicle from its Center-of-Gravity**


### Error costs:

Error costs are the penalty numeric values given to the difference in the actual and desired values of various parameters. 

      fg[0] += CppAD::pow(vars[cte_start + t], 2)*a;
      fg[0] += CppAD::pow(vars[epsi_start + t], 2)*b;
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2)*c;
      fg[0] += CppAD::pow(vars[delta_start + t], 2)*d;
      fg[0] += CppAD::pow(vars[a_start + t], 2)*e;
      fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2)*f;
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2)*g;
      
a,b,c,d,e,f and g are the constant weights here. These are used to reduce or increase the direct dependency of the cost on the square of the error.
I have taken these values to be:
**a:20

**b:20**

**c:10**

**d:400**

**e:10**

**f:350**

**g:15**

These values are taken after manual hit and trial so that the car follows the desired trajectory properly.

### Time step and prediction length:

prediction length or the number of prediction to be made in advance, N, is set to 13. That means I will predict 13 future positions of the car in advance.
Time step or dt is set to 0.1 which means that we want to predict these N future positions at a gap of 0.1 seconds.


### Dealing with Latency:

here we know that the system has a latency of 100 ms which means that the command given now will be executed after 100 ms.
This can also be interpreted as if by the time actuators will respond to the commands it will already be some future state of the car and not the current state actually. Hence we initialize the values with next future state in advance and then predict next N number of future positions.

since dt is also taken as 0.1 ms, the first next predicted future position more or less corresponds to the current position of the car.

          double x_t1 = 0+ v*dt;
          double y_t1 = 0.0;
          double psi_t1 = - v*steer_value*dt/Lf;
          double v_t1 = v + throttle_value*dt;
          double cte_t1 = cte + v*sin(epsi)*dt;
          double epsi_t1 = epsi + psi_t1;
          // cout<<"cte: "<<cte<<endl;

          state << x_t1, y_t1, psi_t1, v_t1, cte_t1, epsi_t1;
          mpc.Solve(state, coeffs); 
Also to simulate the same effect it might have had in the actual environment, thread was made to sleep 100 ms before sending the control command.
          
### Conclusion:
The car was able to drive well, also with the latency taken into account.




      

