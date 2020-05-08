# PID_Controller_Car

![](gif/pid_controller.gif)

## Installation:

The [CarND-PID-Controller](https://github.com/AndresGarciaEscalante/PID_Controller_Car/tree/master/CarND-PID-Control-Project) README file in the repository provides with all the installation steps to ***download and execute the simulator***. 

Once the simulator is installed follow the next steps to run the code:

* In the  ***CarND-PID-Control-Project*** execute the following commands:

```
$ mkdir build && cd build
$ cmake .. && make
$ ./pid
```

* In another terminal, go to the folder where the simulator is located and then run the following commands:

```
$ mkdir chmod +x term2_sim.x86_64
$ ./term2_sim.x86_64
```

* A new windows should pop up and select the path planning project.

* As soon as the simulation starts the car must move autonomously.

## Project Description
Implement a PID controller for self-driving car's steering angle that allows it to drive safely and smoothly in a highway in a simulated 3D environment. The PID coefficients must be tuned to achieve the desired behavior.

The following functions were implemented to control the steering of the car:

### Velocity Controller
For the purpose of this project the ***velocity of the car*** will remain constant at ***30 mph***. The only ***exception*** is when the ***angle of the steering is greater than 10 degrees***.

### P coefficient and error
The P error allows the ***present error value*** to make the output signal stabilize , however, this might not neccesary means that the output will be at the reference value. Large amounts of coeffient leads to oscillations.  

### I coefficient and error
The I error allows to look at the ***past error values of the system*** and with this solve the problem of the ***bias*** which is a offset from the target value. The coefficient in most of the cases tends to be small.

### D coefficient and error
The D error allows to look at the ***future error values of the system*** with the derivative we can predict the behaviour. Large values for the coefficient leads to abrupt changes in the output.

### Manually tune the PID coefficients
In order to make the twiddle algorithm work we need first a good initial estimate of the PID coefficients. In this case, it was done by applying the concepts described above and manually incerting values to the parameters. The P parameter was tuned first, then P and D parameters, and finally the PID parameters together. The values were chosen by increasing (multiply) or decreasing (divide) the current parameter's value by 10.

### Twiddle PID coefficients
Once the the initial coeffients were set we let the algorithm tune the parameters. The algorithm starts with the given values and (adds/subtracts) them an offset value ***dp***, after that the new value is executed in the simulator and the average error is stored and compared with the ***current_best_error***. The code in python of the algorithm is shown bellow:

```
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
```

## Project Outcome
The car was able to drive autonomously without collading and going outside the boundaries of the highway.

***Important:*** Check the full video of the [PID_Controller_Project](https://www.youtube.com/watch?v=UmNsTYCOG4M&t=45s)