# Hopf Osicllator based snake robot controller
This project was submitted as a final project towards EN.605.716. It simulates coupled Hopf oscillators as described in [1] to control a snake robot.
## Requirements
* ROS Kinetic 

* [OakLake's snake robot gazebo simulation](https://github.com/OakLake/SnakeBot_ROS) (optional)

## Running

To run the simulation using Oaklake's snake: 

`roslaunch hopf_snakebot sim.launch`

Otherwise, you will have to change the topic parameter in `nodes/controller` to match the Float32 topic that controls each joint. Then run:

`rosrun hopf_snakebot controller`

## References
Wang, Z., Gao, Q., & Zhao, H. (2017). CPG-Inspired Locomotion Control for a Snake Robot Basing on Nonlinear Oscillators. Journal of Intelligent and Robotic Systems: Theory and Applications, 85(2), 209–227. https://doi.org/10.1007/s10846-016-0373-9
