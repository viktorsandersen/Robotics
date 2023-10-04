# Lab 6: RRT-Connect

Consider the robotic scene “Kr16WallWorkCell” at itslearning in which you are asked to develop a
program for a robotic pick and place operation. You are supposed to pick up the object using the joint
configuration q_pick and place the object at the joint configuration q_place. You should use
RRT-connect to find a collision free path between the two joint configurations. Try to estimate the
parameter epsilon to optimize the performance w.r.t. the path length and the search time. Remember
that RRT-connect is a probabilistic method, so you need to do some statistics (You are short in time. Do the
best you can).
Some further details:

- q_pick = (-3.142,-0.827,-3.002,-3.143,0.099,-1.573) [rad]
- q_place = (1.571,0.006,0.030,0.153,0.762,4.490) [rad]
- The exercise must be solved using the project on itslearning which includes RobWork.
- Remember to grasp the bottle in your code.
- Use the workcell “Kr16WallWorkCell”.