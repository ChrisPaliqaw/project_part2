# project_part2

Moves an RB1 robot into a cart using laser and odometry data. It demonstrates the concepts of
[The Construct](https://www.theconstructsim.com/)'s <ins>ROS2 Basics C++</ins> course. As of June 26, 2022, I'm still adding features.

![RB-1 Robot moving toward a cart](rb1_warehouse.png "RB-1 Robot moving toward a cart")

Concepts:
- Components (no main function)
- Multithreaded programming using std::lock_guard, but I plan to implement the next step using a multi-threaded executor with a revenant callback group.
- Subscription and publication of topics
- A ROS2 parameter adapts the algorithm to either Gazebo or a real RB1 in the Construct's [RoBox](https://www.theconstructsim.com/robox/) lab.
