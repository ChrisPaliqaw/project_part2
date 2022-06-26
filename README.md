# project_part2

Moves an RB1 robot into a cart using laser and odometry data. It demonstrates the concepts of
(The Construct)[https://www.theconstructsim.com/]'s ROS2 Basics C++ course.

![RB-1 Robot moving toward a cart](rb1_warehouse.png "RB-1 Robot moving toward a cart")

Concepts:
- Components
- Multithreaded programming using std::lock_guard, but I plan to migrate it to a multi-threaded executor with a revenant callback group.
- Subscription and publication of topics
- A ROS2 parameter adapts the algorithm to either Gazebo or a real RB1 in the Construct's [RoBox](https://www.theconstructsim.com/robox/) lab.
