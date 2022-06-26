# project_part2

A simple program to move an RB1 robot into a cart using laser and odometry data. It demonstrates the concepts studied in
(The Construct)[https://www.theconstructsim.com/]'s ROS2 Basics C++ course.

It demonstrates
- Components
- Multithreaded programming using std::lock_guard, but I plan to migrate it to a multi-threaded executor with a revenant callback group.
- Subscription and publication of topics
- Uses a ROS2 parameter to adapt to either gazebo or a real RB1 in the Construct's [RoBox](https://www.theconstructsim.com/robox/) lab.
