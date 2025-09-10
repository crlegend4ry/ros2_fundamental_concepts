# ROS2 Fundamental Concepts
This project is part of a ROS2 examination designed to assess the understanding of core ROS2 concepts through a TurtleBot3 simulation in Gazebo. The exam is divided into three parts — Publisher/Subscriber, Service, and Action — each requiring the creation and use of ROS2 nodes in Python or C++.

**Part A:** Implements a <code>circle_publisher</code> node to send velocity commands for circular motion and an <code>odom_logger</code> subscriber node to read and log robot odometry.

**Part B:** Introduces a <code>square_service_server</code> that makes the robot move in a square when triggered, along with a <code>square_service_client</code> to call the service.

**Part C:** Features a <code>rotate_action_server</code> to rotate the robot by a specified angle with periodic feedback, and a <code>rotate_action_client</code> to send goals and monitor progress.

All components are structured within a clean ROS2 package, demonstrating practical use of topics, services, and actions.


## Objectives
To test understanding of ROS2 fundamental concepts such as **Publisher**, **Subscriber**, **Interface**, **Service**, and **Action** through the simulation software and implementation.


## Instructions
- Use ROS2 Python or C++.
- Each task will be tested in the TurtleBot3 Gazebo simulation.
- Code must be properly organized in a ROS2 package with clear node names.


### Section A: Publisher and Subscriber
- Task A1(Publisher): Create a node name "circle_publisher" that continuously publishes desire velocity commands <code>geometry_msgs/msg/Twist</code> to make Turtlebot3 move in a circle of radius ~0.5 meters.
  
- Task A2(Subscriber): Create a node name "odom_logger" that subscribes to <code>/odom</code> and prints:
  - Robot's position(x, y)
  - Robot's orientation(yaw angle) <br/>
  **Note: Please look up on keyword "Quaternion to Euler angle" and "yaw" angle <br/> is the heading angle of the robot.**
    
<br/>
<div align="center">
  <img width="966" height="412" alt="eulor_angle_conversion" src="https://github.com/user-attachments/assets/71098397-1dac-4bff-a654-2cc4ab71e065" />
  Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
</div>

### Section B: Service
- Task B1(Service Server): Create a service server node named "square_service_server" with service type <code>std_srvs/srv/Empty</code>.
  - When called, the robot should move in a square path with 0.5 meter per side using the velocity commands.
  - After finishing, the robot should stop.

- Task B2(Service Client): Create a client node "square_service_client" to call your created service.

### Section C: Action
- Task C1(Action Server): Create an action server node named "rotate_action_server" using a custom action definition <code>Rotate.action</code>:

<div align="center">
  <img width="778" height="247" alt="Image" src="https://github.com/user-attachments/assets/1b30c016-2022-496b-987e-ab4d5b921492" />
</div>
<br/>

<ul>
  The server should:
  <ul>
    <li>Rotate the Turtlebot3 in place by publishing <code>/cmd_vel</code>.</li>
    <li>Track the remaining angle of the robot for calculating the proper velocity through the simple P controller by the following concept:</li>
  </ul>
<ul>

<div align="center">
  <img width="883" height="203" alt="Image" src="https://github.com/user-attachments/assets/0a748a39-7e30-40d3-b022-c8047fb59118" />
</div>
<br/>

    - Publish feedback every 0.1 second (10 Hz).
    - Stop and succeed when finished.

- Task C2(Action Client): Create a client node "rotate_action_client" that:
  - Sends a goal angle (e.g., +3.14 radians or 180 degrees).
  - Prints feedback (remaining angle).
  - Prints the result when done: "Goal reached successfully" or "Goal aborted".