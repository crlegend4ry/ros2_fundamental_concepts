# ROS2 Fundamental Concepts
This project is part of a ROS2 examination that tests the understanding of basic ROS2 concepts by using a robot simulation with TurtleBot3 in Gazebo. The test is divided into three parts: Publisher and Subscriber, Service, and Action. Each part checks if the user can build and use different types of ROS2 nodes in Python or C++.

In **Part A**, the user creates a publisher node called ***"circle_publisher"***, which sends speed commands to make the robot move in a circle. Also, a subscriber node called ***"odom_logger"*** reads the robot’s position and direction from the "/odom" topic and prints them.

In **Part B**, the user builds a service server called ***"square_service_server"***. When another node calls this service, the robot moves in a square shape and then stops. A service client node called ***"square_service_client"*** is also created to call this service.

In **Part C**, the user creates an action server called ***"rotate_action_server"***. This node rotates the robot by a given angle, sends feedback every 0.1 seconds, and stops when the robot finishes turning. A matching action client node, ***"rotate_action_client"***, sends the goal, shows the feedback(how much angle is left), and tells if the task was successful.

All the code must be placed inside a well-organized ROS2 package with clear and meaningful node names. This examination is designed to show the learner’s ability to apply ROS2 communication methods using topics, services, and actions in a practical situation.


## Objectives
To test understanding of ROS2 fundamental concepts such as “Publisher”, “Subscriber”, “Interface”, “Service”, and “Action” through the simulation software and implementation.


## Instructions
- Use ROS2 Python or C++.
- Each task will be tested in the TurtleBot3 Gazebo simulation.
- Code must be properly organized in a ROS2 package with clear node names.


### Section A: Publisher and Subscriber
- Task A1(Publisher): Create a node name “circle_publisher” that continuously publishes desire velocity commands (geometry_msgs/msg/Twist) to make Turtlebot3 move in a circle of radius ~0.5 meters.
  
- Task A2(Subscriber): Create a node name “odom_logger” that subscribes to “/odom” and prints:
  - Robot's position(x, y)
  - Robot's orientation(yaw angle)

### Section B: Service
- Task B1(Service Server): Create a service server node named “square_service_server” with service type “std_srvs/srv/Empty”.
  - When called, the robot should move in a square path with 0.5 meter per side using the velocity commands.
  - After finishing, the robot should stop.

- Task B2(Service Client): Create a client node “square_service_client” to call your created service.

### Section C: Action
- Task C1(Action Server): Create an action server node named “rotate_action_server” using a custom action definition “Rotate.action”.

  <img width="783" height="249" alt="Custom Action Definition" src="https://github.com/user-attachments/assets/668d4451-1180-4560-aeca-3afded29d9a6" />

  The server should:
  - Rotate the Turtlebot3 in place by publishing “/cmd_vel”.
  - Track the remaining angle of the robot for calculating the proper velocity through the simple P controller by the following concept:

    <img width="883" height="203" alt="Image" src="https://github.com/user-attachments/assets/0a748a39-7e30-40d3-b022-c8047fb59118" />

  - Publish feedback every 0.1 second (10 Hz).
  - Stop and succeed when finished.

- Task C2(Action Client): Create a client node “rotate_action_client” that:
  - Sends a goal angle (e.g., +3.14 radians or 180 degrees).
  - Prints feedback (remaining angle).
  - Prints the result when done: “Goal reached successfully” or “Goal aborted”.