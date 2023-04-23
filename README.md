# TURTLEBOT3 GAZEBO CONTROLLER

## PREREQUSITES

You should have Gazebo installed through a standard installation on Ubuntu (I used Jammy Jellyfish 22.04). Also ROS2 and the ROS2 packages connected to turtlebot3. I've files and folders with turtlebot3 inside this repo but it perhaps may be easier for you to install turtlebot3 from their official github repository and just use py_pubsub folders from this repo.

## NODES

In this project I've created simple nodes that are comprised of a publisher, a subscriber and a controller that uses both these concepts.
The controllers job is to make the turtlebot go to the designated position on an XY grid specified in the controller.py main function.

## USEFUL COMMANDS

After you have successfully installed gazebo on your OS, you might want to use the command:

```Bash
~/folder_with_this_repo $ ros2 launch turtlebot_3_gazebo empty_world.launch.py
```

Then open a new terminal and either navigate to a ~/folder_with_this_repo/src/py_pubsub/py_pubsub/ and then use controller.py or publisher_member_function.py or subscriber_member_function.py:

```Bash
py_pubsub $ python3 controller.py 
```

Or build the ROS2 packages with colcon build (if there are any issues run: $ rosdep update) and then run it as a ROS2 node:

```Bash
ros2 run py_pubsub controller
```

## FINAL DESCRIPTION

This exercise was part of the course called Mobile Robotics at my Uni, made with 2 other students.
To see the project in action watch any of the following yt videos:

Controller:

[Controller YT video](https://www.youtube.com/watch?v=dAE_x_FUPx0)

Publisher and Subscriber:

[Pub&Sub YT video](https://www.youtube.com/watch?v=oguTRrokDfI)
