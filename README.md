# Programming robots in ROS
> ROS introduction, setting up the development environment. Implement ROS packages in Python. Basic ROS communication, implementing publishers and subscribers. Principles of robotics, programming a simulated robot in joint and workspace. Roslaunch, ROS parameter server. Acquisition and processing of sensory data in ROS. Programming da Vinci surgical robot in simulated environment. Programming humanoid robot. In simulated environment. Define custom messages. ROS service and ROS action.

10 years of ROS: https://www.youtube.com/watch?v=mDwZ21Zia8s

## ROS basics:
- The ROS toics can handle only one variable type. To get thy type: `rosmsg show <topicname>`.
- List available ROS message types: `rosmsg list`
  - a message file extension is `.msg`
- see current ROS topics: `rostopic list`
- start ROS: `roscore`
- record ROS `rosbag record --all`
- play ROS `rosbag play <myfile>.bag`
- launch file `roslaunch <myfile>`
- action files are: `.action`
