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

---------

## ROS2 basics:
- System configuration: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
- **note:** ros version depends on ubuntu version!, more: [Unable to locate package ros foxy desktop](https://answers.ros.org/question/402597/unable-to-locate-package-ros-foxy-desktop/?answer=409572#post-id-409572)
- A good starting point is the [turtlesim](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

using rqt to:
- change values: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
- viewlogs: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html


### Nodes
> *A node is a fundamental ROS 2 element that serves a single, modular purpose in a robotics system.*
>
> *The ROS graph is a network of ROS 2 elements processing data together at one time. It encompasses all executables and the connections between them if you were to map them all out and visualize them.*
>
> *Each node in ROS should be responsible for a single, module purpose (e.g. one node for controlling wheel motors, one node for controlling a laser range-finder, etc). Each node can send and receive data to other nodes via topics, services, actions, or parameters.*
>
> ![Each node in ROS should be responsible for a single, module purpose](https://docs.ros.org/en/foxy/_images/Nodes-TopicandService.gif)
>
> - list node names: `ros2 node list`
> - to reassign default node properties, like node name, topic names, service names, etc., to custom values use [remapping](https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules): `--remap`, example: `ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle`
> - get information about the node: `ros2 node info <node_name>`
>
> tutorial with *turtlesim*: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#ros2-node-list

### Topics
> *Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system. Nodes send data over topics using messages. Publishers and subscribers must send and receive the same type of message to communicate.*
> 
> | ![singel subscriber-publisher relation](https://docs.ros.org/en/foxy/_images/Topic-SinglePublisherandSingleSubscriber.gif) | ![many to many realtions](https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif) | 
> | ---- | ---- |
> | ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages. | A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics. |
>
> - to visualise current topic subscriptions on nodes: `rqt_graph`
> - command line tools:
>   - list of all the topics currently active in the system: `ros2 topic list`
>   - list of all the topics type appended in brackets:`ros2 topic list -t`
> - see the data being published on a topic: `ros2 topic echo <topic_name>` *this command won’t return any data. That’s because it’s waiting for `<topic_name>` to publish something.*
> - see how many publishers/subscriptions are on the topic: `ros2 topic info <topic_name>`
> - to learn what structure of data the message expects: `ros2 interface show <msg type>`
> - publish data onto a topic directly from the command line using: `ros2 topic pub <topic_name> <msg_type> '<args>'`
>   - *this argument needs to be input in YAML syntax, ex: `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`* 
>   - *argument to publish one message then exit: `--once`*
>   - *to publish the command in a steady stream at 1 Hz use `--rate 1` option, ex: `ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`* 
> - view the rate at which data is published: `ros2 topic hz <topic_name>`
>
> tutorial with *turtlesim*: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#tasks

### Services
> *Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model, versus topics’ publisher-subscriber model. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.*
>
> *Services have types that describe how the request and response data of a service is structured. Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.*
>
> *Nodes can communicate using services in ROS 2. Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.*
>
> ***You generally don’t want to use a service for continuous calls; topics or even actions would be better suited.***
>
> | ![services](https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif) | ![service responses](https://docs.ros.org/en/foxy/_images/Service-MultipleServiceClient.gif) |
> | --- | ---- |
>
> - a list of all the services currently active in the system: `ros2 service list`
> - type of a service: `ros2 service type <service_name>` 
>   - service call sends no data when making a request and receives no data when receiving a response: `Empty`
> - see the types of all the active services: `ros2 service list -t`
>   -  you can append the option: `--show-types`, `-t`, `list`
> -  all the services of a specific type: `ros2 service find <type_name>`
> - to know the structure of the input arguments: `ros2 interface show <type_name>.srv`
> - call a service: `ros2 service call <service_name> <service_type> <arguments>`. *Note: The `<arguments>` part is optional. ex: `ros2 service call /clear std_srvs/srv/Empty`*
>
> tutorial with *turtlesim*: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html#tasks

### Parameters
> *A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as `integers`, `floats`, `booleans`, `strings`, and `lists`. In ROS 2, each node maintains its own parameters.* [more...](https://docs.ros.org/en/foxy/Concepts/About-ROS-2-Parameters.html)
>
> Nodes have parameters to define their default configuration values. You can *get* and *set* parameter values from the command line. You can also save the parameter settings to a file to reload them in a future session.
> 
> - see the parameters belonging to your nodes: `ros2 param list`
>   - *Every node has the parameter `use_sim_time`* 
> - display the type and current value of a parameter: `ros2 param get <node_name> <parameter_name>` *ex: `ros2 param get /turtlesim background_g`*
> - change a parameter’s value at runtime: `ros2 param set <node_name> <parameter_name> <value>`
> -  view all of a node’s current parameter values: `ros2 param dump <node_name>` *Dumping parameters comes in handy if you want to reload the node with the same parameters in the future.*
> - load parameters from a file to a currently running node: `ros2 param load <node_name> <parameter_file>`
> - start the same node using your saved parameter values: `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>`
>
> tutorial based on *turtlesim*: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#tasks

### Actions
> Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a *goal*, *feedback*, and a *result*.
> 
> *Actions are built on topics and services. **Their functionality is similar to services, except actions can be canceled.** They also provide steady feedback, as opposed to services which return a single response.*
>
> *Actions use a client-server model, similar to the publisher-subscriber model. An `“action client”` node **sends a goal** to an `“action server”` node that acknowledges the goal and **returns a stream of feedback** and a result.* Actions have types, similar to topics and services.
>
> ![action client-server connection](https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif)
>
> *Not only can the client-side (your input in the teleop) stop a goal, but the server-side can as well. When the server-side chooses to stop processing a goal, it is said to “abort” the goal.*
>
> Actions are like services that allow you to execute long running tasks, provide regular feedback, and are cancelable.
>
> A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.
>
> This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. ***Don’t assume every action server will choose to abort the current goal when it gets a new one.***
>
> - see node’s actions: `ros2 node info /<node_name>`
> - identify all the actions in the ROS graph, run the command: `ros2 action list`
> - find actions's type: `ros2 action list -t` *In brackets to the right of each action name is the action type. You will need this when you want to execute an action from the command line or from code.*
> - You can further introspect the action: `ros2 action info /<action_name>` *this will list clients and servers*
> - the structure of the action type: `ros2 interface show <interface_name>` *the detailed description of the values in the action*
> - send an action goal from the command line: `ros2 action send_goal <action_name> <action_type> <values>` *ex: `ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"`*
>
> tutorial based on *turtlesim*: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html#tasks

### Launching nodes
> *Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously. Running a single launch file with the `ros2 launch` command will start up your entire system - all nodes and their configurations - at once.*
>
> Launch files caould be written in [Python or in XML, YAML](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html)
>
> more about launch files: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
>
> *Once you learn to write your own launch files, you’ll be able to run multiple nodes - and setup their configuration*
> 
> - start ros2 packages: `ros2 launch turtlesim launch.py`
>
> tutorial based on *turtlesim*: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html#tasks

### Recording data - `ros2 bag`
> A command line tool for recording data published on topics in your system. It accumulates the data passed on any number of topics and saves it in a database. You can then replay the data to reproduce the results of your tests and experiments. Recording topics is also a great way to share your work and allow others to recreate it. To view the individual messages, you would have to open up the database, in this case sqlite3, to examine it, which is beyond the scope of ROS 2.
>
> Install it:
> ```
> sudo apt-get install ros-humble-ros2bag \
>                      ros-humble-rosbag2-storage-default-plugins
> ```
>
> `ros2 bag` can only record data from topics that are published on.
> - see a list of your system’s topics: `ros2 topic list`
> - To record the data published to a topic: `ros2 bag record <topic_name>`
> - see details about your recording: `ros2 bag info <bag_file_name>`
> - replaying the bag file: `ros2 bag play <bag_file_name>`
>
> tutorial based on *turtlesim*: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html#tasks
