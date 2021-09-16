# uavcom
This package defines UAV communication system. It consists of **uav_com** and **com_sim** nodes.

### uav_com
This node defines UAV message transport system via ROS topics.  Each UAV suppose to launch this node. It has the *master* and the *slave* hardware network interfaces. *Slave* interface suppose to be located on the UAV top and can only be connected to the *master* interface.  *Master* interface creates communication environment, which suppose to be located on the UAV bottom. Bomber UAV type has only *slave* mode, and scout contains both. 
##### launching uav_com
The launch example you can find in "multi_uavcom.launch" and "uavcom.launch" files in launch catalog. 

There are two things that you suppose to do:
- namespace should be *master* or *scout*, which defines UAV type and a board number, which defines UAV it self (e.g. /scout12 or /bomber66)
- spawned model number should correspond to boardname in namespace
##### uav_com usage
To publish messages to a remote UAV topic (e.g. bomber3) from a local UAV instance (e.g. scout1) you suppose to publish messages in this topic form:  /from/uav_com/to/topicName (e.g. /scout1/uav_com/bomber3/mavros/setpoint_position/pose).

To subscribe to a remote UAV topic you should do exactly the same that was written above - subscribe to this topic form: /from/uav_com/to/topicName (e.g. /scout1/uav_com/bomber3/mavros/global_position/global).
### com_sim
This node is a communication environment simulation. It find all **uav_com** *masters* and *slaves* I/O interfaces and retranslate taking into account environment simulation.