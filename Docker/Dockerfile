FROM ros:melodic-ros-base

# Install ROS tutorials for testing purposes
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-tutorials \
    ros-melodic-common-tutorials

# Install ROS_System_Monitor
ADD Scripts/install_ros_system_monitor /setup/install_ros_system_monitor 

RUN bash /setup/install_ros_system_monitor

# Install test scripts
ADD Scripts/talker_listener.launch /test/talker_listener.launch
ADD Scripts/run_ros_system_monitor_test /test/run_ros_system_monitor_test

ENTRYPOINT [ "bash", "test/run_ros_system_monitor_test" ]