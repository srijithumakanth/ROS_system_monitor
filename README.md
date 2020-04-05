# ``ROS-System-Monitor``

## `Overview`

A C++ program similar to " htop " , to display memory and CPU utilization by ROS process nodes.

The program also displays ROS logging console messages published by various ROS nodes and services running on the ROS master.

`Example Output:`

![System Monitor](images/ROS_system_monitor.png)

## `Demo`

### `System Requirements:`
1. [Docker](https://www.docker.com)
2. Linux - tested on Ubuntu Desktop 18.04.4
   * [Docker installation instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu)
3. Windows - tested on Windows 10 Pro version 1909
   * [Docker install download](https://www.docker.com/products/docker-desktop)
   * Docker configured to use Linux containers

## `Instructions`
1. Clone the project repository
```bash
    git clone https://github.com/srijithumakanth/ROS_system_monitor.git
```
2. Build Docker Image
```bash
    cd ROS_system_monitor
    cd Docker
    docker build --tag ros_system_monitor .
``` 
3. Run Docker container from image
   
   #### `Expected Behavior:`
   Container will start the ROS Master, pause for 10 seconds to allow the ROS Master and test nodes to come up, and then start ros_system_monitor.

   ROS node processor's and memory utilization will be displayed.

   Most recent ROS messages from the running nodes will be displayed. Number of messages displayed will depend on available screen space (Automatically set).

   Press `CTRL-C` to close ros_monitor.
```bash
    docker run -it --name ros_system_monitor_test ros_system_monitor
```

4. To clean up after testing
```bash
    docker rm ros_system_monitor_test
    docker rmi ros_system_monitor
```

## Make
This project uses [Make](https://www.gnu.org/software/make/). The Makefile has four targets:
* `build` compiles the source code and generates an executable
* `format` applies [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html) to style the source code
* `debug` compiles the source code and generates an executable, including debugging symbols
* `clean` deletes the `build/` directory, including all of the build artifacts

## Instructions

1. Clone the project repository: `git clone https://github.com/udacity/CppND-System-Monitor-Project-Updated.git`

2. Build the project: `make build`

3. Run the resulting executable: `./build/monitor`
![Starting System Monitor](images/starting_monitor.png)

4. Follow along with the lesson.

5. Implement the `System`, `Process`, and `Processor` classes, as well as functions within the `LinuxParser` namespace.

6. Submit!