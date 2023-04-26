# Agilex Fleet Client

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS1 - Noetic](https://wiki.ros.org/noetic)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget qt5-default \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  # maven default-jdk   # Uncomment to install dependencies for message generation
```

### Client in ROS 1

Start a new ROS 1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/agx_fleet_ws/src
cd ~/agx_fleet_ws/src
git clone https://github.com/lagrangeluo/agx_fleet_client.git
```

Install all the dependencies through `rosdep`,

```bash
cd ~/agx_fleet_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -yr
```

Source ROS 1 and build,

```bash
cd ~/agx_fleet_ws
source /opt/ros/noetic/setup.bash
colcon build
```

roslaunch a client for navigation action server /move_base,

```bash
roslaunch agx_fleet_client_bringup agx_fleet_client.launch
```

**attention:** if your move base action server name is not "move_base", you should change the param of launch file as follows:

```xml
agx_fleet_client.launch:
    <param name="move_base_server_name" value="move_base"/>
```

if you want to specify robot name and fleet name, change param:

```xml
<arg name="agx_robot_prefix" value="limo_1"/> 
...
	<param name="fleet_name" type="string" value="agx_fleet"/>
```





## Examples

### Barebones Example

This example emulates a running ROS 1 robot,

```bash
source ~/ff_ros1_ws/install/setup.bash
roslaunch ff_examples_ros1 fake_client.launch
```

This example emulates a running ROS 2 robot,

```bash
source ~/ff_ros2_ws/install/setup.bash
ros2 launch ff_examples_ros2 fake_client.launch.xml
```

The clients will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. Start the server using

```bash
source ~/ff_ros2_ws/install/setup.bash
ros2 launch ff_examples_ros2 fake_server.launch.xml

# Verify that the server registers the fake clients
# [INFO] [1636706176.184055177] [fake_server_node]: registered a new robot: [fake_ros1_robot]
# [INFO] [1636706176.184055177] [fake_server_node]: registered a new robot: [fake_ros2_robot]
```

ROS 2 messages over the `/fleet_states` topic can also be used to verify that the clients are registered,

```bash
source ~/ff_ros2_ws/install/setup.bash
ros2 topic echo /fleet_states
```

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).

### ROS 1 Turtlebot3 Simulation

Before starting these examples, remember to install all the prerequisites according to the [official tutorials](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) of using `Turtlebot3`, under `noetic`.

```bash
sudo apt install ros-noetic-dwa-local-planner
```

Launch the basic simulation of a single Turtlebot3, with a free fleet client attached to it, by sourcing the ROS 1 workspace and launching the provided example launch file,

```bash
source ~/ff_ros1_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger; roslaunch ff_examples_ros1 turtlebot3_world_ff.launch
```

This launch file starts the simulation in `gazebo`, visualization in `rviz`, as well as the simulated navigation stack of the single turtlebot3. Once the simulation and visualization show up, the robot can be commanded as per normal through `rviz` with `2D Nav Goal`.

If the server is already running, it should display that a new robot has been registered.

```bash
[INFO] [1636706001.275082185] [turtlebot3_fleet_server_node]: registered a new robot: [ros1_tb3_0]
```

Another way to check, is to listen in on the `/fleet_states` topic, using `ros2 topic echo /fleet_states`.

Next, to send requests and commands, check out the example scripts and their uses [here](#commands-and-requests).
