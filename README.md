# ROS 2 Performance issues

This repo contains a ROS2 package with a simple node that creates heavy publishers

Running it extensivelty utilizes the CPU, and allows performing a comparison between different operation modes (RMW/DDS etc.)

# Comparison with ROS 1

Docker image can be built for ROS 1 (Noetic) and compared against as well

# Usage

Parameters can be adjusted in the corresponding launch files

## ROS 2

Run separate nodes:
```
ros2 launch yo_exp2 main.launch.py
```

Run composed nodes:
```
ros2 launch yo_exp2 composed.launch.py
```
(The amount of nodes can be adjusted as well)


## ROS 1

```
docker build -t yo_exp1 .
docker run -it --rm --name ros1-yo-exp yo_exp1
```

In another terminal:

```
docker exec -it ros1-yo-exp bash
launch
```

### Monitoring the CPU usage
```
monitor.py  <executable-name>  <log-file-suffix>
```

Executable name for main launch: `comp_sub_node`

Executable name for composable launch: `component_container`
