# Installation

First clone this repository using this command:

```
git clone --recursive <reponame>
```

Then pull the weights and the bag using this command:

```
sh get-resources
```

## Dependencies

## ft_msgs

Available in the FT-Autonomous repo [here](https://github.com/FT-Autonomous/ft_msgs/)

### C++

This module depends unconditionally upon:
- geometry_msgs (ros)
- sensor_msgs (ros)
- OpenCV

To use the ZED camera rather than the prerecorded bags, you need the [ZED SDK](https://www.stereolabs.com/developers/release/), which requires CUDA.

To use the c++ clustering algorithm, you need a version of OpenGL that supports compute shaders as the c++ clustering paralellises a significant amount of the workload on the GPU.

If you are using X11, you can query your OpenGL version using:

```
glxinfo | grep -i 'Max opengl core profile'
```

### Python

```
pip3 install -r requirements.txt
```

# Build

If your device does support cuda, first delete the ignore file in the zed_capture folder.

```
rm src/zed_capture/COLCON_IGNORE
```


The run the followng:

```
colcon build
```

If your device doesn't support cuda, just use the build command directly.

```
colcon build
```

# Source Underlay and Overlay

```
. install/setup.sh
```

# Running

To run everything, use `ros2 launch ft_peception_synthesis py`.

# Bags

Before building, download the bag from slack/github and then extract it in the folder `src/ft_perception_synthesis/bags/static`.

[![test]][the_rest]

[test]: uml.jpg
[the_rest]: ?
