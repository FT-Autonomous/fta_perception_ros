# Installation

First pull the weights and the bag using this command:

```
sh get-resources
```

## Build

```
colcon build
```

If your device doesn't support cuda, use the build command

```
colcon build --base-paths src/{ft_perception_synthesis,segmentation,cluster,center_estimation,perception_msgs}
```

# Running

To run everything, use `ros2 launch ft_peception_synthesis py`.

# Bags

Before building, download the bag from slack/github and then extract it in the folder `src/ft_perception_synthesis/bags/static`.

# Pre-Cluster Downsampling

Currently, the c++ clustering algorithm is not used.
It does not much of an advantage over the python implementation since the main problem is the fact that the algorithm is `O(n^2)` where `n` is the number of points belonging to a particular cone class.
We are currently writing a clustering algorithm which will paralellise the most expensive part of the algorithm on the GPU.
This should give us an algorithm that has `O(n)` complexity, or `O(n/b)` where `b` is the batch size. 

The temporary solution to this is to downsample the depth map before passing it into the clustering algorithm.
I have found a `downsample_factor` value (parameter in the perception node) of 5 combined with a `min_samples` (parameter in the cluster node) value of 10 to be sufficient for debugging purposes.


[![test]][the_rest]

[test]: uml.jpg
[the_rest]: ?
