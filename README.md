To run everything, use `ros2 launch ft_peception_synthesis py`.
Node parameters are controlled using YAML config files in the `config` directory of the `ft_perception_synthesis` package rather than as launch arguments.

You will probably need to change the parameters of the segmentation node.
Currently it uses camera #1 by default and it looks for weights in `$HOME/projects/<model>.ts`.

As config files are unique to nodes, it would be fine for your config file to look like the config file below, which is an example of what your segmentation node config file might look like.

```
/**: # Match all nodes (will only ever match a single node)
	ros__parameters:
		model: <other model>
		weights: <new path>
```

Here I use the `/**` operator rather than the actual node path.

# Pretrained Weights

See releases page.

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
