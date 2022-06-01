In order to run both the video capture node and the image segmentation node, use `ros2 run ft_peception_synthesis synthesis.launch.py`.
Node parameters are controlled using YAML config files in the `config` directory of the `ft_perception_synthesis` package rather than as launch arguments.

You will probably need to change the parameters of the segmentation node.
Currently it uses camera #1 by default and it looks for weights in `$HOME/projects/<model>.pth`.

As config files are unique to nodes, it would be fine for your config file to look like the config file below, which is an example of what your segmentation node config file might look like.

```
/**: # Match all nodes (will only ever match a single node)
	ros__parameters:
		model: <other model>
		weights: <new path>
```

Here I use the `/**` operator rather than the actual node path.

# Pretrained Weights

* [CGNet (~3mb)](https://naza.uzoukwu.net/files/fta/cgnet.pth)
* [ICNet (~113mb)](https://naza.uzoukwu.net/files/fta/icnet.pth)

[![test]][the_rest]

[test]: uml.jpg
[the_rest]: ?
