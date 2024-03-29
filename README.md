# drive
## code
- The project is a typical ROS project.
- `src/` contains all source code.
## usage
- Open a terminal at project root (the directory containing this file).
- `roscore`
- `rviz`
- `source devel/setup.bash`
- `catkin_make && rosrun plan plan_node`
## demonstration

- Common legend

| Object | Description |
| --- | --- |
| White circle | Agent's circumcircle |
| Black line on agent  | Agent's orientation  |
| Yellow path  | Given path           |
| Corresponding pink paths   | Padding obstacles in configuration space  |
| Faint green circle around agent | Sensing region |
| Inner pink rectangles | Physical obstacle bounding boxes |
| Outer pink obstacles  | Configuration space obstacle bounding boxes |
| Green path | Output path |
| Red circle | Local target position of agent after path optimization |

- RRT\* legend

| Object | Description |
| --- | --- |
| Cyan rectangle | Sampling region |
| White tree | The RRT\* tree |

- Visibility graph legend

| Object | Description |
| --- | --- |
| White lines | Visibility graph |

- Straight path with 2 points.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/1.s2p-orrt.gif) | ![](github/1.s2p-vis.gif) |

- Straight path with 3 points.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/1.s3p-orrt.gif) | ![](github/1.s3p-vis.gif) |


- Straigh path with 5 points.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/1.s5p-orrt.gif) | ![](github/1.s5p-vis.gif) |

- Perpendicular turn with 3 points.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/2.903p-orrt.gif) | ![](github/2.903p-vis.gif) |

- Perpendicular turn with 5 points.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/2.905p-orrt.gif) | ![](github/2.905p-vis.gif) |

- Obstacles.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/3.obs-orrt.gif) | ![](github/3.obs-vis.gif) |

- Obstacles over points on given path.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/4.obs-on-rp-orrt.gif) | ![](github/4.obs-on-rp-vis.gif) |

- Complete block.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/5.obs-over-lane-orrt.gif) | ![](github/5.obs-over-lane-vis.gif) |

- Small passage.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/6.obs-small-passage-orrt.gif) | ![](github/6.obs-small-passage-vis.gif) |

- Shielding final point.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/8.cha1-orrt.gif) | ![](github/8.cha1-vis.gif) |

- Arc.

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/7.cir-orrt.gif) | ![](github/7.cir-vis.gif) |

- Arbitrary path (top down view).

| RRT\* | Visibility graph |
| --- | --- |
| ![](github/9.gen-orrt.gif) | ![](github/9.gen-vis.gif) |

- Arbitrary path (side view).

![](github/9.gen-orrt-vis-sideview.gif)

- RRT\* planning frequency vs number of sensed obstacles.

![](github/orrt_hz_obs.png)

- Visibility graph planning frequency vs number of sensed obstacles.

![](github/vis_hz_obs.png)

- RRT\* planning frequency vs number of sampled nodes.

![](github/orrt_hz_nodes.png)

