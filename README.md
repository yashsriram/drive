# drive
## description
- Real-time algorithms for following a given path while avoiding initially unknown static obstacles.
- Uses RRT\* and Visibility graph along with padding obstacles.
## roadmap
- Problems solved until now are documented in `report.pdf`
## code
- The project is a typical ROS project.
- `src/` contains all source code.
## documentation
- For most of the code, the documentation is itself.
## usage
- Open a terminal at project root (the directory containing this file).
- `roscore`
- `rviz`
- `source devel/setup.bash`
- `catkin_make && rosrun plan plan_node`
## demonstration

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
| ![](github/7.cir-orrt.gif) | ![](github/7.cir-orrt.gif) |

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

