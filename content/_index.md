+++
+++

- Real-time algorithms for following a given path while avoiding initially unknown static obstacles.
- Uses RRT\* and Visibility graph along with padding obstacles.

<img src="github/9.gen-orrt-vis-sideview.gif" width="100%"/>

| RRT\* planning frequency vs number of sensed obstacles. | Visibility graph planning frequency vs number of sensed obstacles. |
| --- | --- |
| <img src="github/orrt_hz_obs.png" width="100%"/> | <img src="github/vis_hz_obs.png" width="100%"/> |

| RRT\* | Visibility graph |
| --- | --- |
| Straigh path with 5 points.
| <img src="github/1.s5p-orrt.gif" width="100%"/> | <img src="github/1.s5p-vis.gif" width="100%"/> |
| Perpendicular turn with 5 points.
| <img src="github/2.905p-orrt.gif" width="100%"/> | <img src="github/2.905p-vis.gif" width="100%"/> |
| Obstacles.
| <img src="github/3.obs-orrt.gif" width="100%"/> | <img src="github/3.obs-vis.gif" width="100%"/> |
| Obstacles over points on given path.
| <img src="github/4.obs-on-rp-orrt.gif" width="100%"/> | <img src="github/4.obs-on-rp-vis.gif" width="100%"/> |
| Complete block.
| <img src="github/5.obs-over-lane-orrt.gif" width="100%"/> | <img src="github/5.obs-over-lane-vis.gif" width="100%"/> |
| Small passage.
| <img src="github/6.obs-small-passage-orrt.gif" width="100%"/> | <img src="github/6.obs-small-passage-vis.gif" width="100%"/> |
| Shielding final point.
| <img src="github/8.cha1-orrt.gif" width="100%"/> | <img src="github/8.cha1-vis.gif" width="100%"/> |
| Arc.
| <img src="github/7.cir-orrt.gif" width="100%"/> | <img src="github/7.cir-vis.gif" width="100%"/> |
| Arbitrary path (top down view).
| <img src="github/9.gen-orrt.gif" width="100%"/> | <img src="github/9.gen-vis.gif" width="100%"/> |

- Arbitrary path (side view).

<img src="github/9.gen-orrt-vis-sideview.gif" width="100%"/>

- RRT\* planning frequency vs number of sampled nodes.

<img src="github/orrt_hz_nodes.png" width="100%"/>

| Object | Description |
| --- | --- |
| White circle | Agent's circumcircle |
| Black line on agent  | Agent's orientation  |
| Faint gray path  | Given path           |
| Corresponding pink paths   | Padding obstacles in configuration space  |
| Faint green circle around agent | Sensing region |
| Inner pink rectangles | Physical obstacle bounding boxes |
| Outer pink obstacles  | Configuration space obstacle bounding boxes |
| Green path | Output path |
| Red circle | Local target position of agent after path optimization |
| Cyan rectangle | Sampling region |
| White tree | The RRT\* tree |
| White lines | Visibility graph |


