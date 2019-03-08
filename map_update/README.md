## map_update

This package provides a ROS node to integrate virtual borders into an occupancy grid map (OGM) of the environment. Thus, the prior OGM will be modified according to the desired virtual border. Such a virtual border is composed of three main components:

- Virtual border points defining the boundary of a virtual border (red lines in image 2)
- A seed point indicating the area to be modified (yellow dots in image 2)
- An occupancy probability  assigned to the area to be modified (numbers in image 2)

The resulting posterior map (after integrating a virtual border) can be used in future navigation tasks as basis for a costmap to change the robot's navigational behavior.

![Example Images](/images/map_update.png)

### Nodes

1. **map_updater**

   - Subscribed topics:

     /map ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))

   - Published topics

      /map ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))

   - Services

      ~update_map ([lab_msgs/MapUpdate](../lab_msgs/srv/MapUpdate.srv))


### License
The package is released under the terms of BSD 3-Clause. For full terms and conditions, see the [LICENSE](../LICENSE) file.

### Reference
More details can be found in the following papers: 

D. Sprute, K. Tönnies, and M. König, “Virtual Borders: Accurate Definition of a Mobile Robot’s Workspace Using Augmented Reality,” in *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2018, p. 8574–8581 ([Link](https://arxiv.org/abs/1709.00954))

```
@inproceedings{sprute:2018b,
	author = {Dennis Sprute and Klaus T{\"o}nnies and Matthias K{\"o}nig},
	title = {Virtual Borders: Accurate Definition of a Mobile Robot's Workspace Using Augmented Reality},
	booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},	
	year = {2018},
	pages={8574--8581}
}
```

D. Sprute, R. Rasch, K. Tönnies, and M. König, “A Framework for Interactive Teaching of Virtual Borders to Mobile Robots,” in *2017 26th IEEE International Symposium on Robot and Human Interactive Communication (RO-MAN)*, 2017, pp. 1175-1181 ([Link](https://arxiv.org/abs/1702.04970))

```
@inproceedings{sprute:2017b,
	author = {Dennis Sprute and Robin Rasch and Klaus T{\"o}nnies and Matthias K{\"o}nig},
	title = {A Framework for Interactive Teaching of Virtual Borders to Mobile Robots},
	booktitle={2017 26th IEEE International Symposium on Robot and Human Interactive Communication (RO-MAN)},
	year = {2017},
	pages={1175--1181}
}
```




