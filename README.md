# point_cloud_pkg


# 运行

* 2d point cloud
`roslaunch point_cloud_pkg test.launch `

* 3d 
`roslaunch point_cloud_pkg point_collsion3d.launch `


## bug

需要启动一个带有map frame仿真器，推荐turtlebot3
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 

roslaunch turtlebot3_navigation turtlebot3_navigation.launch open_rviz:=false

```


原理说明见
[blog](https://blog.csdn.net/qq_37087723/article/details/130536739?csdn_share_tail=%7B%22type%22%3A%22blog%22%2C%22rType%22%3A%22article%22%2C%22rId%22%3A%22130536739%22%2C%22source%22%3A%22qq_37087723%22%7D)


