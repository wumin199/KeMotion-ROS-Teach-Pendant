
KeMotion-ROS Teach Pendant
======



## 特点

1.	该demo具有仿真(Simulation)和实际连接(Bring up)两种模式
2.	仿真采用moveit中的dummy controller来仿真，而不是采用gazebo
3.	实际连接中，ros端是基于[rmi_driver](https://github.com/smith-doug/rmi_driver)功能包，采用[robot_movement_interface](https://github.com/ros-industrial/robot_movement_interface)接口实现
4.	仿真采用的moveit接口来实现ptp、lin等的规划，和实际keba机器人控制器的规划有差别，这里仅做参考用

![KeMotion-ROS Teach Pendant](docs/images/KeMotion-ROS_Teach_Pendant.png)

5.  视频演示效果见[demo](docs/videos/demo.mp4)

## ros端准备工作

- 版本说明：

| 项目| 版本 |
| :------ | :------ |
|ubuntu|16.04|
|ros|kinetic|
|[rmi_driver](https://github.com/smith-doug/rmi_driver)|DigitalIO分支或最新分支|
|[robot_movement_interface](https://github.com/ros-industrial/robot_movement_interface)|-|


安装和升级**MoveIt!**，方法：

```sh
sudo apt-get update
sudo apt-get install ros-kinetic-moveit*
```

安装依赖包**TrackIK**，方法：

```sh
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
```


- 安装该Demo方法

```sh
cd ~/catkin_ws/src
git clone https://github.com/wumin199/KeMotion-ROS-Teach-Pendant.git
cd ..
catkin_make
source devel/setup.bash
```

## 控制器端准备工作

仿真模式下不需要该步骤，但如果需要连接实际机器人进行测试，请执行：

- 版本说明

| 项目| 版本 |
| :------ | :------ |
|控制器版本|V3.16及以上|
|ROS库|0.0.10|

示教器程序和工程见`项目工程:ER7_RPT.project`(联系keba工程师提供)

需要说明的事，`项目工程:ER7_RPT.project`中配置的是`CP088/C Motion`控制器，采用的是`T70R`，实际中可以修改为其他控制器类型和示教器类型，只要做对应的修改并编译通过即可。


## 启动方法

- ros端：

1. 启动rviz: `roslaunch er7_moveit_config demo.launch`
2. 启动demo界面: `roslaunch rtp_gui demo.launch`

如果需要测试实际连接机器人的情况，请执行：

- <span id="controllerSetup">控制器端：</span>

1. 让工控机和控制器处于同一网段，比如控制器采用`192.168.101.100`，工控机采用`192.168.101.101`
2. 下载控制器工程和示教器模板程序
3. 使示教器处于外部模式，按`F1`加载并启动示教器程序(`F2`是停止并卸载示教器程序）




## 操作说明

`Drag/Drog`界面说明：

![DragJogMask](docs/images/DragJogMask.png)

| 界面元素| 说明 |
| :------ | :------ |
|Simulation和Bringup|选中Simulation表示使用仿真模式，反之使用Bringup模式|
|Execute|在Simulation或Bringup模式下执行连续运动，连续运动的点位在`PointList`这个标签页中|
|J1+~J6-|单轴点动，只在Simulation模式下有效|
|X+～Rz-|笛卡尔点动，只在Simulation模式下有效|
|Home|机器人回零，只在Simulation模式下有效|
|Stop|机器人停止，只在Simulation模式下有效，可以终止仿真模式下的连续运动|
|Velocity Scaling|速度设置，只在Simulation模式下有效|
|J1/deg~J6/deg|显示当前机器人的各轴关节值|
|X/mm~Y/deg|显示机器人当前的笛卡尔坐标值，采用RPY表示法|

`PointList`界面说明：

![PointListMask](docs/images/PointListMask.png)

| 界面元素| 说明 |
| :------ | :------ |
|ptp(point1)~ptp(point4)|按下Execute按钮后执行的连续运动的点位，执行顺序由上到下进行。各点具有一定的默认值，可以通过`teach and save`按钮修改|
|Attribute栏|显示的是当前点位下的坐标值和速度值|
|teach and save|用来试教记录当前点位给point1～point4|


- Simulation模式

可以通过点动关节坐标或笛卡尔坐标来实现模型的运动，并通过`teach and save`按钮来保持当前点给point1～point4。通过`Execute`按钮可以实现point1～point4的连续运动。可以通过`Velocity Scaling`改变点动的速度。


- Bring up模式

使用该模式前，需要先进行控制器端的一些设置[控制器端](#controllerSetup)，让示教期程序处于运行状态，之后按如下顺序进行。

在`Bringup`模式下执行`Execute`，可以将point1～pont4下发给实际的机器人控制器执行，且此时机器人模型是由控制器上传的位置信息来更新的。默认各点的圆滑参数为`ovlrel = 80`

该模式下，使用关节点动或笛卡尔点动无效。

- Bugs:

1. 点动时，move_group有可能会奔溃，这是moveit的一个bug : [Apparent race condition causes trajectory_execution_manager to crash](https://github.com/ros-planning/moveit/issues/1481)

遇到这个问题，目前只能关闭程序后再重启。

## rmi_driver简单说明

该demo的bring up模式是基于keba的[rmi_driver](https://github.com/smith-doug/rmi_driver)进行的。


与该功能包通讯采用topic方式，其信息为：

话题名: `command_list`

话题类型: `robot_movement_interface::CommandList`

关于该话题的详细信息，可以参考[robot_movement_interface](https://github.com/ros-industrial/robot_movement_interface)

即用户只要将自己想要执行的点位信息转换为`robot_movement_interface::CommandList`类型的数据并发布到`command_list`话题上，则`rmi_driver`会接受到该话题，并进行转发处理，使得真实的机器人控制器执行下发的命令并周期性反馈执行情况。
