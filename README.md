
KeMotion-ROS Teach Pendant
======

## Features

1.	This demo has two modes: `simulation` and `bring up`
2.	Simulation through fake controller instead of gazebo
3.	bring up mode is based on [rmi_driver](https://github.com/smith-doug/rmi_driver). This package uses interfaces [robot_movement_interface](https://github.com/ros-industrial/robot_movement_interface) 
4.	moveit interface adopted in simulation mode is little different from actual keba robot controller in ptp/lin, which is only for reference

5. `voice control` based on [iflytek](https://www.xfyun.cn/)

![KeMotion-ROS Teach Pendant](docs/images/KeMotion-ROS_Teach_Pendant.png)

6.  video demo see [demo](docs/videos/demo.mp4)

## Preparation:ROS

- Version and dependency:

| item| version |
| :------ | :------ |
|ubuntu|16.04|
|ros|kinetic|
|[rmi_driver](https://github.com/smith-doug/rmi_driver)|DigitalIO or newer branch|
|[robot_movement_interface](https://github.com/ros-industrial/robot_movement_interface)|-|


This demo's branch descriptions:

| branch| description |
| :------ | :------ |
|[V0](https://github.com/wumin199/KeMotion-ROS-Teach-Pendant/tree/V0)|This branch includes `simulation` and `bring up` modes, but not voice control function|
|V1|This branch includes `simulation` and `bringup` modes, contains `voice control` function as well|


Install or upgrade **MoveIt!**:

```sh
sudo apt-get update
sudo apt-get install ros-kinetic-moveit*
```

Install dependency **TrackIK**:

```sh
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
```

Install dependency **speech recognition**:

copy `libmsc.so` from source code `/voice_control/libs/` to system directory based on Linux architecture:

```sh
sudo cp libmsc.so /usr/lib/libmsc.so
```

and also install other speech recognition dependies:

```sh
sudo apt-get install libasound2-dev
```

- Install this package **RTP Demo**:

```sh
cd ~/catkin_ws/src
git clone https://github.com/wumin199/KeMotion-ROS-Teach-Pendant.git
cd ..
catkin_make
source devel/setup.bash
```

## Preparation:Keba Controller

- Version：

| item| version |
| :------ | :------ |
|controller version|V3.16 or newer|
|ROS lib|0.0.10|

KAIRO program and controller project, see `ER7_RPT.project`(Contact keba engineer to provide it)

In this project, you can modify `CP088/C Motion` or `T70R` into others.


## Setup

- ROS

1. run rviz: `roslaunch er7_moveit_config demo.launch`
2. run demo gui: `roslaunch rtp_gui demo.launch`

If you want to switch to bring up mode, you need to :

- <span id="controllerSetup">Controller(bring up mode)</span>

1. Make sure PC and controller in the same network. For example, controller adopts `192.168.101.100`，PC adopts`192.168.101.101`
2. Download the controller project into controller
3. Make sure teach pendant in external mode, press `F1` to load and start the program.(`F2` to stop and uninstall the program)


If you want to test voice control function, please excute the following instructions:

1. run voice control demo: `roslaunch voice_control demo.launch`

The default recognition language is Chinese, if you want it to be English, please modify it in [demo.launch](voice_control/launch/demo.launch):

from

`<param name="language" type="string" value="zh_cn" />`

to 

`<param name="language" type="string" value="en_us" />`


## Usage

`Drag/Drog`Mask:

![DragJogMask](docs/images/DragJogMask.png)

| UI Element| notes |
| :------ | :------ |
|Simulation and Bringup|Select `Simulation` to use simulation mode, and `Bringup` for actual connection with keba controller|
|Execute|Perform a continuous movement in simulation or bringup mode, the points of which are in the `PointList` Tab |
|J1+~J6-|Axis jogging, only valid in simulation mode|
|X+～Rz-|Cartesian jogging,only valid in simulation mode|
|Home|Robot homing, only valid in simulation mode|
|Stop|Robot stoping, only valid in simulation mode|
|Velocity Scaling|Velocity setting, only valid in simulation mode|
|J1/deg~J6/deg|Display current joint values|
|X/mm~Y/deg|Dispaly current cartesian values, using RPY notation|

`PointList`Mask:

![PointListMask](docs/images/PointListMask.png)

| UI Element| notes |
| :------ | :------ |
|ptp(point1)~ptp(point4)|Continuous move from point1 to point4 when pressing `Execute`. Each point has a default value and can be modified by pressing `teach and save` |
|Attribute column|Display coordinates and velocity under current point|
|teach and save|Record current values to pont1 ~ point4|


`VoiceControl`Mask:

![VoiceControlMask](docs/images/VoiceControlMask.png)

| UI Element| notes |
| :------ | :------ |
|Record|Speech record starts while pressing `Record`, Speech record stops while releasing `Record`|
|Recording Status|Show speech recognition results. If the results match the keywords supported below, the robot will make corresponding actions|


- Simulation

You can jog the model and press `teach and save` to save current values to point1 ~ point4. Continuous move can be realized by pressing `teach and save`. velocity can be changed by `Velocity Scaling`.


- Bring up

Before using this mode, you should set up controller[controller setup](#controllerSetup), and make sure teach pendant program is in running state.

Press `Execute` under `Bringup`, it will send point1 ~ point4 to keba controller for execution. At this time, robot model is updated by controller feedback.

Default blending `ovlrel = 80` is set for each point.

Jog is invalid under this mode.

- Bugs

1. MoveGroupInterface interface can cause move group crash. This issue may happen when jogging. see[Apparent race condition causes trajectory_execution_manager to crash](https://github.com/ros-planning/moveit/issues/1481)


- voice control

voice control can run in `Simulation` or `Bring up` modes.

It's based on iflytek, you can refer to [iflytek resource](https://www.xfyun.cn/). The recognition efficiency depends on the capability of iflytek speech recognition package.

In the trial version of iflytek speech recognition package, the API is bound with personal ID, and the API is limited to 500 times per day. After 500 times, the function cannot be used on the day.

The `libmsc.so` mentioned above is bound with personal ID. It is suggested that users apply for their own recognition package SDK on iflytek open platform  to replace `libmsc.so` here, and modify the [appid](voice_control/src/iat_voice_control.cpp) in the source code.

```c++
const char* login_params = "appid = 5d48d7a3, work_dir = .";
```

And recompile the demo, otherwise it will be easier to trigger the limit on the number of times the API can be accessed if multiple people test the code.


Speech recognition by default is Chinese, if you need to enable speech recognition in English, please do corresponding modification in [demo.launch](voice_control/launch/demo.launch)


If the demo is running on a virtual machine, some necessary setup is required to ensure that the virtual machine collects sound input.

Currently supported voice commands are as follows:

| voice| description |
| :------ | :------ |
|move to test point/运动到测试点|keywords: point or 测试|
|move up/向上运动|keywords: up or 上|
|move down/向下运动|keywords: down or 下|
|move left/向左运动|keywords: left or 左|
|move right/向右运动|keywords: right or 右|
|move forward/向前运动|keywords: forward or 前|
|move backward/向后运动|keywords: backward or 后|
|robot stop/停止运动|keywords: stop or 停止|

In the mask of `VoiceControl`, hold down the `Record` button and speak. After the end of the speech, release the button. After a short period of time, the speech recognition results will be recorded under the tag `Recording Status`.If the recognition results contains the keywords in the table above, the robot will perform the corresponding action.

Some notes:

1. If no words are spoken within 10s after holding down `Record` button, the recognition will end and the result will be null.
2. If a sentence is followed by an interval of more than 2s before the next sentence, only the speech before 2s can be recognized.
3. Various reasons will affect the recognition accuracy: the voice volume is too small, the pronunciation is not clear, the noise is too loud, etc.


## rmi_driver

bring up mode is based on [rmi_driver](https://github.com/smith-doug/rmi_driver)

topic is used to communicate with *rmi_driver*:

topic: `command_list`

data type: `robot_movement_interface::CommandList`

That is, as long as users convert the point information they want to execute into `robot_movement_interface::CommandList` and publish it to the topic `command_list`, `rmi_driver` will receive and relay it to the real robot controller.
