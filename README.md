# Humaniod Robot Joint Synchronization teleoperation
Click to see our demo video:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Dbm4_3v5M8o/0.jpg)]([https://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID_HERE](https://www.youtube.com/watch?v=Dbm4_3v5M8o&ab_channel=K.Ni))


## 1 .hri full body
How to use the repo:
### Setup env
sudo apt install ros-noetic-hri-msgs: the ROS4HRI ROS messages

sudo apt install ros-noetic-hri: libhri, a C++ library to ease integration of ROS4HRI in your C++ nodes

sudo apt install ros-noetic-pyhri: pyhri, a Python library to ease integration of ROS4HRI in your Python nodes

sudo apt install ros-noetic-human-description: a parametric kinematic model of a human, in URDF format

sudo apt install ros-noetic-hri-rviz: rviz plugins to display detected humans (faces and bodies) and the humans' 3D skeletons (if you run a skeleton tracker)

> You also need to pip install some pkgs.

### launch 

```bash
$ roslaunch hri_fullbody hri_fullbody.launch
```


### Trouble Shooting
If the Arm is not move, please install the ikpy versoin `3.2.2`.
```bash
$ pip install ikpy==3.2.2
```


## 2. camera 
### Install
```bash
$ sudo apt-get install ros-noetic-openni-camera
$ sudo apt-get install ros-noetic-openni-launch
$ sudo apt-get install ros-noetic-openni2-launch
```

### camera testing
```bash
$ roslaunch openni2_launch openni2.launch
```

## 3. Launch
```bash
$ roslaunch talos_teleop env.launch
```

### ROSBAG
```bash
$ rosbag record -o test.bag -e /tf /tf_static /camera/rgb/camera_info /camera/rgb/image_raw /camera/depth/image_rect_raw /camera/depth/camera_info 
$ rosbag play --clock src/talos_teleop/bags/test.bag
```


r_wrist_ics
l_wrist_ics
r_ankle_ics
l_ankle_ics
