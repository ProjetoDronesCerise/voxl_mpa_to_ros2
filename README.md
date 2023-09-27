# voxl-mpa-to-ros2

ROS2 Foxy Nodes that takes in mpa data and published it to ROS2

## Build Instructions

1. Requires the qrb5165-emulator (found [here](https://gitlab.com/voxl-public/support/voxl-docker)) to run docker ARM image
    * (PC) ```cd [Path To]/voxl-mpa-to-ros2```
    * (PC) ```sudo voxl-docker -i voxl-emulator```
2. Build project binary:
    * (qrb5165-emulator) ```./install_build_deps.sh qrb5165 stable```
    * (qrb5165-emulator) ```./clean.sh```
    * (qrb5165-emulator) ```./build.sh```
    * (qrb5165-emulator) ```./make_package.sh ipk```


### Installation
Install mpa-to-ros2 by running (VOXL):
```
(VOXL2/RB5):
```
apt install voxl-mpa-to-ros2
```

### Start Installed MPA ROS2 Node

Run the following commands(on voxl2):

Source the ros2 foxy setup script:

```source /opt/ros/foxy/setup.bash```

You can then run the nodes with: 

```
ros2 launch voxl_mpa_to_ros2 voxl_mpa_to_ros2.launch
```

##### Supported Interfaces
The current supported mpa->ros2 translations are:  

-cameras from voxl-camera-server or any services that publish an overlay (other than encoded images - only raw)

-IMUs

-6DOF data from voxl-qvio-server

-Point Clouds from the TOF sensor or services providing point clouds such as voxl-dfs-server

### Expected Behavior
```
voxl:/$ ros2 launch voxl_mpa_to_ros2 voxl_mpa_to_ros2.launch

MPA to ROS app is now running

Found new interface: stereo
Found new interface: tof_conf
Found new interface: tof_depth
Found new interface: tof_ir
Found new interface: tof_noise
Found new interface: tracking
Found new interface: tof_pc

```
