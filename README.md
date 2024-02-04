# Tiago Simulator

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![humble](https://github.com/jmguerreroh/tiago_simulator/actions/workflows/master.yaml/badge.svg?branch=humble)](https://github.com/jmguerreroh/tiago_simulator/actions/workflows/master.yaml)

This package allows running different Gazebo worlds, including the [AWS Robomaker](https://github.com/aws-robotics) worlds, using the Tiago robot from [PAL Robotics](https://github.com/pal-robotics)

** In the event of communication difficulties, please consider switching the DDS. Recommended: use [Eclipse Cyclone DDS](https://docs.ros.org/en/foxy/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html). 
You can do this by installing it with `sudo apt install ros-humble-rmw-cyclonedds-cpp` and setting the `RMW_IMPLEMENTATION` environment variable: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Add it to your `.bashrc`**

# Installation 

You need to have previously installed ROS2. Please follow this [guide](https://docs.ros.org/en/humble/Installation.html) if you don't have it.
```bash
source /opt/ros/humble/setup.bash
```

Clone the repository to your workspace:
```bash
cd <ros2-workspace>/src/
git clone https://github.com/jmguerreroh/tiago_simulator.git

```
Prepare your thirparty repos:
```bash
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
cd <ros2-workspace>/src/
vcs import < tiago_simulator/thirdparty.repos
```
*Please make sure that this last command has not failed. If this happens, rerun it.*

# Building project

```bash
cd <ros2-workspace>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
``` 

# Setup Environment

Add it to your `.bashrc` file
```bash
source /usr/share/gazebo/setup.bash
source <ros2-workspace>/install/setup.bash
``` 

# Run Gazebo & Tiago in ROS2
```bash
ros2 launch tiago_simulator simulation.launch.py
``` 

To change the Gazebo world or the initial position/rotation of the Tiago robot, you can modify the `config/params.yaml` file.

If you have a low performance and you have installed the NVIDIA driver, you can use your GPU by selecting the NVIDIA PRIME profile:
```bash
sudo prime-select nvidia
```

Otherwise, you can close the Gazebo client:
```bash
pkill -f gzclient
``` 

Also, you can use [Nav2](https://navigation.ros.org/) with the robot in the world selected in `config/params.yaml`:
```bash
ros2 launch tiago_simulator navigation.launch.py
``` 
# Running moveit and gazebo:

Before using moveit and gazebo with tiago robot, there are some small fixes to be done.

First, on the tiago urdf, we have to delete line 33 to 35:
```bash
sudo sed -i '33d;34d;35d;36d' /opt/ros/humble/share/tiago_description/robots/tiago.urdf.xacro
```

Then we need to edit the gripper controller config yaml:
```bash
sudo sed -i 's/\${EE_SIDE_PREFIX}/gripper/g' /opt/ros/humble/share/pal_gripper_controller_configuration/config/gripper_controller.yaml
```
Finally (optional), if we want to do grasping tasks we should incorpore a gazebo plugin for it in the gripper xacro:
```bash
sudo sed -i '/<\/xacro:macro>/,/<\/robot>/{
    /<\/xacro:macro>/a\
<gazebo>\
    <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">\
        <arm>\
            <arm_name>pal_gripper_fix</arm_name> \
            <palm_link>wrist_ft_link</palm_link> \
            <gripper_link>gripper_left_finger_link</gripper_link>\
            <gripper_link>gripper_right_finger_link</gripper_link>\
        </arm>\
        <forces_angle_tolerance>100</forces_angle_tolerance>\
        <update_rate>4</update_rate>\
        <grip_count_threshold>4</grip_count_threshold>\
        <max_grip_count>8</max_grip_count>\
        <release_tolerance>0.005</release_tolerance>\
        <disable_collisions_on_attach>false</disable_collisions_on_attach>\
        <contact_topic>__default_topic__</contact_topic>\
    </plugin>\
</gazebo>
}' /opt/ros/humble/share/pal_gripper_description/urdf/gripper.gazebo.xacro

```

## About

This project was made by [José Miguel Guerrero], Associate Professor at [Universidad Rey Juan Carlos].

Copyright &copy; 2024.

[![Twitter](https://img.shields.io/badge/follow-@jm__guerrero-green.svg)](https://twitter.com/jm__guerrero)

## License

Shield: 

[![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

This work is licensed under a
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg


[Universidad Rey Juan Carlos]: https://www.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
