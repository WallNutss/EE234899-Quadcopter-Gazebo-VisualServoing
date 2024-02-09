# sjtu_drone_description

This package contains the xacro/urdf/sdf model description of the sjtu drone and the corresponding plugin for Gazebo 11 and ROS 2 Humble. It already have been verified by Me(Wallnuts), it also works in ROS 2 Foxy Version.

## Structure
* __models__: Gazebo quadcopter sdf model and model meshes
* __include__: Header files for the PID controller and drone plugin
* __src__: Source code for the drone plugin and PID controller
* __urdf__: Xacro and urdf model description files
* __worlds__: Contains more playground world

## Usage
If you want to recreate my IBVS Simulation, you can use the worlds I'm using with the name of `scenario`. I divide them in 3 skenario, skenario1_3, skenario2, and skenario3_2. Don't mind the weird numbering in behind them, it just my own revision number. I also added a small city world scenario to be used in the name of `small_city.world`. 

## Worlds
[Important] To fully load the world you need to donwload the gazebo models first:
```sh
curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp && mkdir -p ~/.gazebo/models/ && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip
```

## Reference
* https://github.com/leonhartyao/gazebo_models_worlds_collection/
* https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/