# Urban Environment Simulation

This repository contains city environment simulation with [Unity3D](https://unity.com/).

Currently, we support the two urban scenes:
- [small](https://github.com/RuslanAgishev/ImageSynthUnity):
    for synthetic data collection and with integrated robot model and
    [ROS](https://www.ros.org/) bridge,
    based on [WindridgeCity](https://assetstore.unity.com/packages/3d/environments/roadways/windridge-city-132222);
- [big](https://gitlab.com/vedu/cscape):
    for synthetic data collection with high definition renderring and day-night change,
    based on [CScape](https://assetstore.unity.com/packages/tools/modeling/cscape-city-system-86716).

<img src="figures/pedestrians.gif"> <img src="figures/city_traffic.gif">

## Content

- [Installation](docs/INSTALL.md)

- [Synthetic data collection](docs/SynthDataCollection.md):
    1. [image segmentation, depth estimation, normals and optical flow](docs/SynthDataCollection.md#what-does-it-do)
       with [ml-imagesynthesis](https://bitbucket.org/Unity-Technologies/ml-imagesynthesis/src/master/).
    2. [localization and odometry](docs/SynthDataCollection.md#localization-and-odometry-data-logging).
    3. [3D-bounding boxes](docs/SynthDataCollection.md#3d-object-detection).

- [City traffic simulation](docs/CittyTrafficSimulation.md):
    includes animated pedestrians moving on sidewalks and pedestrian crossings as well as cars, following predefined
    routes on the roads.

- [ROS-based mobile robot simulation](https://github.com/RuslanAgishev/ImageSynthUnity/blob/planning/README.md) with the following sensors:
    1. lidar Velodyne VLP-16,
    2. frontal RGB-camera,
    3. IMU,
    4. semantic local map which is rendered as a video stream from a virtual camera above the robot.

<img src="figures/robot_model.png"/>
