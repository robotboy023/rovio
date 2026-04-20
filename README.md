# ROVIO - RObust Visual Inertial Odometry #
![ROVIO CI build](https://github.com/suyash023/rovio/actions/workflows/rovio-ci.yml/badge.svg)

<p align="center">
  <img src="/doc/rovio_clip_0_15_to_0_25.gif" width="600">
</p>

## TL;DR

ROVIO is a lightweight, research-grade Visual-Inertial Odometry (VIO) framework
that performs well on resource-constrained platforms when properly tuned.

This fork provides:
- Full ROS 2 (Humble) support
- Simplified build and evaluation scripts
- Dataset automation (EUROC)
- Ongoing maintenance and performance improvements

If you want a fast start:
```bash
mkdir -p ~/rovio_ws/src
cd ~/rovio_ws/src
git clone git@github.com:suyash023/rovio.git
git clone git@github.com:suyash023/rovio_interfaces.git
cd rovio && git submodule update --init --recursive
source rovio/scripts/rovio_commands.sh
cd ~/rovio_ws &&  install_dependencies && build_rovio
```
## History and Present ##

This repository originates from **ETH Zurich** and is released under the **BSD (MIT-style) License**. This fork is actively maintained by **[robotboy_023](https://github.com/suyash023)**, preserving both the spirit and licensing of the original work.
ROVIO is a lightweight yet powerful VIO framework that performs exceptionally well on **resource‑constrained platforms** when properly tuned. It has been demonstrated across **a wide variety of robotic platforms**, delivering accurate and robust odometry even under tight compute budgets (see ICRA 2018 reference below).
I have worked extensively with ROVIO for several years—deploying it on constrained hardware, tuning it for diverse sensors, and contributing modifications that significantly improve accuracy and robustness. This fork aims to consolidate that experience to improve:

- Adoption and usability
- Scalability and maintainability
- Accuracy and computational efficiency

## Who This Repository Is For

This fork is intended for:
- Hobbyists, tinkerers and engineers deploying VIO on real hardware
- Researchers benchmarking or extending ROVIO
- ROS 2 users seeking a maintained ROVIO workflow

## Maintainer notes and additions ##

This README is a modified and extended version of the original documentation. Updates include:
* Simplified setup instructions
* ROS 2–aligned build and usage steps
* Scripted workflows for datasets, evaluation, and benchmarking

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License). Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: https://youtu.be/ZMAISVy-6ao

Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)
* https://rpg.ifi.uzh.ch/docs/ICRA18_Delmerico.pdf (Computational efficiency of ROVIO)
* https://nva.sikt.no/registration/0198ec92f656-bb2ddc14-baf5-4b0a-8f0b-aca855076c48  (NTNU arl masters thesis on ROVIO using thermal camera (2021))
* https://www.worldscientific.com/doi/10.1142/S2301385024410012 (Improving ROVIO  2023)

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki

## Software Dependencies ##

### Install without openGL scene (Recommended) ##

* ros2 (Tested with desktop version of humble)
* kindr (https://github.com/ethz-asl/kindr) - A git submodule.
* ~~lightweight_filtering (as submodule, use "git submodule update --init --recursive")~~ : Lightweight_filtering has been removed as submodule from this repo. It is a static dependency now.
* rovio_interfaces : This is a package needed to use services and custom messages for ROVIO. ROS2 now mandates that messages and services be maintained in a separate package.
* image_view: ros2 package image_view is needed to view the visualization images published by rovio.


## Building ROVIO ##

All builds have been tested on **Ubuntu 22.04 (64 bit) + ROS 2 Humble**.

A helper script, `rovio_commands.sh`, provides a convenient CLI for:
* Installing dependencies
* Building ROVIO
* Installing datasets
* Converting EUROC datasets (ROS 1 → ROS 2)
* Running ROVIO on EUROC
* Installing evaluation tools
* Evaluating results (ATE / RPE)

```bash
source ~/rovio_ws/src/scripts/rovio_commands.sh
```
Steps:

* Make a ros2 workspace in a desired (home here by default `~/`) folder:
```bash
mkdir -p ~/rovio_ws/src
```
* clone rovio and all the dependencies:
```bash
cd ~/rovio_ws/src/
git clone git@github.com:suyash023/rovio.git
git clone git@github.com:suyash023/rovio_interfaces.git
cd rovio
git submodule update --init -- recursive
```
* Install dependencies: ROS2, image_view
```bash
install_dependencies
```

* Build rovio

Source the script for command line utilities:
```bash
source ~/rovio_ws/src/rovio/scripts/rovio_commands.sh
```
Next, navigate to the rovio_ws directory and enter command to build rovio
```bash
cd ~/rovio_ws/
build_rovio
```
Internally, colcon build command is run to execute the building process.

>Note: Compared to the original repo Scene functionality has been removed. If needed again please open an issue


### Installing EUROC datasets ###

The EUROC MAV Dataset is commonly used for benchmarking VIO algorithms. 
More details: https://ethz-asl.github.io/datasets/euroc-mav/
This repository defaults to the Machine Hall sequences.
Make sure the script `rovio_commands.sh` is sourced and then execute the following command:
```bash
cd ~/rovio_ws/
install_euroc_datsets
```
> Dataset downloads may take time depending on your network—perfect time for coffee ☕
The datasets get installed in the `~/rovio_ws/datsets` folder.
These datasets are in ROS1 format. To convert them in ROS2 format, navigate to the rovio_ws directory and enter the following command:
```bash
cd ~/rovio_ws/
convert_euroc_datasets
```
Converted datasets are saved as `[dataset_name]_ros2` in respective dataset folders.

### ROVIO on Euroc datasets ###

By default, the config files provided in `cfg` folders have calibration and config parameters for the EUROC dataset.
To run ROVIO on the EUROC dataset source the `rovio_commands.sh` script and execute the following command:
```bash
cd ~/rovio_ws/
run_rovio_euroc
```
On running this command ROVIO will be executed on all ros2 folders in the `~/rovio_ws/datasets/machine_hall` folder.
A vizualization window appears during execution.

ROVIO can also be run on euroc dataset by launching the node and  playing the ros2 bag files. This is called the live version.
This is closer to the way ROVIO will be run on Robot. 
```bash
run_rovio_euroc_live
```
Results from this run are saved in `[daataset]_ros2/rovio/` fold.

### Evaluating ROVIO on EUROC datasets ###

To evaluate ROVIO on EUROC dataset the tool [evo](https://michaelgrupp.github.io/evo/) is used. It computes the [ATE and RPE metrics](https://docs.openvins.com/eval-metrics.html) trajectory error metrics.
With the `rovio_commands.sh` script sourced, execute the follwing command to install evo:
```bash
install_evaluation_dependencies
```
To perform the evaluation execute the following command:
```bash
evaluate_rovio_euroc
```
This computes the metrics and plots and saves the data in the `rovio` folder in each of the datasets folder.
It also generates a csv file for each dataset that contains the metrics for the results in the respective results folder.
Evaluation of results can also be performed on the evaluation of live results.
### ROVIO on your robot ###

To run ROVIO on your custom camera-IMU setup please refer to this [documentation](doc/CustomSetup.md)

## Modifications & Roadmap

### Core Features
- [x] ROS 2 support
- [x] Reset Service calls
- [ ] ROS 2 image‑based visualization
- [ ] Multithreading
- [ ] Resource usage  monitor
  - [ ] CPU
  - [ ] RAM
  
### Infrastructure
- [ ] CI/CD pipeline
- [x] ROS 2 YAML parameter support
  -  [x] Topics as parameters
- [x] TF publishing
- [ ] Reset services
- [ ] JSON files for config

### Algorithmic Improvements
- [x] IMU - camera time‑offset handling
- [ ] Strong‑feature prioritization
- [ ] Rolling‑shutter compensation
- [ ] 3D LiDAR sensor fusion
- [ ] Sliding‑window BA (GTSAM/Ceres/G2o)
- [ ] Vanishing point detection and fusion
- [x] Compiler flags
  - [x] ffast-math (OFast)
  - [x] arm and cpu based
- [x] Resizing image
  - [x] Scale camera matrix
  - [ ] Float resizing for accuracy

### Evaluation & Tooling
- [ ] RPE support in `rovio_commands.sh`
- [ ] Cumulative ATE/RPE across datasets
- [ ] Live version evaluation
  - [ ] Script to convert point_msgs to pose_msgs
- [ ] Observability analysis

### Documentation
- [ ] Wiki updates
- [x] Sensor calibration guide
- [ ] Parameter breakdown
- [x] Custom setup computational tuning.

### Deployment
- [ ] Docker support
- [ ] Prebuilt Docker images

### Research Integrations
- [x] TU Delft optimizations
  - [x] Toggle feature selection based on Feature detector score or shi-tomasi score
- [ ] MAPLAB optimizations
- [x] Custom Health Monitoring msg.
- [x] STL based health tracking and reset.
- [ ] ROTVIO

## Support & Collaboration

If you need help tuning ROVIO for a custom platform or sensor setup,
limited consulting and troubleshooting support is available. Please contact me via [email](mailto:suyashyeotikar@gmail.com) or post in issues section.

Community contributions, issue reports, and pull requests are always welcome.