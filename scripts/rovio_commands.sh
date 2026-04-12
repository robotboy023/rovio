#!/bin/bash

#Author: Suyash Yeotikar
#This file contains helper functions to execute commands of the rovio package.
#Usage: source rovio_commands.sh . Then execute any of the functions below.
#Note: run all commands in this file from rovio_ws folder in a terminal


#function to build rovio
function build_rovio() {
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
}

#function to build rovio fresh
function build_rovio_fresh() {
  ROVIO_WS=$(pwd)
  rm -rf $ROVIO_WS/build $ROVIO_WS/install $ROVIO_WS/log
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
}

#function to build rovio fresh in debug mode
function build_rovio_fresh() {
  ROVIO_WS=$(pwd)
  rm -rf $ROVIO_WS/build $ROVIO_WS/install $ROVIO_WS/log
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
}

#function to convert euroc datasets from ros 1 to ros2 bag formats
function convert_euroc_rosbags(){
  DATASETS_DIR=$(pwd)/datasets/machine_hall
  folders=($( ls $DATASETS_DIR))
  for fol in "${folders[@]}"; do
    ROS2_DIR=$DATASETS_DIR/$fol/${fol}_ros2
    if [[ -d $ROS2_DIR ]]; then
      echo "ROS2 dir exists. No need to convert dataset: ${ROS2_DIR}"
    else
      echo "Converting dataset: ${fol}"
      rosbags-convert --dst ${ROS2_DIR} $DATASETS_DIR/${fol}/${fol}.bag
    fi

  done

}

#function to build rovio in debug mode
function build_rovio_debug() {
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
}

#function to install euroc datasets
function install_euroc_datasets() {
  DATASETS_DIR=$(pwd)/datasets
  EUROC_LINK="https://www.research-collection.ethz.ch/bitstreams/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/download"
  mkdir -p $DATASETS_DIR
  wget -P $DATASETS_DIR $EUROC_LINK
  unzip -d ${DATASETS_DIR} ${DATASETS_DIR}/download
  rm -rf ${DATASETS_DIR}/download
}



#fuction to install evaluation dependecies
#Note evo installation might require specific version dependencies.
#Follow the debug messages and install the specific versions
function install_evaluation_dependencies() {
  pip install evo
}

#function to install dataset conversion dependencies
function install_dataset_dependencies() {
  sudo apt-get install python3-pip
  pip install rosbags
}


#function to check if ROS2 is installed
function check_ros2_install() {
  ROS_LOCATION="/opt/ros"
  if [[ -d $ROS_LOCATION ]]; then
    echo "yes"
  else
    echo "no"
  fi
  return 0
}


#function to install kindr
function install_kindr() {
  KINDR_URL="https://github.com/ethz-asl/kindr.git"
  CURRENT_DIR=$(pwd)
  cd ~/
  git clone ${KINDR_URL}
  cd kindr
  mkdir -p build
  cd build
  cmake ..
  make -j10
  sudo make install
  cd ${CURRENT_DIR}
}

#function to install image_view to view ROVIO vis published
function install_image_view() {
  sudo apt install -y ros-${1}-image-view
}


#function to install ROS2 if not alredy present
function install_ros2() {
  ROS_DISTRO="humble"
  install=check_ros2_install
  if [[ $install == "yes" ]]; then
    echo "ROS2 found! Skipping ROS2 install"
  else
    echo "Installing ROS2: $ROS_DISTRO"
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-${ROS_DISTRO}-desktop -y
    sudo apt install ros-dev-tools -y
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
  fi
}


#function to install rovio dependencies
function install_dependencies() {
  install_ros2
  install_image_view "humble"
  install_kindr
}

#function to install rovio scene depdencies
function install_scene_dependencies() {
  sudo apt-add-repository universe
  sudo apt-get update
  sudo apt-get install freeglut3-dev libglew-dev
}

#function to wait for ROVIO to finish completion
function wait_for_rovio() {
  echo "Waiting for ROVIO to start..."
  until ros2 node list | grep -q "/rovio" > /dev/null; do
    sleep 1
  done

  echo "ROVIO running, waiting for completion..."
  while ros2 node list | grep -qx "/rovio" > /dev/null; do
    sleep 1
  done

  echo "ROVIO finished"
}


#function to run rovio on euroc datasets
function run_rovio_euroc() {
  EUROC_DATASETS_LOCATION=$(pwd)/datasets/machine_hall
  ROVIO_WS=$(pwd)/install/setup.bash
  dirList=($(ls ${EUROC_DATASETS_LOCATION}))
  source $ROVIO_WS
  for dir in "${dirList[@]}"; do
    ROS2_BAG_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/${dir}_ros2.db3
    ROVIO_OUTPUT_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/rovio/
    echo "Deleting prevous rovio output location"
    rm -rf ${ROVIO_OUTPUT_LOCATION}
    echo "Processing dataset: ${ROS2_BAG_LOCATION}"
    if [ -f ${ROS2_BAG_LOCATION} ]; then
      ros2 launch rovio ros2_rovio_rosbag_loader_launch.yaml rosbag_filename:=$ROS2_BAG_LOCATION &
      wait_for_rovio
      killall -9 image_view
    else
      echo "Skipping dataset: ${dir}, no ros2 bag file found"
    fi
  done
}

function run_rovio_euroc_debug() {
  EUROC_DATASETS_LOCATION=$(pwd)/datasets/machine_hall
  ROVIO_WS=$(pwd)/install/setup.bash
  dirList=($(ls ${EUROC_DATASETS_LOCATION}))
  source $ROVIO_WS
  for dir in "${dirList[@]}"; do
    ROS2_BAG_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/${dir}_ros2.db3
    ROVIO_OUTPUT_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/rovio/
    echo "Deleting prevous rovio output location"
    rm -rf ${ROVIO_OUTPUT_LOCATION}
    echo "Processing dataset: ${ROS2_BAG_LOCATION}"
    if [ -f ${ROS2_BAG_LOCATION} ]; then
      ros2 launch rovio ros2_rovio_rosbag_loader_launch.yaml rosbag_filename:=$ROS2_BAG_LOCATION  --launch-prefix 'gdb -ex run --args' 
      wait_for_rovio
      killall -9 image_view
    else
      echo "Skipping dataset: ${dir}, no ros2 bag file found"
    fi
  done
}



#function to run rovio on euroc datasets using the live verson
run_rovio_euroc_live() {
  EUROC_DATASETS_LOCATION=$(pwd)/datasets/machine_hall
  ROVIO_WS=$(pwd)/install/setup.bash
  dirList=($(ls ${EUROC_DATASETS_LOCATION}))
  source $ROVIO_WS

  for dir in "${dirList[@]}"; do
    ROS2_BAG_LOCATION="${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/${dir}_ros2.db3"
    ROVIO_OUTPUT_LOCATION="${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/rovio_live"
    rm -rf "${ROVIO_OUTPUT_LOCATION}"
    mkdir -p "${ROVIO_OUTPUT_LOCATION}"

    echo "Processing dataset: ${ROS2_BAG_LOCATION}"
    if [ -f "${ROS2_BAG_LOCATION}" ]; then
      # Launch ROVIO in background
      ros2 launch rovio ros2_rovio_node_launch.yaml &
      ROVIO_PID=$!

      # Wait until /clock is available
      until ros2 topic list | grep -q "/clock"; do
        sleep 0.2
      done

      # Start recording in background
      nohup ros2 bag record -a -o "${ROVIO_OUTPUT_LOCATION}/rovio_live_output.bag" > /dev/null 2>&1 &
      BAG_RECORD_PID=$!

      # Play bag in foreground → script will wait until finished
      ros2 bag play "${ROS2_BAG_LOCATION}" --clock

      # After playback finishes, stop recording and ROVIO
      killall -9 rovio_node
      sleep 1
      killall -9 image_view
      sleep 1
      killall -9 ros2
      sleep 1

      echo "Finished dataset: ${dir}"
    else
      echo "Skipping dataset: ${dir}, no ros2 bag file found"
    fi
  done
}

#function to evaluate rovio trajectory on euroc dataset results. Generates plots and ATE RPE metrics
function evaluate_rovio_euroc() {
  DATASETS_DIR="$(pwd)/datasets/machine_hall"
  DIR_LIST=($(ls $DATASETS_DIR ))
  GT_TOPIC="/leica/position"
  ODOM_TOPIC="/rovio/odometry"
  for dir in "${DIR_LIST[@]}"; do
    ROVIO_RESULT=${DATASETS_DIR}/$dir/${dir}_ros2/rovio/
    BAG_FILE=$(ls $ROVIO_RESULT | grep "bag")
    echo "Evaluating bag file: ${BAG_FILE}"
    BAG_LOCATION=${ROVIO_RESULT}/${BAG_FILE}
    evo_ape bag2 ${BAG_LOCATION} ${GT_TOPIC} ${ODOM_TOPIC} -va --save_results ${ROVIO_RESULT}/${dir}_ape_results.zip
    evo_res ${ROVIO_RESULT}/${dir}_ape_results.zip --save_table ${ROVIO_RESULT}/${dir}_ape_rovio.csv
  done
}



