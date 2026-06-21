# Base image for ARM64 architecture
# Need to verify the base image
FROM osrf/ros2:humble-ubuntu-jammy-arm64

# Perform update and install dependencies
RUN apt-get update && \
    apt-get install -y git build-essential cmake libboost-all-dev libssl-dev libusb-1.0-0-dev && \
    rm -rf /var/lib/apt/lists/*

# Build ROVIO ws clone and then build
RUN mkdir -p ~/rovio_ws/src/ && \
    git clone https://github.com/suyash023/rovio.git ~/rovio_ws/src/rovio && \
    cd ~/rovio_ws/src/rovio && \
    git submodule update --init --recursive && \
    cd .. && \
    git clone https://github.com/suyash023/rovio_interfaces.git ~/rovio_ws/src/rovio_interfaces && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Specify ENTRYPOINT with setup and commands script
ENTRYPOINT [ "bash", "-c", "source ~/rovio_ws/install/setup.bash && ~/rovio_ws/src/rovio/scripts/rovio_commands.sh" ]
