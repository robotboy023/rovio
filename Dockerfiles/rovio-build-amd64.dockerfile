#Base image
FROM osrf/ros:humble-desktop-full

# Perform update and install dependencies
RUN apt-get update && \
    apt-get install -y git build-essential cmake bash libboost-all-dev libssl-dev libusb-1.0-0-dev git wget unzip && \
    apt-get install -y ros-humble-image-view && \
    rm -rf /var/lib/apt/lists/*

# Kindr installation
RUN cd ~/ && \
    git clone "https://github.com/ethz-asl/kindr.git" && \
    cd kindr && mkdir -p build && cd build && cmake .. && \
    make install

# Build ROVIO ws clone and then build
RUN /bin/bash -c "mkdir -p ~/rovio_ws/src/ && cd ~/rovio_ws/src/ && \
    git clone https://github.com/suyash023/rovio.git && \
    cd ~/rovio_ws/src/rovio && \
    git submodule update --init --recursive && \
    cd .. && \
    git clone https://github.com/suyash023/rovio_interfaces.git && \
    cd ~/rovio_ws/ && \
    source /opt/ros/humble/setup.bash && \
    colcon build"

# Append sourcing script to /etc/bash.bashrc
RUN echo "source /root/rovio_ws/src/rovio/scripts/rovio_commands.sh" >> /etc/bash.bashrc
    # Specify ENTRYPOINT with setup and commands script
