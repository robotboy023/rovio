#Base image
FROM osrf/ros2:humble

#Perform update
RUN apt-get update && \
    apt-get install git && \
    apt-get install -y build-essential cmake libboost-all-dev libssl-dev libusb-1.0-0-dev

#Build ROVIO ws clone and then build
RUN mkdir -p ~/rovio_ws/src/ && \
    git clone git@github.com:suyash023/rovio.git && \
    git submodule update --init --recursive && \
    git clone git@github.com:suyash023/rovio_interfaces.git && \
    cd .. && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \

# Specify ENTRYPOINT with ~/rovio_ws/install/setup.bash
ENTRYPOINT [ "bash", "-c",
    "~/rovio_ws/install/setup.bash",
    "~/rovio_ws/src/rovio/scripts/rovio_commands.sh" ]
