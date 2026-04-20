cmake_minimum_required(VERSION 3.16)
project(rovio)

# Compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if ( CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64|ARM64")
	message(STATUS "Building for ARM 64-bit system with arm compile flags")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -mtune=cortex-a76 -Ofast -floop-block -floop-interchange -fno-rounding-math -fprefetch-loop-arrays")
elseif ( CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM")
	message(STATUS "Building for ARM 32-bit system with armv7 compile flags")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -Ofast -floop-block -floop-interchange -fno-rounding-math -fprefetch-loop-arrays")
else()
	message(STATUS "Building for non arm system")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Ofast -floop-block -floop-interchange -fno-rounding-math -fprefetch-loop-arrays")
endif()
# --- ROVIO parameters ---
set(ROVIO_NMAXFEATURE 25 CACHE STRING "Number of features for ROVIO")
set(ROVIO_NCAM 1 CACHE STRING "Number of enabled cameras")
set(ROVIO_NLEVELS 4 CACHE STRING "Number of image levels for the features")
set(ROVIO_PATCHSIZE 6 CACHE STRING "Size of patch (edge length in pixels)")
set(ROVIO_NPOSE 0 CACHE STRING "Additional estimated poses for external pose measurements")
add_definitions(
		-DROVIO_NMAXFEATURE=${ROVIO_NMAXFEATURE}
		-DROVIO_NCAM=${ROVIO_NCAM}
		-DROVIO_NLEVELS=${ROVIO_NLEVELS}
		-DROVIO_PATCHSIZE=${ROVIO_PATCHSIZE}
		-DROVIO_NPOSE=${ROVIO_NPOSE}
)

add_subdirectory(lightweight_filtering)

include_directories(${PROJECT_SOURCE_DIR}/lightweight_filtering/include)



# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
# YAML
find_package(PkgConfig REQUIRED)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)

find_package(rovio_interfaces REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

# --- Includes ---
include_directories(
		include
		${EIGEN3_INCLUDE_DIR}
		${YamlCpp_INCLUDE_DIRS}
)

# --- Library ---
set(ROVIO_SRC
		src/Camera.cpp
		src/FeatureCoordinates.cpp
		src/FeatureDistance.cpp
		include/rovio/HealthMonitor.hpp
)
if(MAKE_SCENE)
	list(APPEND ROVIO_SRC src/Scene.cpp)
endif()

add_library(${PROJECT_NAME}_lib ${ROVIO_SRC})
ament_target_dependencies(${PROJECT_NAME}_lib
		rclcpp
		geometry_msgs
		nav_msgs
		sensor_msgs
		std_msgs
		cv_bridge
		tf2
		tf2_ros
		std_srvs
		visualization_msgs
		rosbag2_cpp
		rosbag2_storage
		rovio_interfaces
		image_transport
)
target_link_libraries(${PROJECT_NAME}_lib
		${YamlCpp_LIBRARIES}
		${OpenCV_LIBRARIES}
)


# --- Executables ---
add_executable(rovio_node src/rovio_node.cpp)
ament_target_dependencies(rovio_node rclcpp rovio_interfaces)
target_link_libraries(rovio_node ${PROJECT_NAME}_lib)

add_executable(rovio_rosbag_loader src/rovio_rosbag_loader.cpp)
ament_target_dependencies(rovio_rosbag_loader rclcpp)
target_link_libraries(rovio_rosbag_loader ${PROJECT_NAME}_lib)

add_executable(feature_tracker_node src/feature_tracker_node.cpp)
ament_target_dependencies(feature_tracker_node rclcpp)
target_link_libraries(feature_tracker_node ${PROJECT_NAME}_lib)


# --- Install ---
install(TARGETS
		${PROJECT_NAME}_lib
		rovio_node
		rovio_rosbag_loader
		feature_tracker_node
		DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
		DESTINATION include/
)

install(DIRECTORY launch/
		DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY cfg/
		DESTINATION share/${PROJECT_NAME}/cfg)

# --- Export for downstream packages ---
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
		rclcpp
		std_msgs
		geometry_msgs
		nav_msgs
		sensor_msgs
		cv_bridge
		tf2
		tf2_ros
		rosidl_default_runtime
		image_transport
)

ament_package()
