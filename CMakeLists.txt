cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(lidar_obstacle_detection)

set(CXX_STANDARD 14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

#find_package(PCL 1.11 REQUIRED)
find_package(PCL 1.12 REQUIRED)

message("PCL includes")
message(${PCL_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (lidar_obstacle_detector 
		src/sensors/lidar.h 
		src/render/box.h 
		src/render/render.h 
		src/render/render.cpp
		src/kdtree.h
		src/euclidean_clusterer.h
		src/euclidean_clusterer.hpp
		src/point_cloud_processor.h 
		src/point_cloud_processor.hpp 
		src/environment.cpp )

target_link_libraries (lidar_obstacle_detector ${PCL_LIBRARIES})




