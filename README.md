# Lidar Obstacle Detector

This project implements filtering, segmentation, and clustering of real point cloud data to detect obstacles in a driving environment.

## Requirements

* CMake 3.1 or newer
* C++14 compiler
* PCL 1.7 or newer

Tested with CMake 3.13.0 + GCC 5.5 + PCL 1.7.2 and CMake 3.20.1 + MSVC 16.11.6 + PCL 1.12.

## Project structure

Create the *build* directory alongside *CMakeLists.txt*.

Make sure the *data* folder is present next to *CMakeLists.txt*. If not, download it from [here](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/tree/master/src/sensors/data/pcd) and place in the root of the project.

The project structure should look like this:
```
│   .gitignore
│   CMakeLists.txt
│   README.md
│   
├───build
│                       
├───data
│   └───pcd
│       │   simpleHighway.pcd
│       │   
│       ├───data_1
│       │       0000000000.pcd
│       │       0000000001.pcd
│       │       0000000002.pcd
│       │       ...
│       │       0000000021.pcd
│       │       
│       └───data_2
│               0000000000.pcd
│               0000000001.pcd
│               ...
│               0000000151.pcd
│               0000000152.pcd
│               0000000153.pcd
│               
│       
└───src
    │   environment.cpp
    │   euclidean_clusterer.h
    │   euclidean_clusterer.hpp
    │   kdtree.h
    │   point_cloud_processor.h
    │   point_cloud_processor.hpp
    │   
    ├───render
    │       box.h
    │       render.cpp
    │       render.h
    │       
    └───sensors
            lidar.h
            
```

## Build

**Linux**

From the *build* directory run
```
cmake ..
make
```

**Windows**

Depending on how PCL was installed, it may be necessary to specify VCPKG toolchain file. Run the following commands from the 'build' directory:

```
cmake .. -DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake
cmake --build .
```


## Launch

Go to the *build* folder and run *lidar_obstacle_detector*. It will start streaming PCD files from *../data/pcd/data_1*, so in case of an error make sure the data files are present at that location.


## Credits

This project was created as an assignment from the [Sensor Fusion Engineer Nanodegree program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313) at Udacity.
