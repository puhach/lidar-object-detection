Download data from the following link and place it in the root of the project: 
https://github.com/udacity/SFND_Lidar_Obstacle_Detection/tree/master/src/sensors/data/pcd

Create the 'build' directory alongside CMakeLists.txt.

Now the project structure should look like this:

│   .gitignore
│   CMakeLists.txt
│   my_readme.txt
│   README.md
│   
├───build
├───data
│   └───pcd
│       │   simpleHighway.pcd
│       │   
│       ├───data_1
│       │       0000000000.pcd
│       │       0000000001.pcd
│       │       ....
│       │       0000000020.pcd
│       │       0000000021.pcd
│       │       
│       └───data_2
│               0000000000.pcd
│               0000000001.pcd
│               0000000002.pcd
│               ....
│               0000000152.pcd
│               0000000153.pcd
│               
├───media
│       ObstacleDetectionFPS.gif
│       
└───src
    │   environment.cpp
    │   processPointClouds.cpp
    │   processPointClouds.h
    │   
    ├───render
    │       box.h
    │       render.cpp
    │       render.h
    │       
    └───sensors
            lidar.h
            


Configure with CMake from the 'build' directory:

cmake .. -DCMAKE_TOOLCHAIN_FILE = c:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake

Build with Visual Studio

More details here:
https://knowledge.udacity.com/questions/830799