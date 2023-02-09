# BUILDING

This guide will take you through building this program and its dependencies.

## Prerequisites
1. [cmake](https://cmake.org/download/)
2. [ninja](https://github.com/ninja-build/ninja/releases)/make (ninja is recommended for faster builds, but certainly not needed)

### Building Dependencies

#### OpenCV:
Generally, you are going to want to build opencv from source. If you can find an installer for your system, you can skip this section.

First, get the source code and untar/unzip it.

Second, open a terminal to the source code folder
```bash
mkdir build && cd build
cmake .. -GNinja -DCMAKE_INSTALL_PREFIX=<path to the place you want opencv to install to>
ninja
ninja install
```

#### WPILib
This is required for the networking and camera server utility libraries. Because only these two are needed, you can exclude everything else from the build to save time.

```bash
cmake .. -GNinja -DWITH_JAVA=OFF -DWITH_WPILIB=OFF -DWITH_SIMULATION_MODULES=OFF -DWITH_WPIMATH=OFF -DWITH_EXAMPLES=OFF -DWITH_TESTS=OFF -DWITH_GUI=OFF
ninja
ninja install
```

**NOTE:** If you decide to install this to a nonstandard prefix, make sure to set `CMAKE_PREFIX_PATH` to the path with all of the `libraryname/*.cmake" files in your installation directory so that cmake can find them.

### Building this program
```bash
mkdir build && cd build
cmake .. -GNinja -DOpenCV_DIR=<where_you_installed_opencv>/lib/cmake/opencv4 -DCMAKE_PREFIX_PATH=<where_you_installed_wpilib>
ninja
```

If you are on a raspberry pi (or any arm-based board), be sure to add the flag `-DIS_RPI=ON` which will enable some arm-cpu-only features.
