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

### Building this program
```bash
mkdir build && cd build
cmake .. -GNinja -DOpenCV_DIR=<where_you_installed_opencv>/lib/cmake/opencv4
ninja
```
