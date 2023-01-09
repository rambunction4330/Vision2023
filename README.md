# AprilTagComputation
Prototype code for vision processing on team 4330's FRC 2023 season robot

## Building
See [BUILDING.md](BUILDING.md)


## Source Code Structure
|Directory|Description|
|---------|-----------|
|`tools/camera`|Source code for the camera tool used to take images of your chessboard used order to calibrate your camera|
|`tools/calibration`|Source code for the calibration tool used to generate the camera matrix and distance coefficients used in the main program`|
|`src/`|Source code of the main program used to find the location and orientation of apriltags relative to the camera|
