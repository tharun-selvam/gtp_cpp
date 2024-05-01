## Steps to run
  1. Run `cmake CMakeLists.txt`
  2. RUn `make`
  3. Run `\test` for running `test.cpp` and `\track` for `track.cpp`

## Installing Eigen
  1. Follow [this](https://eigen.tuxfamily.org/dox/GettingStarted.html)
  2. `Eigen` documentation can be found [here](https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html)

## Overview of code
  1. It is assumed that the `track_centers`, `track_tangents` and `track_normals` are received as csv files.
  2. `track.cpp` reads the files and stores them in a `Track` instance as `Eigen` arrays.
  3. `gtp.cpp` uses this `Track` instance to do the rest.
