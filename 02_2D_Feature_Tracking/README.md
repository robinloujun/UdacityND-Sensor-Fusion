# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1
 * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
 * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


## Result Evaluation

The TOP3 combinition of detector/descriptor are FAST+BRIEF, FAST+ORB and FAST+BRISK. 

| Index | Detector Type | Descriptor Type | Average Time of Detection | Average Time of Extraction | Average Time of Matching |                   |
|-------|---------------|-----------------|---------------------------|----------------------------|--------------------------|-------------------|
| 1     | FAST          | BRIEF           | 0.9393974                 | 0.6775177                  | 0.185990555555556        | 0.600968551851852 |
| 2     | FAST          | ORB             | 0.9422022                 | 1.1129034                  | 0.181077111111111        | 0.745394237037037 |
| 3     | FAST          | BRISK           | 0.9578988                 | 1.1761046                  | 0.433062888888889        | 0.855688762962963 |
| 4     | HARRIS        | BRIEF           | 3.3983657                 | 0.3152178                  | 0.094221                 | 1.26926816666667  |
| 5     | HARRIS        | ORB             | 3.2152563                 | 0.7460052                  | 0.077153111111111        | 1.3461382037037   |
| 6     | HARRIS        | BRISK           | 3.6506481                 | 0.345173                   | 0.080521555555556        | 1.35878088518519  |
| 7     | ORB           | BRIEF           | 5.8015786                 | 0.4669485                  | 0.160361                 | 2.1429627         |
| 8     | SHITOMASI     | BRIEF           | 9.1282166                 | 0.6506422                  | 0.278831888888889        | 3.35256356296296  |
| 9     | SHITOMASI     | ORB             | 9.072227                  | 0.896068                   | 0.123887333333333        | 3.36406077777778  |
| 10    | ORB           | ORB             | 5.9472101                 | 4.2064609                  | 0.142845888888889        | 3.4321722962963   |
| 11    | SHITOMASI     | BRISK           | 9.9801928                 | 1.0092652                  | 0.513501                 | 3.83431966666667  |
| 12    | FAST          | SIFT            | 0.9106237                 | 10.4212216                 | 0.456855222222222        | 3.92956684074074  |
| 13    | HARRIS        | SIFT            | 3.8975358                 | 8.1692364                  | 0.029513777777778        | 4.03209532592593  |
| 14    | ORB           | BRISK           | 13.8050829                | 0.9871515                  | 0.143852666666667        | 4.97869568888889  |
| 15    | SHITOMASI     | SIFT            | 10.0448677                | 9.7781341                  | 0.351375222222222        | 6.72479234074074  |
| 16    | ORB           | SIFT            | 5.926501                  | 19.6065732                 | 0.225053333333333        | 8.58604251111111  |
| 17    | BRISK         | BRIEF           | 27.2860893                | 0.9298253                  | 0.389618666666667        | 9.53517775555556  |
| 18    | BRISK         | BRISK           | 27.2539596                | 2.1234399                  | 0.383549333333333        | 9.92031627777778  |
| 19    | FAST          | FREAK           | 0.9000033                 | 29.2501754                 | 0.198020333333333        | 10.1160663444444  |
| 20    | HARRIS        | FREAK           | 4.0206911                 | 27.4518713                 | 0.090433222222222        | 10.5209985407407  |
| 21    | BRISK         | ORB             | 27.9623179                | 4.0374051                  | 0.330155555555556        | 10.7766261851852  |
| 22    | ORB           | FREAK           | 6.0714523                 | 27.1523255                 | 0.113089333333333        | 11.1122890444444  |
| 23    | SHITOMASI     | FREAK           | 9.8239357                 | 27.9435637                 | 0.158382555555556        | 12.6419606518519  |
| 24    | AKAZE         | BRISK           | 37.0025929                | 1.3476577                  | 0.325510777777778        | 12.8919204592593  |
| 25    | AKAZE         | BRIEF           | 38.4017745                | 0.9326549                  | 0.225276888888889        | 13.186568762963   |
| 26    | AKAZE         | ORB             | 39.9150244                | 2.7960572                  | 0.181236222222222        | 14.2974392740741  |
| 27    | SIFT          | BRIEF           | 44.1632669                | 0.7349171                  | 0.204650111111111        | 15.034278037037   |
| 28    | BRISK         | SIFT            | 27.7345632                | 17.5549221                 | 0.848392888888889        | 15.3792927296296  |
| 29    | SIFT          | BRISK           | 44.7978566                | 1.153151                   | 0.195478333333333        | 15.3821619777778  |
| 30    | AKAZE         | SIFT            | 36.9556497                | 12.5074602                 | 0.257156777777778        | 16.5734222259259  |
| 31    | BRISK         | FREAK           | 27.4238469                | 29.4807189                 | 0.412431444444444        | 19.1056657481481  |
| 32    | AKAZE         | AKAZE           | 35.8277093                | 28.3273411                 | 0.279182888888889        | 21.478077762963   |
| 33    | AKAZE         | FREAK           | 37.4454713                | 27.9017637                 | 0.246222888888889        | 21.864485962963   |
| 34    | SIFT          | FREAK           | 53.0200463                | 27.3133204                 | 0.181662111111111        | 26.838342937037   |
| 35    | SIFT          | SIFT            | 58.9512428                | 50.9683455                 | 0.249426222222222        | 36.7230048407407  |