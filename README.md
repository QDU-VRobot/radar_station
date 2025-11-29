# Radar (station)
Detect robots by their cars and armors with monocular camera and YOLO  
Designed for and tested on Linux (Serial needs to be rewritten for Windows)

## Flow:
Assume all functions are enabled
```mermaid
graph TB
    subgraph Mark[Mark points]
        direction TB
        M1[/Input a frame/] -->|Press other key| M2[Adjust real height] <--> M3[Mark points for PnP] -->|Confirm| M4[Adjust height for contours] <--> M5[Mark contours] -->|Confirm| M6["cv::solvePnP()"] --> M7[\Save config\]
        M1 -->|Press c| M10[/Input PnP pos/] --> M11[/Input contours pos/] --> M6
        M3 -->|Middle click| M8[Clear] --> M2; M5 -->|Middle click| M9[Clear] --> M4
    end
    subgraph Infer
        direction TB
        I0[/Read image/] --> I1{Car inference is done?} -->|true| I2[Get car result] --> I3[Update tracker] --> I4[Get car ROI] --> I5[Armor inference] -->I6{Image is empty?} -->|false| I7[Car inference with thread pool] --> I8[\Inference result\]
        I1 -->|false| I6; I6 -->|true| I8
    end
    subgraph Output
        direction TB
        O0[/Read results/] --> O1[Calculate pos] --> O2{Serial port is busy?} -->|false| O3[\Send pos to serial port\] --> O4[Draw on result images] --> O5[\Record image\]
        O2 -->|true| O4
    end
    1(Start) --> 2[/Parse commandline/] --> 3{TensorRT engine available?} -->|false| 4[Generate TensorRT engine] --> 5[Open serial port] --> 6[Open video with config] --> 7{Point data available?} -->|false| M1
    3 -->|true| 5; 5 -->|New thread| 15((Serial receiver)); 7 -->|true| 8
    M7 --> 8[Load models] --> 9[Register signal] --> 10[/Capture a frame/] --> 11{Captured a frame?}
    11 -->|false| 13[Stop video] --> 14(exit); 11 -->|true| 12[Undistort] --> I0
    I8 --> O0; O5 --> 10
```

## Usage:
1. Install a compiler that supports C++17 and dependencies below (plus **git** and **git-lfs** for cloning and **cmake** for building)
2. Clone this repository **(Make sure models are cloned too, or train your own model)**
3. Run `cmake -B build` to generate a *build* folder
4. Run `cmake --build build/ -j` to compile
5. Check executables and libraries in *build* folder
6. Use the calibration executable to generate camera config (`build/bin/calibration`)
7. Add more parameters by checking help output (`build/bin/radar --help`)
8. Set up camera and serial port (based on RoboMaster 2026 serial protocol V1.0.0)
9. Mark points on first startup (first contour for PnP and other contours for highlands)
10. Check result

## Bundled libraries: (Compiled along with main program)
* [TensorRT-YOLO](https://github.com/laugh12321/TensorRT-YOLO)
* [ByteTrack](https://github.com/ifzhang/ByteTrack)

## External libraries: (Use package manager to install them)
* [OpenCV](https://opencv.org/releases)
* [Eigen3](https://gitlab.com/libeigen/eigen)
* [CUDA & TensorRT](https://developer.nvidia.com/cuda-downloads)
* [MVS](https://www.hikrobotics.com/cn/machinevision/service/download) (For HikCamera)

The ONNX files are built by [trtyolo](https://github.com/laugh12321/TensorRT-YOLO/blob/main/docs/cn/model_export.md) with plugins and dynamic batch

## Result:
In RoboMaster 2025, this program vulnerated opponents for 338.1s per game in average

## Special thanks:
Datasets provided by [Taurus](https://github.com/SCAU-RM-NAV/RM2023_Radar_Dataset)

## TODO:
* [ ] Train a better model with videos recorded
* [ ] Modify tracker's parameters to improve stability
* [ ] Adjust vulnerabilitiy's trigger
* [ ] Cooperate with a LiDAR
* [ ] Behavior-based guess
* [ ] **NEW** Radio decrypt
