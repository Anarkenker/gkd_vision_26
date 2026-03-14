# GKD2026 RMUL 视觉代码

感谢同济大学的开源， 本项目是基于同济大学开源框架改造的视觉算法库

## 依赖环境

编译要求：

- CMake `>= 3.16`
- C++17
- OpenCV
- fmt
- Eigen3
- spdlog
- yaml-cpp
- nlohmann_json
- OpenVINO 2024.6
- `usb-1.0`

本仓库还依赖以下厂商或外设 SDK/库：

- 海康工业相机 SDK

## 编译

### 1. 准备模型和 SDK

仓库默认使用以下模型文件：

- `assets/yolov5.xml`
- `assets/yolov5.bin`
- `assets/yolov8.xml`
- `assets/yolov8.bin`
- `assets/yolo11.xml`
- `assets/yolo11.bin`
- `assets/yolo11_buff_int8.xml`
- `assets/yolo11_buff_int8.bin`
- `assets/tiny_resnet.onnx`

OpenVINO 路径当前在顶层 `CMakeLists.txt` 中写死为：

```cmake
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
```

如果开发机安装路径不同，需要先改这里或在本地环境中覆盖。

### 2. 运行 CMake

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 运行示例

GKD 英雄：

```bash
./build/gkdhero configs/gkdhero.yaml
```

GKD 步兵：

```bash
./build/gkdinfantry configs/gkdinfantry.yaml
```

GKD 步兵 MPC：

```bash
./build/gkdinfantry_MPC configs/gkdinfantry.yaml
```

调试版程序通常可直接替换为对应的 `*_debug` 可执行文件。

当前仓库不再从配置文件读取敌方颜色。保留的 GKD 链路都会通过 `io/gkdcontrol` 从电控侧接收运行时姿态和敌方颜色。



## 代码链路概览

### 自瞄

`相机 -> 检测器/YOLO -> 解算器 -> 跟踪器 -> 瞄准器/规划器 -> 射击逻辑 -> 电控发送`

## 致谢

感谢同济大学开源的视觉框架与相关思路。
