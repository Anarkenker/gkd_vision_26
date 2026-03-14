# Calibration

这个目录下目前常用的标定程序有 3 个：

- `capture`：采集棋盘图片和对应四元数
- `calibrate_camera`：标定相机内参
- `calibrate_handeye`：标定手眼外参

当前代码默认使用：

- 海康相机
- `GKDControl` 接收姿态
- 棋盘角点检测 `findChessboardCorners`

## 1. 先准备配置

标定配置文件默认是 [configs/calibration.yaml](/Users/macbookpro/Documents/code/sp_vision_25/configs/calibration.yaml)。

目前需要关心的字段：

- `pattern_cols`：棋盘内角点列数，不是格子数
- `pattern_rows`：棋盘内角点行数，不是格子数
- `center_distance_mm`：相邻角点间距，单位 `mm`
- `R_gimbal2imubody`：云台系到 IMU 机体系的旋转矩阵
- `camera_matrix`：相机内参，先做完内参标定再填
- `distort_coeffs`：畸变系数，先做完内参标定再填

示例：

```yaml
pattern_cols: 11
pattern_rows: 8
center_distance_mm: 15

R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

camera_matrix: [...]
distort_coeffs: [...]
```

## 2. 编译

在仓库根目录执行：

```bash
cmake -S . -B build
cmake --build build -j
```

生成的标定程序是：

- `build/capture`
- `build/calibrate_camera`
- `build/calibrate_handeye`

## 3. 标定内参

### 3.1 采集图片

如果你直接用这套代码采图：

```bash
./build/capture configs/calibration.yaml -o assets/calib_intrinsic
```

运行时说明：

- 界面会显示棋盘检测结果
- 按 `s` 保存当前帧
- 按 `q` 退出
- 每次保存会生成一对文件：`1.jpg`、`1.txt`

说明：

- `calibrate_camera` 只读取 `jpg`，不会使用对应的 `txt`
- 所以内参标定时，即使用 `capture` 采集出来带了四元数文件，也没关系

建议：

- 至少采 `15` 到 `30` 张
- 棋盘尽量覆盖画面中心、四角、边缘
- 需要有远近变化和较大倾斜角
- 不要只拍几张几乎一样的图

### 3.2 运行内参标定

```bash
./build/calibrate_camera assets/calib_intrinsic -c configs/calibration.yaml
```

程序会逐张显示角点检测结果，并在终端输出：

- `camera_matrix`
- `distort_coeffs`
- 重投影误差

把输出的 `camera_matrix` 和 `distort_coeffs` 填回 [configs/calibration.yaml](/Users/macbookpro/Documents/code/sp_vision_25/configs/calibration.yaml)。

## 4. 标定手眼外参

### 4.1 采集图片和姿态

```bash
./build/capture configs/calibration.yaml -o assets/calib_handeye
```

`capture` 会同时保存：

- 图像：`1.jpg`, `2.jpg`, ...
- 四元数：`1.txt`, `2.txt`, ...

四元数文件格式是：

```text
w x y z
```

注意：

- 文件编号必须连续，不能缺号
- `calibrate_handeye` 会从 `1.jpg` 一直读到遇到第一张缺失图片为止

### 4.2 采集姿态建议

手眼标定时建议：

- 棋盘固定不动
- 云台或整机去转动
- 采样姿态要分散，不能只左右小幅摆动
- 尽量覆盖不同 `yaw/pitch`
- 至少保留 `10` 组以上有效样本，越多越稳

如果 `R_gimbal2imubody` 本身填错，手眼结果也会跟着错。

### 4.3 运行手眼标定

先确保 [configs/calibration.yaml](/Users/macbookpro/Documents/code/sp_vision_25/configs/calibration.yaml) 里已经有正确的：

- `camera_matrix`
- `distort_coeffs`

然后执行：

```bash
./build/calibrate_handeye assets/calib_handeye -c configs/calibration.yaml
```

程序会输出：

- `R_camera2gimbal`
- `t_camera2gimbal`

其中：

- `R_camera2gimbal` 是旋转矩阵
- `t_camera2gimbal` 单位是 `m`

把这两个填回实际运行配置，例如：

- [configs/gkdinfantry.yaml](/Users/macbookpro/Documents/code/sp_vision_25/configs/gkdinfantry.yaml)
- [configs/gkdhero.yaml](/Users/macbookpro/Documents/code/sp_vision_25/configs/gkdhero.yaml)

## 5. 常见问题

### 棋盘尺寸怎么填

`pattern_cols` / `pattern_rows` 填的是内角点数，不是黑白格数量。

例如棋盘如果是 `12 x 9` 个格子，那么内角点通常是：

```yaml
pattern_cols: 11
pattern_rows: 8
```

### `center_distance_mm` 填什么

填相邻角点之间的距离，也就是单个格子的边长，单位 `mm`。

### 为什么内参标定能用 `capture` 采的图

因为 [calibrate_camera.cpp](/Users/macbookpro/Documents/code/sp_vision_25/calibration/calibrate_camera.cpp) 只读取 `jpg`，不会读取四元数文件。

### 为什么手眼标定失败

常见原因：

- 内参还没填对
- 棋盘尺寸填错
- 采样姿态变化太小
- 图片编号不连续
- `R_gimbal2imubody` 不正确
- 棋盘检测或 `solvePnP` 失败的有效样本太少

### 终端输出如何使用

标定程序目前都是把结果打印到终端，不会自动回写 YAML。你需要把输出手动拷回配置文件。

## 6. 其他程序

- `calibrate_robotworld_handeye`：更复杂的机器人世界手眼标定，普通相机到云台外参标定一般不用它
- `split_video`：把已有录像按区间裁出来，做数据整理时可选
