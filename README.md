# ats_livox_adapter

PointCloud2 到 `livox_ros_driver2/CustomMsg` 的格式转换，供 MuJoCo 与 Gazebo 仿真喂给 Point-LIO。

Point-LIO 以 `preprocess.lidar_type: 1` 订阅 `CustomMsg`，仿真器只能便宜地输出 PointCloud2；在 rclpy 里逐点构造
`CustomPoint` 每帧约 121 ms（见 `rmu_gazebo_simulator/src/gz_livox_bridge.cpp` 的实测注释），所以转换放在 C++。

## 输入契约

`sensor_msgs/PointCloud2`，每点 24 字节：

| 字段 | 类型 | 偏移 | 含义 |
| --- | --- | --- | --- |
| `x` `y` `z` | float32 | 0 4 8 | 传感器系坐标（必需） |
| `intensity` | float32 | 12 | 转成 `reflectivity`（可选，缺省 100） |
| `offset_time` | uint32 | 16 | 相对 `header.stamp` 的发射时间，纳秒（可选，缺省 0） |
| `line` | uint8 | 20 | 扫描线号（可选，缺省 0） |
| `tag` | uint8 | 21 | 点标签（可选，缺省 0） |

转换只改格式：时间戳原样保留，`timebase` 取 `header.stamp`，非有限坐标的点被丢弃但不重排其他点的 `offset_time`。
距离过滤默认关闭（`min_range`/`max_range` 参数可开）。

## 使用

```bash
ros2 run ats_livox_adapter pointcloud_to_livox_node --ros-args \
  -p input_topic:=/mujoco/livox_points -p output_topic:=/livox/lidar -p frame_id:=front_mid360
```

也可作为组件加载：`ats_livox_adapter::PointCloudToLivoxNode`。

## 构建与测试

```bash
colcon build --base-paths src --packages-select ats_livox_adapter
build/ats_livox_adapter/test_convert
```

依赖 `livox_ros_driver2`（工作区内的 `src/ats_sentry_nav/livox_ros_driver2`）。许可证 Apache-2.0。
