# UUV Trajectory JSON Schema

## 概述
UUV 编队轨迹数据采用 JSON 格式存储，支持元数据、仿真配置、逐帧节点状态等信息。

## 根结构

```json
{
  "metadata": { /* 元数据 */ },
  "frames": [ /* 帧数据数组 */ ]
}
```

---

## 元数据 (metadata)

```json
{
  "metadata": {
    "simulator_version": "1.0",
    "simulation_time_s": 100.5,
    "total_frames": 1005,
    "frame_interval_s": 0.1,
    "formation_config": { /* 仿真配置 */ }
  }
}
```

### 字段说明

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `simulator_version` | string | - | 仿真器版本号 (e.g. "1.0") |
| `simulation_time_s` | number | 秒 | 总仿真时长 |
| `total_frames` | integer | - | 轨迹总帧数 |
| `frame_interval_s` | number | 秒 | 帧时间间隔 (= sim_step) |
| `formation_config` | object | - | 仿真配置参数对象 |

---

## 仿真配置 (formation_config)

```json
{
  "formation_config": {
    "node_num": 5,
    "rel_distance": 50.0,
    "collision_radius": 4.0,
    "init_speed": 5.0,
    "init_heading": 0.0,
    "heading_rate": 0.0,
    "acceleration": 0.0,
    "sim_step": 0.1,
    "max_frames": 1000,
    "return_frames": 1000,
    "main_node": {
      "lon": 120.123456,
      "lat": 30.654321
    }
  }
}
```

### 字段说明

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `node_num` | integer | - | 节点总数 (2-10) |
| `rel_distance` | number | 米 | 节点间距 (默认队形生成参数) |
| `collision_radius` | number | 米 | 碰撞检测半径 |
| `init_speed` | number | m/s | 初始速度 |
| `init_heading` | number | 度 | 初始航向 (0-360) |
| `heading_rate` | number | 度/秒 | 航向变化率 (转弯速率) |
| `acceleration` | number | m/s² | 加速度 |
| `sim_step` | number | 秒 | 仿真时间步长 |
| `max_frames` | integer | - | 最大仿真帧数 |
| `return_frames` | integer | - | 返回帧数 |
| `main_node.lon` | number | 度 | 主节点初始经度 |
| `main_node.lat` | number | 度 | 主节点初始纬度 |

---

## 帧数据 (frames)

```json
{
  "frames": [
    {
      "frame_id": 0,
      "timestamp_s": 0.0,
      "formation_type": "Rectangle",
      "nodes": [ /* 节点数组 */ ]
    },
    {
      "frame_id": 1,
      "timestamp_s": 0.1,
      "formation_type": "Rectangle",
      "nodes": [ /* ... */ ]
    }
  ]
}
```

### 帧头字段

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `frame_id` | integer | - | 帧编号 (从0开始) |
| `timestamp_s` | number | 秒 | 相对时间戳 |
| `formation_type` | string | - | 队形类型: "Line", "Rectangle", "Circle", "Diamond", "Triangle" |
| `nodes` | array | - | 该帧的节点数据数组 |

---

## 节点数据 (nodes[])

```json
{
  "nodes": [
    {
      "node_id": 0,
      "type": "leader",
      "lon": 120.123456,
      "lat": 30.654321,
      "speed_ms": 5.0,
      "heading_deg": 45.0,
      "rel_x": 0.0,
      "rel_y": 0.0,
      "target_x": 0.0,
      "target_y": 0.0,
      "status": {
        "is_joining": false,
        "is_leaving": false,
        "join_progress": 0.0
      }
    },
    {
      "node_id": 1,
      "type": "follower",
      "lon": 120.124,
      "lat": 30.655,
      "speed_ms": 4.8,
      "heading_deg": 45.5,
      "rel_x": 25.5,
      "rel_y": -42.3,
      "target_x": 50.0,
      "target_y": -50.0,
      "status": {
        "is_joining": false,
        "is_leaving": false,
        "join_progress": 0.0
      }
    }
  ]
}
```

### 节点字段

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `node_id` | integer | - | 节点编号 (0=主节点，1+=从节点) |
| `type` | string | - | 节点类型: "leader" 或 "follower" |
| `lon` | number | 度 | 地理坐标：经度 (精度 1e-6) |
| `lat` | number | 度 | 地理坐标：纬度 (精度 1e-6) |
| `speed_ms` | number | m/s | 当前速度 (精度 1e-3) |
| `heading_deg` | number | 度 | 航向角 (0-360, 精度 1e-3) |
| `rel_x` | number | 米 | 相对主节点 X 坐标 (ENU系，精度 1e-3) |
| `rel_y` | number | 米 | 相对主节点 Y 坐标 (ENU系，精度 1e-3) |
| `target_x` | number | 米 | 目标相对 X 坐标 (精度 1e-3) |
| `target_y` | number | 米 | 目标相对 Y 坐标 (精度 1e-3) |
| `status.is_joining` | boolean | - | 是否正在加入编队 |
| `status.is_leaving` | boolean | - | 是否正在脱离编队 |
| `status.join_progress` | number | - | 加入进度 (0.0-1.0) |

---

## 完整示例

```json
{
  "metadata": {
    "simulator_version": "1.0",
    "simulation_time_s": 10.0,
    "total_frames": 101,
    "frame_interval_s": 0.1,
    "formation_config": {
      "node_num": 3,
      "rel_distance": 50.0,
      "collision_radius": 4.0,
      "init_speed": 5.0,
      "init_heading": 0.0,
      "heading_rate": 0.0,
      "acceleration": 0.0,
      "sim_step": 0.1,
      "max_frames": 1000,
      "return_frames": 1000,
      "main_node": {
        "lon": 120.0,
        "lat": 30.0
      }
    }
  },
  "frames": [
    {
      "frame_id": 0,
      "timestamp_s": 0.0,
      "formation_type": "Line",
      "nodes": [
        {
          "node_id": 0,
          "type": "leader",
          "lon": 120.0,
          "lat": 30.0,
          "speed_ms": 5.0,
          "heading_deg": 0.0,
          "rel_x": 0.0,
          "rel_y": 0.0,
          "target_x": 0.0,
          "target_y": 0.0,
          "status": {
            "is_joining": false,
            "is_leaving": false,
            "join_progress": 0.0
          }
        },
        {
          "node_id": 1,
          "type": "follower",
          "lon": 120.0,
          "lat": 29.9995,
          "speed_ms": 5.0,
          "heading_deg": 0.0,
          "rel_x": 0.0,
          "rel_y": -50.0,
          "target_x": 0.0,
          "target_y": -50.0,
          "status": {
            "is_joining": false,
            "is_leaving": false,
            "join_progress": 0.0
          }
        },
        {
          "node_id": 2,
          "type": "follower",
          "lon": 120.0,
          "lat": 29.999,
          "speed_ms": 5.0,
          "heading_deg": 0.0,
          "rel_x": 0.0,
          "rel_y": -100.0,
          "target_x": 0.0,
          "target_y": -100.0,
          "status": {
            "is_joining": false,
            "is_leaving": false,
            "join_progress": 0.0
          }
        }
      ]
    }
  ]
}
```

---

## 精度说明

- **经纬度 (lon/lat)**：精确到 0.000001 度（约 0.1 米）
- **相对坐标 (rel_x, rel_y, target_x, target_y)**：精确到 0.001 米
- **速度 (speed_ms)**：精确到 0.001 m/s
- **航向 (heading_deg)**：精确到 0.001 度
- **加入进度 (join_progress)**：精确到 0.001 (千分位)

---

## 使用场景

### 1. 外部应用读取轨迹数据

```python
import json

with open("trajectory.json", "r", encoding="utf-8") as f:
    data = json.load(f)

# 获取仿真配置
config = data["metadata"]["formation_config"]

# 遍历所有帧
for frame in data["frames"]:
    print(f"Frame {frame['frame_id']}: {frame['formation_type']}")
    for node in frame["nodes"]:
        print(f"  Node {node['node_id']}: ({node['lon']}, {node['lat']})")
```

### 2. 可视化应用用法

```python
# 基于轨迹数据进行 3D 绘图、动画播放等
import matplotlib.pyplot as plt

# 提取第一帧的节点位置
first_frame = data["frames"][0]
nodes = first_frame["nodes"]
xs = [n["rel_x"] for n in nodes]
ys = [n["rel_y"] for n in nodes]

plt.scatter(xs, ys)
plt.title(f"Formation: {first_frame['formation_type']}")
plt.show()
```

---

## 文件大小估算

- **单个节点单帧数据**：~200 字节 (包括 JSON 格式开销)
- **100 个节点，1000 帧**：~20 MB
- **压缩后（二进制格式）**：~5-8 MB (压缩率 60-75%)

推荐：
- **帧数 < 100，节点 < 5**：使用 JSON
- **帧数 > 100，节点 > 5**：使用二进制格式
