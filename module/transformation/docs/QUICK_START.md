# UUV Trajectory Export - 快速使用指南

## 概述

transformation 模块现已支持轨迹数据的双格式导出：**JSON**（易读易调试）和**二进制**（高效存储）。

---

## 快速开始

### 1️⃣ 基础导出 - JSON 格式

```cpp
#include "transformation/transformation.hpp"

// 仿真运行后
if (g_pFormationSimulator) {
    // 导出为 JSON 字符串
    std::string json_data = g_pFormationSimulator->exportTrajectoryJSON();
    
    // 保存到文件
    std::ofstream file("trajectory.json");
    file << json_data;
    file.close();
    
    printf("✅ Exported to trajectory.json\n");
}
```

### 2️⃣ 高效导出 - 二进制格式

```cpp
#include "transformation/transformation.hpp"

// 仿真运行后
if (g_pFormationSimulator) {
    // 导出为二进制文件
    bool success = g_pFormationSimulator->exportTrajectoryBinary("trajectory.traj");
    
    if (success) {
        printf("✅ Exported to trajectory.traj\n");
    }
}
```

### 3️⃣ 获取仿真统计信息

```cpp
#include "transformation/transformation.hpp"

// 获取详细统计
Json::Value stats = g_pFormationSimulator->getTrajectoryStatistics();

printf("Total frames: %d\n", stats["total_frames"].asInt());
printf("Formation error - Max: %.3f m, Avg: %.3f m\n",
       stats["formation_error"]["max"].asDouble(),
       stats["formation_error"]["average"].asDouble());
printf("Average speed: %.2f m/s\n", stats["average_speed"].asDouble());
printf("Simulation duration: %.2f s\n", 
       stats["simulation_duration_s"].asDouble());
```

### 4️⃣ 获取当前仿真状态

```cpp
#include "transformation/transformation.hpp"

// 获取实时状态
Json::Value status = g_pFormationSimulator->getSimulationStatus();

printf("Current time: %.2f s\n", status["current_time"].asDouble());
printf("Current formation: %s\n", status["current_formation"].asCString());
printf("Active nodes: %d\n", status["node_count"].asInt());
printf("In transition: %s\n", status["is_transition"].asBool() ? "Yes" : "No");
```

---

## 详细说明

### 📊 何时使用 JSON vs 二进制

| 场景 | 推荐格式 | 理由 |
|------|---------|------|
| 实时 IPC 传输 | **JSON** | 易于文本协议处理 |
| 离线存储（大文件） | **二进制** | 文件大小 60-75% 更小 |
| 调试与分析 | **JSON** | 人类可读 |
| 跨平台兼容性 | **二进制** | 二进制格式固定（Little-Endian） |
| 数据库存储 | **两者** | 根据查询方式选择 |

### 🔍 文件格式对比

```
场景：5 个节点，1000 帧轨迹

JSON 格式：
  文件大小：~20 MB
  读取速度：慢（需要 JSON 解析）
  可读性：高（可用文本编辑器打开）
  编辑性：高（可直接修改数值）

二进制格式：
  文件大小：~5-6 MB（75% 压缩率）
  读取速度：快（直接内存映射）
  可读性：低（二进制数据）
  编辑性：低（需要特殊工具）
  随机访问：O(1)（通过帧索引表）
```

---

## 集成到现有工作流

### 在 Transformation_Use() 中集成

```cpp
void Transformation_Use(CalcTempParam& task_param) {
    if (g_pFormationSimulator == nullptr) {
        printf("仿真器未初始化！\n");
        return;
    }

    // 运行仿真
    UAVTrajectory trajectory_data = g_pFormationSimulator->step_simulation();
    auto all_trajectory = trajectory_data.getAllTrajectory();

    task_param.trajectory_result.clear();
    Json::Value frames_array(Json::arrayValue);
    task_param.run_frames = g_pFormationSimulator->getRunframe() * 10;

    if (all_trajectory.empty()) {
        task_param.trajectory_result["frames"] = frames_array;
        return;
    }

    // === 新增：导出轨迹数据 ===
    
    // 方案 1：通过 JSON 直接返回
    if (task_param.export_format == "json") {
        std::string json_data = g_pFormationSimulator->exportTrajectoryJSON();
        task_param.trajectory_result = Json::parseString(json_data);
        printf("✅ Exported trajectory as JSON\n");
        return;
    }
    
    // 方案 2：导出为二进制文件
    if (task_param.export_format == "binary") {
        bool success = g_pFormationSimulator->exportTrajectoryBinary(
            task_param.export_filename);
        task_param.trajectory_result["export_status"] = success ? "success" : "failed";
        task_param.trajectory_result["export_file"] = task_param.export_filename;
        printf("✅ Exported trajectory as binary\n");
        return;
    }

    // 原有逻辑（默认 JSON 返回）
    for (const auto& group : all_trajectory) {
        // ... 现有帧处理代码 ...
    }
    task_param.trajectory_result["frames"] = frames_array;
}
```

### 扩展 CalcTempParam 结构（可选）

```cpp
struct CalcTempParam {
    // 现有字段...
    
    // 新增导出配置字段
    std::string export_format;       // "json" | "binary"
    std::string export_filename;     // 输出文件路径
    bool include_metadata;           // 是否包含元数据
};
```

---

## Python 客户端使用示例

### 读取导出的 JSON 文件

```python
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取轨迹数据
with open("trajectory.json", "r", encoding="utf-8") as f:
    data = json.load(f)

# 提取配置信息
config = data["metadata"]["formation_config"]
print(f"Nodes: {config['node_num']}, Rel Distance: {config['rel_distance']} m")

# 可视化第一帧
first_frame = data["frames"][0]
print(f"Formation: {first_frame['formation_type']}")

# 提取节点位置
nodes = first_frame["nodes"]
x = [n["rel_x"] for n in nodes]
y = [n["rel_y"] for n in nodes]

# 2D 绘图
plt.figure(figsize=(8, 8))
plt.scatter(x, y, s=100, c=['red'] + ['blue']*(len(nodes)-1))
plt.xlabel("Rel-X (m)")
plt.ylabel("Rel-Y (m)")
plt.title(f"Formation: {first_frame['formation_type']}")
plt.grid()
plt.axis("equal")
plt.show()
```

### 读取导出的二进制文件

```python
import struct
import json

class TrajectoryReader:
    MAGIC = 0x54524A46  # "TRJF"
    VERSION = 0x0100
    
    def __init__(self, filename):
        self.filename = filename
        self.header = None
        self.metadata = None
        
    def read_header(self):
        with open(self.filename, 'rb') as f:
            data = f.read(64)
            (magic, version, _, frame_count, total_nodes, 
             sim_time, meta_size, meta_offset, 
             frame_idx_offset, frame_data_offset, *_) = struct.unpack(
                '<IHHIIQDIII', data[:44])
            
            if magic != self.MAGIC:
                raise ValueError("Invalid trajectory file")
            
            self.header = {
                'magic': hex(magic),
                'version': hex(version),
                'frame_count': frame_count,
                'total_nodes': total_nodes,
                'simulation_time': sim_time,
                'file_size': self._get_file_size()
            }
            
            return self.header
    
    def read_metadata(self):
        with open(self.filename, 'rb') as f:
            # 跳转到元数据位置
            data = f.read(64)
            meta_size = struct.unpack('<I', data[24:28])[0]
            meta_offset = struct.unpack('<I', data[28:32])[0]
            
            f.seek(meta_offset)
            meta_json = f.read(meta_size).decode('utf-8')
            self.metadata = json.loads(meta_json)
            
            return self.metadata
    
    def _get_file_size(self):
        import os
        return os.path.getsize(self.filename)

# 使用示例
reader = TrajectoryReader("trajectory.traj")
header = reader.read_header()
print(f"Frames: {header['frame_count']}, Nodes: {header['total_nodes']}")
print(f"Simulation time: {header['simulation_time']:.2f} s")
print(f"File size: {header['file_size'] / 1024 / 1024:.2f} MB")

metadata = reader.read_metadata()
print(f"Formation config: {metadata['formation_config']}")
```

---

## 常见问题

### Q1: JSON 和二进制文件如何选择？

**A**: 
- 实时通信或调试用 **JSON**（易读）
- 长期存储或大数据集用 **二进制**（节省空间 60-75%）

### Q2: 能否在运行时切换导出格式？

**A**: 可以。建议在 `CalcTempParam` 中添加 `export_format` 字段，由客户端指定格式。

### Q3: 二进制文件的兼容性如何保证？

**A**: 
- 魔数验证（0x54524A46）确保文件类型正确
- 版本号（0x0100）支持向后兼容性
- 使用 `#pragma pack(1)` 确保跨平台一致性

### Q4: 能否只导出部分帧？

**A**: 当前暂不支持，但可扩展。建议在第二阶段实现帧范围导出功能。

### Q5: 导出文件是否包含所有仿真参数？

**A**: 是的。JSON 的 `metadata.formation_config` 和二进制的元数据段都包含完整的配置信息。

---

## 性能建议

### 导出优化

```cpp
// ❌ 低效：多次调用 exportTrajectoryJSON()
std::string json1 = g_pFormationSimulator->exportTrajectoryJSON();
std::string json2 = g_pFormationSimulator->exportTrajectoryJSON();

// ✅ 高效：缓存结果
std::string json_cache = g_pFormationSimulator->exportTrajectoryJSON();
// 重复使用 json_cache
```

### 文件 I/O 优化

```cpp
// 二进制导出更快
bool success = g_pFormationSimulator->exportTrajectoryBinary("trajectory.traj");
// 时间复杂度：O(N)，大约 100-500ms（根据数据量）
```

---

## 相关文档

- 📖 [JSON Schema 完整规范](./JSON_SCHEMA.md)
- 📖 [Binary Format 详细设计](./BINARY_FORMAT.md)
- 📖 [实现总结](./IMPLEMENTATION_SUMMARY.md)
- 🔗 [Header 定义](../include/transformation/trajectory_serializer.hpp)

---

## 版本与支持

- **实现版本**：1.0
- **发布日期**：2026-06-03
- **支持平台**：Windows x64 (MSVC 2022)
- **依赖库**：JsonCpp

---

**需要帮助？** 查看相关文档或检查编译日志。
