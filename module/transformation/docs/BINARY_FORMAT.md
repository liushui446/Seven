# UUV Trajectory Binary Format Specification

## 概述

UUV 编队轨迹二进制格式（`.traj`）设计用于高效存储和快速加载大规模轨迹数据。相比 JSON 格式，二进制格式可减少 60-75% 的文件大小，并支持随机访问。

---

## 文件结构总览

```
┌─────────────────────────────────┐
│  File Header (64 bytes)         │ offset: 0
├─────────────────────────────────┤
│  Metadata (variable size)       │ offset: 64
├─────────────────────────────────┤
│  Frame Index Table (4*N bytes)  │ 
├─────────────────────────────────┤
│  Frame Data (frames)            │
│  ├─ Frame 0                     │
│  ├─ Frame 1                     │
│  └─ ...                         │
└─────────────────────────────────┘
```

---

## 1. 文件头 (TrajectoryFileHeader)

**大小**：64 字节 | **对齐**：无 | **字节序**：Little-Endian

```cpp
struct TrajectoryFileHeader {
    uint32_t magic;              // +0  (4 bytes)  - 魔数: 0x54524A46 ("TRJF")
    uint16_t version;            // +4  (2 bytes)  - 版本: 0x0100 (v1.0)
    uint16_t reserved1;          // +6  (2 bytes)  - 保留字段
    uint32_t frame_count;        // +8  (4 bytes)  - 总帧数
    uint32_t total_nodes;        // +12 (4 bytes)  - 所有帧中的总节点数（累计）
    double   simulation_time;    // +16 (8 bytes)  - 总仿真时长 (秒)
    uint32_t metadata_size;      // +24 (4 bytes)  - 元数据大小 (字节)
    uint32_t metadata_offset;    // +28 (4 bytes)  - 元数据在文件中的偏移量
    uint32_t frame_index_offset; // +32 (4 bytes)  - 帧索引表偏移量
    uint32_t frame_data_offset;  // +36 (4 bytes)  - 帧数据起始偏移量
    uint32_t reserved2[2];       // +40 (8 bytes)  - 未来扩展预留
};  // 总大小：64 字节
```

### 字段说明

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| +0 | magic | uint32 | 魔数 `0x54524A46`（"TRJF" ASCII） |
| +4 | version | uint16 | 版本号：`0x0100` (v1.0) |
| +6 | reserved1 | uint16 | 保留，读取时忽略 |
| +8 | frame_count | uint32 | 轨迹总帧数 |
| +12 | total_nodes | uint32 | 所有帧中节点总数（去重前） |
| +16 | simulation_time | double | 仿真总时长（秒） |
| +24 | metadata_size | uint32 | 元数据 JSON 字符串长度 |
| +28 | metadata_offset | uint32 | 元数据开始位置（绝对偏移） |
| +32 | frame_index_offset | uint32 | 帧索引表开始位置 |
| +36 | frame_data_offset | uint32 | 帧数据开始位置 |
| +40 | reserved2[0] | uint32 | 扩展预留 |
| +44 | reserved2[1] | uint32 | 扩展预留 |

---

## 2. 元数据段 (Metadata)

**位置**：由 header.metadata_offset 指定
**大小**：由 header.metadata_size 指定  
**格式**：JSON 字符串（UTF-8 编码）

包含仿真配置信息，结构与 JSON Schema 中的 `formation_config` 相同：

```json
{
  "version": "1.0",
  "simulation_time_s": 100.5,
  "total_frames": 1005,
  "frame_interval_s": 0.1,
  "formation_config": {
    "node_num": 5,
    "rel_distance": 50.0,
    "collision_radius": 4.0,
    "init_speed": 5.0,
    ...
  }
}
```

---

## 3. 帧索引表 (Frame Index Table)

**位置**：由 header.frame_index_offset 指定
**大小**：4 × frame_count 字节
**对齐**：4 字节对齐

```cpp
// 帧索引表结构（动态数组）
struct FrameIndexEntry {
    uint32_t frame_offset;  // 该帧在文件中的绝对偏移量 (字节)
};

// 索引表 = FrameIndexEntry[frame_count]
```

**用途**：支持随机访问特定帧，无需读取前面所有帧。

**使用例**：
```cpp
// 读取第 N 帧
uint32_t offset_n = frame_index[N];
seekg(offset_n);  // 直接跳转到第 N 帧
```

---

## 4. 帧数据段 (Frame Data)

**位置**：由 header.frame_data_offset 指定
**结构**：顺序存储多个帧

### 单个帧结构

```
┌─────────────────────────┐
│ FrameHeader (20 bytes)  │
├─────────────────────────┤
│ Node 0 (128 bytes)      │
├─────────────────────────┤
│ Node 1 (128 bytes)      │
├─────────────────────────┤
│ ...                     │
├─────────────────────────┤
│ Node N-1 (128 bytes)    │
└─────────────────────────┘
```

#### 4.1 帧头 (FrameHeader)

**大小**：20 字节 | **字节序**：Little-Endian

```cpp
struct FrameHeader {
    int32_t  frame_id;          // +0 (4 bytes)  - 帧编号
    int32_t  timestamp_ms;      // +4 (4 bytes)  - 时间戳 (毫秒)
    uint32_t formation_type;    // +8 (4 bytes)  - 队形类型 (enum: 0=Line, 1=Rectangle, ...)
    uint16_t node_count;        // +12 (2 bytes) - 该帧的节点数
    uint16_t reserved;          // +14 (2 bytes) - 保留字段
    uint32_t reserved2;         // +16 (4 bytes) - 未来扩展
};  // 总大小：20 字节
```

**队形类型枚举**：
- `0` = Line（线形）
- `1` = Rectangle（矩形）
- `2` = Circle（圆形）
- `3` = Diamond（菱形）
- `4` = Triangle（三角形）

#### 4.2 节点数据块 (NodeBinaryData)

**大小**：128 字节 | **对齐**：无（#pragma pack(1)） | **字节序**：Little-Endian

```cpp
#pragma pack(push, 1)
struct NodeBinaryData {
    // 标识符 (4 bytes)
    int32_t node_id;            // +0 (4 bytes)

    // 位置坐标 (48 bytes)
    double  lon_deg;            // +4 (8 bytes)
    double  lat_deg;            // +12 (8 bytes)
    double  rel_x;              // +20 (8 bytes)
    double  rel_y;              // +28 (8 bytes)
    double  target_x;           // +36 (8 bytes)
    double  target_y;           // +44 (8 bytes)

    // 运动状态 (24 bytes)
    double  speed_ms;           // +52 (8 bytes)
    double  heading_deg;        // +60 (8 bytes)
    double  formation_error;    // +68 (8 bytes)

    // 状态标志 (8 bytes)
    uint8_t is_joining;         // +76 (1 byte)
    uint8_t is_leaving;         // +77 (1 byte)
    uint8_t reserved1;          // +78 (1 byte)
    uint8_t reserved2;          // +79 (1 byte)
    float   join_progress;      // +80 (4 bytes)

    // 保留 (44 bytes)
    uint8_t reserved[44];       // +84 (44 bytes)
};  // 总大小：128 字节
#pragma pack(pop)
```

---

## 5. 数据精度与范围

| 字段 | 类型 | 范围 | 精度 |
|------|------|------|------|
| node_id | int32 | -2B ~ +2B | 1 |
| lon_deg, lat_deg | double | -180 ~ +180 | 1e-16 度（实际 1e-6） |
| rel_x, rel_y, target_x, target_y | double | -1M ~ +1M 米 | 1e-16 米（实际 1e-3） |
| speed_ms | double | 0 ~ 1000 m/s | 1e-16 m/s（实际 1e-3） |
| heading_deg | double | 0 ~ 360 度 | 1e-16 度（实际 1e-3） |
| formation_error | double | 0 ~ 10000 米 | 1e-16 米 |
| join_progress | float | 0 ~ 1 | 1e-7（单精度） |
| timestamp_ms | int32 | 0 ~ 2147483647 ms | 1 ms |

---

## 6. 读写流程

### 写入流程

```cpp
// 1. 创建和初始化文件头
TrajectoryFileHeader header = {};
header.magic = 0x54524A46;
header.version = 0x0100;
header.frame_count = frames.size();
header.simulation_time = total_time;

// 2. 写入文件头（64 字节）
file.write((char*)&header, sizeof(TrajectoryFileHeader));

// 3. 写入元数据 JSON
string metadata_json = GenerateMetadata();
header.metadata_size = metadata_json.size();
header.metadata_offset = 64;
file.write(metadata_json.c_str(), metadata_json.size());

// 4. 写入帧索引表（占位）
header.frame_index_offset = file.tellp();
vector<uint32_t> frame_offsets;
for (auto& frame : frames) {
    frame_offsets.push_back(file.tellp());
    file.write((char*)&frame_offset, 4);  // 占位
}

// 5. 写入帧数据
header.frame_data_offset = file.tellp();
for (auto& frame : frames) {
    // 写帧头
    FrameHeader fh = CreateFrameHeader(frame);
    file.write((char*)&fh, sizeof(FrameHeader));
    
    // 写节点
    for (auto& node : frame.nodes) {
        NodeBinaryData nb = ConvertNodeToBinary(node);
        file.write((char*)&nb, sizeof(NodeBinaryData));
    }
}

// 6. 更新文件头
file.seekp(0);
file.write((char*)&header, sizeof(TrajectoryFileHeader));

// 7. 更新帧索引表
file.seekp(header.frame_index_offset);
for (uint32_t offset : frame_offsets) {
    file.write((char*)&offset, 4);
}
```

### 读取流程

```cpp
// 1. 读文件头
TrajectoryFileHeader header;
file.read((char*)&header, sizeof(TrajectoryFileHeader));

// 验证魔数
if (header.magic != 0x54524A46) {
    throw std::runtime_error("Invalid file format");
}

// 2. 读元数据
file.seekg(header.metadata_offset);
string metadata_json(header.metadata_size, '\0');
file.read(&metadata_json[0], header.metadata_size);

// 3. 读帧索引表
file.seekg(header.frame_index_offset);
vector<uint32_t> frame_offsets(header.frame_count);
file.read((char*)frame_offsets.data(), header.frame_count * 4);

// 4. 按需读帧
for (int i = 0; i < header.frame_count; i++) {
    file.seekg(frame_offsets[i]);
    
    // 读帧头
    FrameHeader fh;
    file.read((char*)&fh, sizeof(FrameHeader));
    
    // 读节点
    vector<UUVNode> nodes;
    for (int j = 0; j < fh.node_count; j++) {
        NodeBinaryData nb;
        file.read((char*)&nb, sizeof(NodeBinaryData));
        nodes.push_back(ConvertNodeFromBinary(nb));
    }
}
```

---

## 7. 文件大小估算

**场景**：5 个节点，1000 帧

```
文件头：               64 bytes
元数据：              ~500 bytes  (JSON string)
帧索引表：          4,000 bytes  (1000 × 4)
帧数据：        6,400,000 bytes  (1000 × (20 + 5×128))
─────────────────────────────────
总计：         ~6.4 MB  (二进制)
vs JSON:       ~20 MB   (原始)
压缩率：       68%
```

---

## 8. 兼容性与扩展

- **版本检查**：读取时验证 `header.version`
- **保留字段**：用于向后兼容性
- **魔数检查**：防止意外读取非法文件
- **元数据 JSON**：可扩展，新字段向后兼容

---

## 9. 性能优化建议

| 操作 | 时间复杂度 | 说明 |
|------|-----------|------|
| 读取第 N 帧 | O(1) | 使用帧索引表直接定位 |
| 读取所有帧 | O(N) | 顺序读取 |
| 随机访问 | O(1) | 利用固定帧大小 |
| 序列化 | O(N) | 单次通过写入 |

---

## 10. 调试与验证

### 文件验证工具伪代码

```cpp
bool VerifyBinaryFile(const string& filename) {
    ifstream file(filename, ios::binary);
    TrajectoryFileHeader header;
    file.read((char*)&header, sizeof(TrajectoryFileHeader));
    
    // 检查魔数
    assert(header.magic == 0x54524A46);
    
    // 检查版本
    assert(header.version == 0x0100);
    
    // 检查偏移量合理性
    assert(header.metadata_offset == 64);
    assert(header.frame_index_offset > header.metadata_offset + header.metadata_size);
    assert(header.frame_data_offset > header.frame_index_offset);
    
    // 检查帧数据完整性
    size_t expected_size = header.frame_data_offset + 
        header.frame_count * (sizeof(FrameHeader) + header.avg_nodes_per_frame * 128);
    assert(file_size >= expected_size);
    
    return true;
}
```

---

## 参考资源

- **JSON Schema**：见 `JSON_SCHEMA.md`
- **C++ 实现**：`trajectory_serializer.hpp` / `trajectory_serializer.cpp`
- **单位系统**：
  - 距离：米（m）
  - 角度：度（°）或弧度（rad）
  - 速度：米/秒（m/s）
  - 时间：秒（s）或毫秒（ms）
  - 坐标系：ENU（东北上）/ WGS84 经纬度
