#ifndef TRAJECTORY_SERIALIZER_HPP
#define TRAJECTORY_SERIALIZER_HPP

#include "core/CommonCore.hpp"
#include "transformation.hpp"
#include <fstream>
#include <cstring>

namespace seven {

    // ==================== 二进制格式常量 ====================
    const uint32_t TRAJECTORY_MAGIC = 0x54524A46;  // "TRJF" in hex
    const uint16_t TRAJECTORY_VERSION = 0x0100;    // v1.00
    const uint32_t NODE_BINARY_SIZE = 128;         // 固定128字节/节点

    // ==================== 二进制格式结构定义 ====================

    /**
     * @brief 轨迹文件头 (64 bytes)
     */
    struct TrajectoryFileHeader {
        uint32_t magic;              // 魔数: 0x54524A46 ("TRJF")
        uint16_t version;            // 版本: 0x0100 (1.0)
        uint16_t reserved1;          // 保留字段
        uint32_t frame_count;        // 总帧数
        uint32_t total_nodes;        // 所有节点总数（去重）
        double   simulation_time;    // 总仿真时长 (秒)
        uint32_t metadata_size;      // 元数据大小 (字节)
        uint32_t metadata_offset;    // 元数据在文件中的偏移量
        uint32_t frame_index_offset; // 帧索引表偏移量
        uint32_t frame_data_offset;  // 帧数据起始偏移量
        // 扩展字段用于未来使用
        uint32_t reserved2[2];
    };

    /**
     * @brief 帧头信息 (20 bytes)
     */
    struct FrameHeader {
        int      frame_id;          // 帧编号
        int      timestamp_ms;      // 时间戳 (毫秒)
        uint32_t formation_type;    // 队形类型
        uint16_t node_count;        // 该帧节点数
        uint16_t reserved;
    };

    /**
     * @brief 节点二进制数据块 (128 bytes)
     * 设计为固定大小以支持快速随机访问
     */
#pragma pack(push, 1)
    struct NodeBinaryData {
        // 标识符 (4 bytes)
        int32_t node_id;

        // 位置坐标 (48 bytes)
        double  lon_deg;            // 经度 (度)
        double  lat_deg;            // 纬度 (度)
        double  rel_x;              // 相对X坐标 (米)
        double  rel_y;              // 相对Y坐标 (米)
        double  target_x;           // 目标X坐标 (米)
        double  target_y;           // 目标Y坐标 (米)

        // 运动状态 (24 bytes)
        double  speed_ms;           // 速度 (m/s)
        double  heading_deg;        // 航向 (度)
        double  formation_error;    // 队形误差 (米)

        // 状态标志和进度 (8 bytes)
        uint8_t is_joining;         // 正在加入
        uint8_t is_leaving;         // 正在脱离
        uint8_t reserved1;
        uint8_t reserved2;
        float   join_progress;      // 加入进度 (0.0-1.0)

        // 保留用于扩展 (44 bytes)
        uint8_t reserved[44];
    };
#pragma pack(pop)

    // 验证大小
    static_assert(sizeof(NodeBinaryData) == NODE_BINARY_SIZE,
        "NodeBinaryData size must be exactly 128 bytes");

    /**
     * @brief 轨迹序列化器 - 支持JSON和二进制两种格式
     */
    class TrajectorySerializer {
    public:
        /**
         * @brief 将轨迹数据序列化为JSON字符串
         * @param trajectory 轨迹数据
         * @param config 仿真配置
         * @param include_metadata 是否包含元数据
         * @return JSON字符串
         */
        static std::string toJSON(
            const UAVTrajectory& trajectory,
            const FormationConfig& config,
            bool include_metadata = true
        );

        /**
         * @brief 将轨迹数据序列化为二进制文件
         * @param filename 输出文件路径
         * @param trajectory 轨迹数据
         * @param config 仿真配置
         * @return 是否成功
         */
        static bool toBinary(
            const std::string& filename,
            const UAVTrajectory& trajectory,
            const FormationConfig& config
        );

        /**
         * @brief 从二进制文件反序列化轨迹数据
         * @param filename 输入文件路径
         * @param out_trajectory 输出轨迹数据 (注：实际应用需扩展UAVTrajectory支持此操作)
         * @param out_header 输出文件头信息
         * @return 是否成功
         */
        static bool fromBinary(
            const std::string& filename,
            std::vector<TrajectoryFrame>& out_frames,
            TrajectoryFileHeader& out_header
        );

        /**
         * @brief 获取二进制文件的元数据
         * @param filename 输入文件路径
         * @return Json::Value 包含元数据的JSON对象
         */
        static Json::Value getBinaryMetadata(const std::string& filename);

        /**
         * @brief 计算轨迹数据的压缩率
         * @param json_size JSON序列化后的大小
         * @param binary_size 二进制序列化后的大小
         * @return 压缩率百分比 (0-100)
         */
        static double getCompressionRatio(size_t json_size, size_t binary_size) {
            if (json_size == 0) return 0.0;
            return (1.0 - (double)binary_size / json_size) * 100.0;
        }

    private:
        /**
         * @brief 将UUVNode转换为NodeBinaryData
         */
        static NodeBinaryData nodeToBinary(const UUVNode& node);

        /**
         * @brief 将NodeBinaryData转换为UUVNode
         */
        static UUVNode nodeFromBinary(const NodeBinaryData& binary_data);

        /**
         * @brief 生成JSON格式的元数据块
         */
        static Json::Value generateMetadata(
            const UAVTrajectory& trajectory,
            const FormationConfig& config
        );
    };

} // namespace seven

#endif // TRAJECTORY_SERIALIZER_HPP
