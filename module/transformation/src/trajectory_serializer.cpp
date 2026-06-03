#include "transformation/trajectory_serializer.hpp"
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <sstream>

using namespace Json;

namespace seven {

    // ==================== TrajectorySerializer 方法实现 ====================

    NodeBinaryData TrajectorySerializer::nodeToBinary(const UUVNode& node) {
        NodeBinaryData bin_node = {};
        bin_node.node_id = node.id;
        bin_node.lon_deg = node.pos_.lon_deg;
        bin_node.lat_deg = node.pos_.lat_deg;
        bin_node.rel_x = node.rel_x;
        bin_node.rel_y = node.rel_y;
        bin_node.target_x = node.target_x;
        bin_node.target_y = node.target_y;
        bin_node.speed_ms = node.speed;
        bin_node.heading_deg = node.heading;
        bin_node.formation_error = 0.0;  // 会在序列化时更新
        bin_node.is_joining = node.is_joining ? 1 : 0;
        bin_node.is_leaving = node.is_leaving ? 1 : 0;
        bin_node.join_progress = static_cast<float>(node.join_progress);
        return bin_node;
    }

    UUVNode TrajectorySerializer::nodeFromBinary(const NodeBinaryData& binary_data) {
        UUVNode node = {};
        node.id = binary_data.node_id;
        node.pos_.lon_deg = binary_data.lon_deg;
        node.pos_.lat_deg = binary_data.lat_deg;
        node.rel_x = binary_data.rel_x;
        node.rel_y = binary_data.rel_y;
        node.target_x = binary_data.target_x;
        node.target_y = binary_data.target_y;
        node.speed = binary_data.speed_ms;
        node.heading = binary_data.heading_deg;
        node.is_joining = binary_data.is_joining != 0;
        node.is_leaving = binary_data.is_leaving != 0;
        node.join_progress = binary_data.join_progress;
        return node;
    }

    Json::Value TrajectorySerializer::generateMetadata(
        const UAVTrajectory& trajectory,
        const FormationConfig& config) {
        
        Json::Value metadata;
        metadata["version"] = "1.0";
        metadata["simulation_time_s"] = trajectory.getSimulationTime();
        metadata["total_frames"] = static_cast<int>(trajectory.getFrameCount());
        metadata["frame_interval_s"] = config.sim_step;
        
        Json::Value config_obj;
        config_obj["node_num"] = config.node_num;
        config_obj["rel_distance"] = config.rel_distance;
        config_obj["collision_radius"] = config.collision_radius;
        config_obj["init_speed"] = config.init_speed;
        config_obj["init_heading"] = config.init_heading;
        config_obj["heading_rate"] = config.heading_rate;
        config_obj["acceleration"] = config.acceleration;
        config_obj["sim_step"] = config.sim_step;
        config_obj["max_frames"] = config.max_frames;
        config_obj["return_frames"] = config.return_frames;
        config_obj["main_node"]["lon"] = config.main_node.lon_deg;
        config_obj["main_node"]["lat"] = config.main_node.lat_deg;
        
        metadata["formation_config"] = config_obj;
        return metadata;
    }

    std::string TrajectorySerializer::toJSON(
        const UAVTrajectory& trajectory,
        const FormationConfig& config,
        bool include_metadata) {
        
        Value root;
        
        if (include_metadata) {
            root["metadata"] = generateMetadata(trajectory, config);
        }
        
        Value frames(arrayValue);
        const auto& all_frames = trajectory.getAllTrajectory();
        
        for (const auto& frame : all_frames) {
            Value frame_obj;
            frame_obj["frame_id"] = frame.frame;
            frame_obj["timestamp_s"] = frame.frame * config.sim_step;
            frame_obj["formation_type"] = formationToStr(frame.formation);
            
            Value nodes(arrayValue);
            for (const auto& node : frame.nodes_) {
                Value node_obj;
                node_obj["node_id"] = node.id;
                node_obj["type"] = (node.id == 0) ? "leader" : "follower";
                node_obj["lon"] = std::round(node.pos_.lon_deg * 1e6) / 1e6;
                node_obj["lat"] = std::round(node.pos_.lat_deg * 1e6) / 1e6;
                node_obj["speed_ms"] = std::round(node.speed * 1e3) / 1e3;
                node_obj["heading_deg"] = std::round(node.heading * 1e3) / 1e3;
                node_obj["rel_x"] = std::round(node.rel_x * 1e3) / 1e3;
                node_obj["rel_y"] = std::round(node.rel_y * 1e3) / 1e3;
                node_obj["target_x"] = std::round(node.target_x * 1e3) / 1e3;
                node_obj["target_y"] = std::round(node.target_y * 1e3) / 1e3;
                
                Value status;
                status["is_joining"] = node.is_joining;
                status["is_leaving"] = node.is_leaving;
                status["join_progress"] = std::round(node.join_progress * 1000) / 1000;
                node_obj["status"] = status;
                
                nodes.append(node_obj);
            }
            frame_obj["nodes"] = nodes;
            frames.append(frame_obj);
        }
        
        root["frames"] = frames;
        return writeString(StreamWriterBuilder(), root);
    }

    bool TrajectorySerializer::toBinary(
        const std::string& filename,
        const UAVTrajectory& trajectory,
        const FormationConfig& config) {
        
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            printf("❌ 无法打开文件进行写入：%s\n", filename.c_str());
            return false;
        }

        try {
            // ============ 写入文件头 ============
            TrajectoryFileHeader header = {};
            header.magic = TRAJECTORY_MAGIC;
            header.version = TRAJECTORY_VERSION;
            
            const auto& all_frames = trajectory.getAllTrajectory();
            header.frame_count = static_cast<uint32_t>(all_frames.size());
            
            // 计算总节点数
            uint32_t total_nodes = 0;
            for (const auto& frame : all_frames) {
                total_nodes += frame.nodes_.size();
            }
            header.total_nodes = total_nodes;
            header.simulation_time = trajectory.getSimulationTime();
            header.metadata_offset = sizeof(TrajectoryFileHeader);
            
            // 先写入头部（先占位）
            file.write(reinterpret_cast<const char*>(&header), sizeof(TrajectoryFileHeader));

            // ============ 生成并写入元数据 ============
            Value metadata_json = generateMetadata(trajectory, config);
            std::string metadata_str = writeString(StreamWriterBuilder(), metadata_json);
            header.metadata_size = static_cast<uint32_t>(metadata_str.size());
            
            file.write(metadata_str.c_str(), metadata_str.size());

            // ============ 建立帧索引表 ============
            header.frame_index_offset = static_cast<uint32_t>(file.tellp());
            
            for (size_t i = 0; i < all_frames.size(); ++i) {
                uint32_t placeholder = 0;
                file.write(reinterpret_cast<const char*>(&placeholder), sizeof(uint32_t));
            }

            // ============ 写入帧数据 ============
            header.frame_data_offset = static_cast<uint32_t>(file.tellp());

            for (size_t frame_idx = 0; frame_idx < all_frames.size(); ++frame_idx) {
                const auto& frame = all_frames[frame_idx];
                
                // 更新帧偏移表
                uint32_t current_pos = static_cast<uint32_t>(file.tellp());
                file.seekp(header.frame_index_offset + frame_idx * sizeof(uint32_t));
                file.write(reinterpret_cast<const char*>(&current_pos), sizeof(uint32_t));
                file.seekp(current_pos);

                // 写入帧头
                FrameHeader frame_header;
                frame_header.frame_id = frame.frame;
                frame_header.timestamp_ms = static_cast<int>(frame.frame * config.sim_step * 1000);
                frame_header.formation_type = static_cast<uint32_t>(frame.formation);
                frame_header.node_count = static_cast<uint16_t>(frame.nodes_.size());
                frame_header.reserved = 0;
                
                file.write(reinterpret_cast<const char*>(&frame_header), sizeof(FrameHeader));

                // 写入节点数据
                for (const auto& node : frame.nodes_) {
                    NodeBinaryData bin_node = nodeToBinary(node);
                    file.write(reinterpret_cast<const char*>(&bin_node), sizeof(NodeBinaryData));
                }
            }

            // ============ 更新文件头并重新写入 ============
            file.seekp(0);
            file.write(reinterpret_cast<const char*>(&header), sizeof(TrajectoryFileHeader));

            file.close();
            size_t file_size = std::filesystem::file_size(filename);
            printf("✅ 轨迹数据已序列化到二进制文件：%s\n", filename.c_str());
            printf("   文件大小：%.2f KB，总帧数：%u，总节点数：%u\n", 
                file_size / 1024.0, header.frame_count, header.total_nodes);
            return true;
        }
        catch (const std::exception& e) {
            printf("❌ 二进制序列化失败：%s\n", e.what());
            file.close();
            return false;
        }
    }

    bool TrajectorySerializer::fromBinary(
        const std::string& filename,
        std::vector<TrajectoryFrame>& out_frames,
        TrajectoryFileHeader& out_header) {
        
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            printf("❌ 无法打开文件进行读取：%s\n", filename.c_str());
            return false;
        }

        try {
            // ============ 读取文件头 ============
            file.read(reinterpret_cast<char*>(&out_header), sizeof(TrajectoryFileHeader));
            
            // 验证魔数和版本
            if (out_header.magic != TRAJECTORY_MAGIC) {
                printf("❌ 文件格式错误：魔数不匹配\n");
                return false;
            }
            if (out_header.version != TRAJECTORY_VERSION) {
                printf("⚠️  版本不匹配，可能无法正确读取\n");
            }

            // ============ 读取元数据 ============
            file.seekg(out_header.metadata_offset);
            std::vector<char> metadata_buf(out_header.metadata_size);
            file.read(metadata_buf.data(), out_header.metadata_size);
            std::string metadata_str(metadata_buf.begin(), metadata_buf.end());
            printf("✅ 元数据已读取 (%u 字节)\n", out_header.metadata_size);

            // ============ 读取帧索引表 ============
            file.seekg(out_header.frame_index_offset);
            std::vector<uint32_t> frame_offsets(out_header.frame_count);
            file.read(reinterpret_cast<char*>(frame_offsets.data()), out_header.frame_count * sizeof(uint32_t));

            // ============ 读取帧数据 ============
            out_frames.clear();
            for (uint32_t frame_idx = 0; frame_idx < out_header.frame_count; ++frame_idx) {
                file.seekg(frame_offsets[frame_idx]);

                // 读取帧头
                FrameHeader frame_header;
                file.read(reinterpret_cast<char*>(&frame_header), sizeof(FrameHeader));

                // 读取节点数据
                TrajectoryFrame traj_frame;
                traj_frame.frame = frame_header.frame_id;
                traj_frame.formation = static_cast<Formation_Type>(frame_header.formation_type);

                for (uint16_t node_idx = 0; node_idx < frame_header.node_count; ++node_idx) {
                    NodeBinaryData bin_node;
                    file.read(reinterpret_cast<char*>(&bin_node), sizeof(NodeBinaryData));
                    UUVNode node = nodeFromBinary(bin_node);
                    traj_frame.nodes_.push_back(node);
                }

                out_frames.push_back(traj_frame);
            }

            file.close();
            printf("✅ 轨迹数据已从二进制文件读取\n");
            printf("   总帧数：%u，总节点数：%u，仿真时长：%.2f s\n",
                out_header.frame_count, out_header.total_nodes, out_header.simulation_time);
            return true;
        }
        catch (const std::exception& e) {
            printf("❌ 二进制反序列化失败：%s\n", e.what());
            file.close();
            return false;
        }
    }

    Json::Value TrajectorySerializer::getBinaryMetadata(const std::string& filename) {
        Value metadata;
        TrajectoryFileHeader header;
        
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            metadata["error"] = "无法打开文件";
            return metadata;
        }

        try {
            file.read(reinterpret_cast<char*>(&header), sizeof(TrajectoryFileHeader));
            
            if (header.magic != TRAJECTORY_MAGIC) {
                metadata["error"] = "文件格式错误";
                return metadata;
            }

            file.seekg(header.metadata_offset);
            std::vector<char> metadata_buf(header.metadata_size);
            file.read(metadata_buf.data(), header.metadata_size);
            std::string metadata_str(metadata_buf.begin(), metadata_buf.end());

            CharReaderBuilder builder;
            std::string errs;
            std::istringstream stream(metadata_str);
            if (!parseFromStream(builder, stream, &metadata, &errs)) {
                metadata["error"] = "Metadata parse error";
            }

            file.close();
        }
        catch (const std::exception& e) {
            metadata["error"] = std::string("读取失败: ") + e.what();
        }

        return metadata;
    }

} // namespace seven
