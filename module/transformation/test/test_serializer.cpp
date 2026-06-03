/**
 * @file test_serializer.cpp
 * @brief Serialization unit tests
 *
 * Test coverage:
 *   - Binary serialization round-trip (indirectly tests NodeBinaryData conversion)
 *   - Struct size validation
 *   - JSON serialization structure
 *   - Binary magic number and version
 *   - Compression ratio
 *   - Empty trajectory edge case
 *   - Binary metadata reading
 *   - Random frame access (frame index table)
 *   - Large dataset (1000+ frames)
 *   - Invalid file handling
 */

#include "test_common.hpp"
#include "transformation/transformation.hpp"
#include "transformation/trajectory_serializer.hpp"

#include <fstream>
#include <filesystem>
#include <iostream>
#include <sstream>

using namespace seven;
namespace fs = std::filesystem;

// ==================== Helpers ====================

static UUVNode make_test_node(int id, double lon, double lat,
                               double speed, double heading,
                               double rel_x, double rel_y,
                               double target_x, double target_y,
                               bool is_joining = false, bool is_leaving = false,
                               double join_progress = 0.0) {
    UUVNode node;
    node.id = id;
    node.pos_.lon_deg = lon;
    node.pos_.lat_deg = lat;
    node.speed = speed;
    node.heading = heading;
    node.rel_x = rel_x;
    node.rel_y = rel_y;
    node.target_x = target_x;
    node.target_y = target_y;
    node.is_joining = is_joining;
    node.is_leaving = is_leaving;
    node.join_progress = join_progress;
    return node;
}

static FormationConfig make_test_config() {
    FormationConfig cfg;
    cfg.node_num = 5;
    cfg.max_frames = 1000;
    cfg.return_frames = 100;
    cfg.rel_distance = 10.0;
    cfg.collision_radius = 4.0;
    cfg.init_speed = 2.0;
    cfg.init_heading = 45.0;
    cfg.heading_rate = 2.0;
    cfg.acceleration = 0.1;
    cfg.sim_step = 0.1;
    cfg.main_node.lon_deg = 116.397428;
    cfg.main_node.lat_deg = 39.909204;
    cfg.trans_formation = Formation_Type::Rectangle;
    cfg.current_formation = Formation_Type::Rectangle;
    return cfg;
}

static void populate_test_trajectory(UAVTrajectory& traj,
                                      FormationConfig& cfg,
                                      int frame_count,
                                      int nodes_per_frame) {
    for (int f = 0; f < frame_count; ++f) {
        std::vector<UUVNode> nodes;
        for (int n = 0; n < nodes_per_frame; ++n) {
            double offset = n * 10.0;
            nodes.push_back(make_test_node(
                n,
                cfg.main_node.lon_deg + offset * 0.0001,
                cfg.main_node.lat_deg + offset * 0.0001,
                2.0 + n * 0.5,
                45.0 + n * 10.0,
                offset,
                -offset,
                offset + 1.0,
                -offset + 1.0,
                n == nodes_per_frame - 1,
                n == 0,
                n == nodes_per_frame - 1 ? 0.5 : 0.0
            ));
        }
        traj.addFrame(f, Formation_Type::Rectangle, nodes);
    }
    traj.setMetadata(cfg, frame_count * cfg.sim_step);
}

static std::string temp_file_path(const char* name) {
    return (fs::temp_directory_path() / name).string();
}

static void cleanup_temp_file(const std::string& path) {
    if (fs::exists(path)) {
        fs::remove(path);
    }
}

// ==================== Test Cases ====================

// --- Test 1: Binary round-trip (tests NodeBinaryData conversion indirectly) ---
bool test_node_binary_roundtrip() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;

    // Create a single frame with a well-known node
    std::vector<UUVNode> nodes;
    UUVNode original = make_test_node(
        42, 116.397428, 39.909204,
        3.5, 90.0,
        10.0, -20.0,
        12.0, -18.0,
        true, false, 0.75
    );
    nodes.push_back(original);
    traj.addFrame(0, Formation_Type::Rectangle, nodes);
    traj.setMetadata(cfg, 0.1);

    // Serialize to binary file
    std::string filepath = temp_file_path("test_node_rtt.traj");
    bool write_ok = TrajectorySerializer::toBinary(filepath, traj, cfg);
    ASSERT_TRUE(write_ok);

    // Read back
    std::vector<TrajectoryFrame> out_frames;
    TrajectoryFileHeader out_header;
    bool read_ok = TrajectorySerializer::fromBinary(filepath, out_frames, out_header);
    ASSERT_TRUE(read_ok);

    ASSERT_EQ(out_frames.size(), (size_t)1);
    ASSERT_EQ(out_frames[0].nodes_.size(), (size_t)1);

    // Verify all fields survived the round-trip
    const UUVNode& restored = out_frames[0].nodes_[0];
    ASSERT_EQ(restored.id, original.id);
    ASSERT_DOUBLE_EQ(restored.pos_.lon_deg, original.pos_.lon_deg, 1e-9);
    ASSERT_DOUBLE_EQ(restored.pos_.lat_deg, original.pos_.lat_deg, 1e-9);
    ASSERT_DOUBLE_EQ(restored.speed, original.speed, 1e-9);
    ASSERT_DOUBLE_EQ(restored.heading, original.heading, 1e-9);
    ASSERT_DOUBLE_EQ(restored.rel_x, original.rel_x, 1e-9);
    ASSERT_DOUBLE_EQ(restored.rel_y, original.rel_y, 1e-9);
    ASSERT_DOUBLE_EQ(restored.target_x, original.target_x, 1e-9);
    ASSERT_DOUBLE_EQ(restored.target_y, original.target_y, 1e-9);
    ASSERT_EQ((int)restored.is_joining, (int)original.is_joining);
    ASSERT_EQ((int)restored.is_leaving, (int)original.is_leaving);
    ASSERT_DOUBLE_EQ(restored.join_progress, original.join_progress, 0.01);

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 2: Struct size validation ---
bool test_struct_sizes() {
    // NodeBinaryData uses #pragma pack(1) -> exactly 128 bytes
    ASSERT_EQ(sizeof(NodeBinaryData), (size_t)128);
    // TrajectoryFileHeader: 4+2+2+4+4+8+4+4+4+4+8 = 48 bytes (natural alignment)
    ASSERT_EQ(sizeof(TrajectoryFileHeader), (size_t)48);
    // FrameHeader: 4+4+4+2+2 = 16 bytes (natural alignment)
    ASSERT_EQ(sizeof(FrameHeader), (size_t)16);
    return true;
}

// --- Test 3: JSON serialization ---
bool test_json_serialization() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;
    populate_test_trajectory(traj, cfg, 10, 3);

    std::string json_str = traj.serializeToJSON();
    ASSERT_GT(json_str.size(), (size_t)0);

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream stream(json_str);
    bool parsed = Json::parseFromStream(builder, stream, &root, &errs);
    ASSERT_TRUE(parsed);

    ASSERT_TRUE(root.isMember("metadata"));
    ASSERT_TRUE(root["metadata"].isMember("simulator_version"));
    ASSERT_EQ(root["metadata"]["total_frames"].asInt(), 10);

    ASSERT_TRUE(root.isMember("frames"));
    ASSERT_EQ(root["frames"].size(), 10u);

    Json::Value first_frame = root["frames"][0];
    ASSERT_TRUE(first_frame.isMember("frame_id"));
    ASSERT_EQ(first_frame["frame_id"].asInt(), 0);
    ASSERT_TRUE(first_frame.isMember("nodes"));
    ASSERT_EQ(first_frame["nodes"].size(), 3u);

    return true;
}

// --- Test 4: Binary serialization round-trip ---
bool test_binary_serialization_roundtrip() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;
    populate_test_trajectory(traj, cfg, 20, 4);

    std::string filepath = temp_file_path("test_roundtrip.traj");
    bool write_ok = TrajectorySerializer::toBinary(filepath, traj, cfg);
    ASSERT_TRUE(write_ok);

    std::vector<TrajectoryFrame> out_frames;
    TrajectoryFileHeader out_header;
    bool read_ok = TrajectorySerializer::fromBinary(filepath, out_frames, out_header);
    ASSERT_TRUE(read_ok);

    ASSERT_EQ(out_header.frame_count, (uint32_t)20);
    ASSERT_EQ(out_frames.size(), (size_t)20);

    const auto& orig = traj.getAllTrajectory();
    for (size_t i = 0; i < out_frames.size(); ++i) {
        ASSERT_EQ(out_frames[i].frame, orig[i].frame);
        ASSERT_EQ((int)out_frames[i].formation, (int)orig[i].formation);
        ASSERT_EQ(out_frames[i].nodes_.size(), orig[i].nodes_.size());

        for (size_t j = 0; j < out_frames[i].nodes_.size(); ++j) {
            const UUVNode& o = orig[i].nodes_[j];
            const UUVNode& r = out_frames[i].nodes_[j];
            ASSERT_EQ(r.id, o.id);
            ASSERT_DOUBLE_EQ(r.pos_.lon_deg, o.pos_.lon_deg, 1e-9);
            ASSERT_DOUBLE_EQ(r.pos_.lat_deg, o.pos_.lat_deg, 1e-9);
            ASSERT_DOUBLE_EQ(r.rel_x, o.rel_x, 1e-9);
            ASSERT_DOUBLE_EQ(r.rel_y, o.rel_y, 1e-9);
            ASSERT_DOUBLE_EQ(r.speed, o.speed, 1e-9);
            ASSERT_DOUBLE_EQ(r.heading, o.heading, 1e-9);
            ASSERT_EQ((int)r.is_joining, (int)o.is_joining);
            ASSERT_EQ((int)r.is_leaving, (int)o.is_leaving);
        }
    }

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 5: Magic number verification ---
bool test_binary_magic_number() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;
    populate_test_trajectory(traj, cfg, 5, 2);

    std::string filepath = temp_file_path("test_magic.traj");
    bool ok = TrajectorySerializer::toBinary(filepath, traj, cfg);
    ASSERT_TRUE(ok);

    std::ifstream file(filepath, std::ios::binary);
    ASSERT_TRUE(file.is_open());

    TrajectoryFileHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(TrajectoryFileHeader));
    file.close();

    ASSERT_EQ(header.magic, (uint32_t)0x54524A46);
    ASSERT_EQ(header.version, (uint16_t)0x0100);

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 6: Compression ratio ---
bool test_compression_ratio() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;
    populate_test_trajectory(traj, cfg, 50, 5);

    std::string json_str = traj.serializeToJSON();
    size_t json_size = json_str.size();

    std::string filepath = temp_file_path("test_compression.traj");
    TrajectorySerializer::toBinary(filepath, traj, cfg);
    size_t binary_size = fs::file_size(filepath);

    double ratio = TrajectorySerializer::getCompressionRatio(json_size, binary_size);

    printf("    JSON size: %zu bytes, Binary size: %zu bytes\n", json_size, binary_size);
    printf("    Compression ratio: %.1f%%\n", ratio);

    ASSERT_GT(ratio, 30.0);
    ASSERT_LT(ratio, 100.0);

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 7: Empty trajectory ---
bool test_empty_trajectory() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;
    traj.setMetadata(cfg, 0.0);

    std::string json_str = traj.serializeToJSON();
    ASSERT_GT(json_str.size(), (size_t)0);

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream stream(json_str);
    Json::parseFromStream(builder, stream, &root, &errs);
    ASSERT_TRUE(root.isMember("frames"));
    ASSERT_EQ(root["frames"].size(), 0u);
    ASSERT_EQ(root["metadata"]["total_frames"].asInt(), 0);

    std::string filepath = temp_file_path("test_empty.traj");
    bool write_ok = TrajectorySerializer::toBinary(filepath, traj, cfg);
    ASSERT_TRUE(write_ok);

    std::vector<TrajectoryFrame> out_frames;
    TrajectoryFileHeader out_header;
    bool read_ok = TrajectorySerializer::fromBinary(filepath, out_frames, out_header);
    ASSERT_TRUE(read_ok);
    ASSERT_EQ(out_header.frame_count, (uint32_t)0);
    ASSERT_EQ(out_frames.size(), (size_t)0);

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 8: Binary metadata reading ---
bool test_binary_metadata() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;
    populate_test_trajectory(traj, cfg, 10, 3);

    std::string filepath = temp_file_path("test_metadata.traj");
    TrajectorySerializer::toBinary(filepath, traj, cfg);

    Json::Value metadata = TrajectorySerializer::getBinaryMetadata(filepath);

    ASSERT_TRUE(metadata.isMember("version"));
    ASSERT_TRUE(metadata.isMember("simulation_time_s"));
    ASSERT_TRUE(metadata.isMember("total_frames"));
    ASSERT_EQ(metadata["total_frames"].asInt(), 10);
    ASSERT_TRUE(metadata.isMember("formation_config"));

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 9: Random frame access (frame index table) ---
bool test_random_frame_access() {
    FormationConfig cfg = make_test_config();
    UAVTrajectory traj;
    int frame_count = 30;
    int nodes_per_frame = 3;

    for (int f = 0; f < frame_count; ++f) {
        std::vector<UUVNode> nodes;
        for (int n = 0; n < nodes_per_frame; ++n) {
            nodes.push_back(make_test_node(
                n, 116.0 + f * 0.01, 39.0 + f * 0.01,
                2.0, 45.0,
                (double)(f * 10 + n), (double)(f * 10 + n),
                0.0, 0.0));
        }
        traj.addFrame(f * 10, Formation_Type::Rectangle, nodes);
    }
    traj.setMetadata(cfg, frame_count * cfg.sim_step);

    std::string filepath = temp_file_path("test_random_access.traj");
    TrajectorySerializer::toBinary(filepath, traj, cfg);

    std::vector<TrajectoryFrame> all_frames;
    TrajectoryFileHeader header;
    TrajectorySerializer::fromBinary(filepath, all_frames, header);

    ASSERT_EQ(all_frames.size(), (size_t)frame_count);
    ASSERT_EQ(all_frames[0].frame, 0);
    ASSERT_EQ(all_frames[15].frame, 150);
    ASSERT_EQ(all_frames[frame_count - 1].frame, (frame_count - 1) * 10);

    for (size_t i = 0; i < all_frames.size(); ++i) {
        ASSERT_EQ(all_frames[i].nodes_.size(), (size_t)nodes_per_frame);
        for (size_t j = 0; j < all_frames[i].nodes_.size(); ++j) {
            ASSERT_EQ(all_frames[i].nodes_[j].id, (int)j);
            ASSERT_GT(all_frames[i].nodes_[j].pos_.lon_deg, 115.0);
            ASSERT_LT(all_frames[i].nodes_[j].pos_.lon_deg, 120.0);
        }
    }

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 10: Large file (1100 frames) ---
bool test_large_trajectory() {
    FormationConfig cfg = make_test_config();
    cfg.max_frames = 2000;
    cfg.return_frames = 2000;

    UAVTrajectory traj;
    int frame_count = 1100;
    int nodes_per_frame = 8;
    populate_test_trajectory(traj, cfg, frame_count, nodes_per_frame);

    std::string filepath = temp_file_path("test_large.traj");
    auto write_start = std::chrono::high_resolution_clock::now();
    bool write_ok = TrajectorySerializer::toBinary(filepath, traj, cfg);
    auto write_end = std::chrono::high_resolution_clock::now();
    auto write_ms = std::chrono::duration_cast<std::chrono::milliseconds>(write_end - write_start).count();
    ASSERT_TRUE(write_ok);

    size_t file_size = fs::file_size(filepath);
    printf("    Frames: %d, nodes/frame: %d\n", frame_count, nodes_per_frame);
    printf("    File size: %.2f KB (%zu bytes)\n", file_size / 1024.0, file_size);
    printf("    Write time: %lld ms\n", write_ms);

    size_t expected_min = frame_count * (sizeof(NodeBinaryData) * nodes_per_frame + sizeof(FrameHeader));
    ASSERT_GT(file_size, expected_min);

    auto read_start = std::chrono::high_resolution_clock::now();
    std::vector<TrajectoryFrame> out_frames;
    TrajectoryFileHeader out_header;
    bool read_ok = TrajectorySerializer::fromBinary(filepath, out_frames, out_header);
    auto read_end = std::chrono::high_resolution_clock::now();
    auto read_ms = std::chrono::duration_cast<std::chrono::milliseconds>(read_end - read_start).count();
    ASSERT_TRUE(read_ok);

    printf("    Read time: %lld ms\n", read_ms);

    ASSERT_EQ(out_header.frame_count, (uint32_t)frame_count);
    ASSERT_EQ(out_frames.size(), (size_t)frame_count);
    ASSERT_EQ(out_header.total_nodes, (uint32_t)(frame_count * nodes_per_frame));

    ASSERT_EQ(out_frames[0].frame, 0);
    ASSERT_EQ(out_frames[frame_count / 2].frame, frame_count / 2);
    ASSERT_EQ(out_frames[frame_count - 1].frame, frame_count - 1);

    ASSERT_LT(read_ms, (long long)5000);
    ASSERT_LT(write_ms, (long long)5000);

    cleanup_temp_file(filepath);
    return true;
}

// --- Test 11: Detailed compression ratio comparison ---
bool test_compression_ratio_detailed() {
    printf("\n");
    printf("    %-15s %-10s %-10s %-12s %-12s %-10s\n",
           "Scenario", "Frames", "Nodes/Frm", "JSON size", "Binary size", "Ratio");

    struct { const char* name; int frames; int nodes; } cases[] = {
        {"Small", 100, 3},
        {"Medium", 500, 5},
        {"Large", 1000, 10},
    };

    for (const auto& tc : cases) {
        FormationConfig cfg = make_test_config();
        UAVTrajectory traj;
        populate_test_trajectory(traj, cfg, tc.frames, tc.nodes);

        std::string json_str = traj.serializeToJSON();
        std::string filepath = temp_file_path("test_comp_detail.traj");
        TrajectorySerializer::toBinary(filepath, traj, cfg);
        size_t binary_size = fs::file_size(filepath);

        double ratio = TrajectorySerializer::getCompressionRatio(json_str.size(), binary_size);

        printf("    %-15s %-10d %-10d %-12zu %-12zu %-9.1f%%\n",
               tc.name, tc.frames, tc.nodes,
               json_str.size(), binary_size, ratio);

        ASSERT_GT(ratio, 30.0);
        ASSERT_LT(ratio, 95.0);

        cleanup_temp_file(filepath);
    }

    return true;
}

// --- Test 12: Invalid file handling ---
bool test_invalid_file_handling() {
    std::vector<TrajectoryFrame> frames;
    TrajectoryFileHeader header;

    // Non-existent file
    bool result = TrajectorySerializer::fromBinary("nonexistent_file.traj", frames, header);
    ASSERT_FALSE(result);

    // Invalid file
    std::string bad_file = temp_file_path("bad_file.traj");
    {
        std::ofstream f(bad_file, std::ios::binary);
        const char data[] = "This is not a valid trajectory file";
        f.write(data, sizeof(data));
        f.close();
    }
    result = TrajectorySerializer::fromBinary(bad_file, frames, header);
    ASSERT_FALSE(result);
    cleanup_temp_file(bad_file);

    // Non-existent metadata
    Json::Value meta = TrajectorySerializer::getBinaryMetadata("nonexistent.traj");
    ASSERT_TRUE(meta.isMember("error"));

    return true;
}

// ==================== Test Registration ====================

void register_serializer_tests() {
    TEST_SUITE("Serializer Unit Tests");

    register_test("NodeBinary round-trip (via file)", test_node_binary_roundtrip);
    register_test("Struct size validation", test_struct_sizes);
    register_test("JSON serialization", test_json_serialization);
    register_test("Binary serialization round-trip", test_binary_serialization_roundtrip);
    register_test("Magic number verification", test_binary_magic_number);
    register_test("Compression ratio", test_compression_ratio);
    register_test("Empty trajectory", test_empty_trajectory);
    register_test("Binary metadata reading", test_binary_metadata);
    register_test("Random frame access", test_random_frame_access);
    register_test("Large file (1100 frames)", test_large_trajectory);
    register_test("Detailed compression ratio", test_compression_ratio_detailed);
    register_test("Invalid file handling", test_invalid_file_handling);
}
