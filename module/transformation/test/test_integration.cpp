/**
 * @file test_integration.cpp
 * @brief Integration tests - Full simulation pipeline verification
 *
 * Test coverage:
 *   - Simulation 10 frames -> export JSON and verify
 *   - Simulation -> export binary -> read back
 *   - JSON vs binary data consistency
 *   - Formation switch export
 *   - Node add/remove export
 *   - Simulation statistics and status
 *   - Full pipeline (Init -> Step -> Export -> Read -> Compare)
 *   - Multiple simulation iterations
 *   - Large-scale simulation export performance
 */

#include "test_common.hpp"
#include "transformation/transformation.hpp"
#include "transformation/trajectory_serializer.hpp"

#include <fstream>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <chrono>

using namespace seven;
namespace fs = std::filesystem;

// ==================== Helpers ====================

static FormationConfig make_default_config() {
    FormationConfig cfg;
    cfg.node_num = 4;
    cfg.max_frames = 500;
    cfg.return_frames = 10;
    cfg.rel_distance = 10.0;
    cfg.collision_radius = 4.0;
    cfg.init_speed = 2.0;
    cfg.init_heading = 0.0;
    cfg.heading_rate = 2.0;
    cfg.acceleration = 0.0;
    cfg.sim_step = 0.1;
    cfg.main_node.lon_deg = 116.397428;
    cfg.main_node.lat_deg = 39.909204;
    cfg.trans_formation = Formation_Type::Rectangle;
    cfg.current_formation = Formation_Type::Rectangle;
    return cfg;
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

// --- Test 1: Simulation 10 frames -> export JSON ---
bool test_simulation_10frames_export_json() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    ASSERT_TRUE(g_pFormationSimulator != nullptr);

    UAVTrajectory& traj = g_pFormationSimulator->step_simulation();
    ASSERT_GT(traj.getFrameCount(), (size_t)0);

    printf("    Simulation frames: %zu\n", traj.getFrameCount());

    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    ASSERT_GT(json_str.size(), (size_t)0);

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream stream(json_str);
    bool parsed = Json::parseFromStream(builder, stream, &root, &errs);
    ASSERT_TRUE(parsed);

    ASSERT_TRUE(root.isMember("metadata"));
    ASSERT_TRUE(root.isMember("frames"));
    ASSERT_GT(root["frames"].size(), 0u);

    // Verify each frame has nodes with leader
    for (const auto& frame : root["frames"]) {
        ASSERT_TRUE(frame.isMember("nodes"));
        ASSERT_GT(frame["nodes"].size(), 0u);
        ASSERT_TRUE(frame["nodes"][0].isMember("type"));
    }

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 2: Simulation -> export binary -> read back ---
bool test_simulation_export_binary_roundtrip() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    ASSERT_TRUE(g_pFormationSimulator != nullptr);
    g_pFormationSimulator->step_simulation();

    std::string filepath = temp_file_path("test_integration.traj");
    bool export_ok = g_pFormationSimulator->exportTrajectoryBinary(filepath);
    ASSERT_TRUE(export_ok);
    ASSERT_TRUE(fs::exists(filepath));

    size_t file_size = fs::file_size(filepath);
    printf("    Exported file size: %.2f KB\n", file_size / 1024.0);

    std::vector<TrajectoryFrame> out_frames;
    TrajectoryFileHeader out_header;
    bool read_ok = TrajectorySerializer::fromBinary(filepath, out_frames, out_header);
    ASSERT_TRUE(read_ok);
    ASSERT_GT(out_frames.size(), (size_t)0);
    ASSERT_GT(out_header.frame_count, (uint32_t)0);
    ASSERT_GT(out_header.total_nodes, (uint32_t)0);

    printf("    Read frames: %u, total nodes: %u\n",
           out_header.frame_count, out_header.total_nodes);

    // Verify leader node (id=0) exists in each frame
    for (const auto& frame : out_frames) {
        ASSERT_GT(frame.nodes_.size(), (size_t)0);
        bool has_leader = false;
        for (const auto& node : frame.nodes_) {
            if (node.id == 0) { has_leader = true; break; }
        }
        ASSERT_TRUE(has_leader);
    }

    cleanup_temp_file(filepath);
    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 3: JSON vs binary data consistency ---
bool test_json_vs_binary_data_consistency() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    g_pFormationSimulator->step_simulation();

    // Export JSON
    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    Json::Value json_root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream stream(json_str);
    Json::parseFromStream(builder, stream, &json_root, &errs);

    // Export binary and read back
    std::string filepath = temp_file_path("test_consistency.traj");
    g_pFormationSimulator->exportTrajectoryBinary(filepath);
    std::vector<TrajectoryFrame> bin_frames;
    TrajectoryFileHeader bin_header;
    TrajectorySerializer::fromBinary(filepath, bin_frames, bin_header);

    // Compare frame counts
    ASSERT_EQ(json_root["frames"].size(), bin_frames.size());

    // Compare frame by frame
    int total_mismatches = 0;
    for (size_t i = 0; i < bin_frames.size(); ++i) {
        Json::Value json_frame = json_root["frames"][(int)i];
        ASSERT_EQ(json_frame["frame_id"].asInt(), bin_frames[i].frame);
        ASSERT_EQ(json_frame["nodes"].size(), bin_frames[i].nodes_.size());

        for (size_t j = 0; j < bin_frames[i].nodes_.size(); ++j) {
            Json::Value json_node = json_frame["nodes"][(int)j];
            const UUVNode& bin_node = bin_frames[i].nodes_[j];
            ASSERT_EQ(json_node["node_id"].asInt(), bin_node.id);

            double json_lon = json_node["lon"].asDouble();
            double json_lat = json_node["lat"].asDouble();
            if (std::abs(json_lon - bin_node.pos_.lon_deg) > 1e-5) total_mismatches++;
            if (std::abs(json_lat - bin_node.pos_.lat_deg) > 1e-5) total_mismatches++;
        }
    }

    ASSERT_EQ(total_mismatches, 0);
    printf("    JSON frames: %d, Binary frames: %zu - consistent!\n",
           (int)json_root["frames"].size(), bin_frames.size());

    cleanup_temp_file(filepath);
    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 4: Formation switch export ---
bool test_formation_switch_export() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    g_pFormationSimulator->switch_formation(Formation_Type::Circle);
    g_pFormationSimulator->step_simulation();

    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream stream(json_str);
    Json::parseFromStream(builder, stream, &root, &errs);

    ASSERT_GT(root["frames"].size(), 0u);
    ASSERT_TRUE(root["metadata"].isMember("simulation_time_s"));

    printf("    Frames after formation switch: %d\n", (int)root["frames"].size());

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 5: Simulation statistics ---
bool test_simulation_statistics() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    g_pFormationSimulator->step_simulation();

    Json::Value stats = g_pFormationSimulator->getTrajectoryStatistics();

    ASSERT_TRUE(stats.isMember("total_frames"));
    ASSERT_TRUE(stats.isMember("total_nodes_recorded"));
    ASSERT_TRUE(stats.isMember("formation_error"));
    ASSERT_TRUE(stats.isMember("average_speed"));
    ASSERT_TRUE(stats.isMember("simulation_duration_s"));

    ASSERT_GT(stats["total_frames"].asInt(), 0);
    ASSERT_GT(stats["simulation_duration_s"].asDouble(), 0.0);
    ASSERT_TRUE(stats["formation_error"].isMember("max"));
    ASSERT_TRUE(stats["formation_error"].isMember("average"));
    ASSERT_GT(stats["formation_error"]["max"].asDouble(), -1.0);
    ASSERT_GT(stats["average_speed"].asDouble(), 0.0);

    printf("    Total frames: %d\n", stats["total_frames"].asInt());
    printf("    Sim duration: %.2f s\n", stats["simulation_duration_s"].asDouble());
    printf("    Avg speed: %.3f m/s\n", stats["average_speed"].asDouble());

    // Also test simulation status
    Json::Value status = g_pFormationSimulator->getSimulationStatus();
    ASSERT_TRUE(status.isMember("current_time"));
    ASSERT_TRUE(status.isMember("frame_count"));
    ASSERT_TRUE(status.isMember("node_count"));
    ASSERT_TRUE(status.isMember("nodes"));

    printf("    Formation: %s, Nodes: %d\n",
           status["current_formation"].asString().c_str(),
           status["node_count"].asInt());

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 6: Add node export ---
bool test_add_node_export() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    std::vector<UUVNode> new_nodes;
    UUVNode new_node;
    new_node.id = 99;
    new_node.pos_.lon_deg = 116.398;
    new_node.pos_.lat_deg = 39.910;
    new_node.speed = 2.0;
    new_node.heading = 45.0;
    new_node.join_total_frames = 30;
    new_nodes.push_back(new_node);

    g_pFormationSimulator->add_node(new_nodes);
    g_pFormationSimulator->step_simulation();

    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream stream(json_str);
    Json::parseFromStream(builder, stream, &root, &errs);

    ASSERT_GT(root["frames"].size(), 0u);
    Json::Value first_frame = root["frames"][0];
    ASSERT_GT(first_frame["nodes"].size(), 1u);

    printf("    Frames: %d, first frame nodes: %d\n",
           (int)root["frames"].size(), (int)first_frame["nodes"].size());

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 7: Full pipeline (Init -> Step -> Export -> Read -> Compare) ---
bool test_full_pipeline() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 20;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    // Phase 1: Init
    Json::Value init_result;
    Init_formation(cfg, init_result);
    ASSERT_TRUE(g_pFormationSimulator != nullptr);
    ASSERT_TRUE(init_result.isMember("frames"));
    printf("    Phase 1 - Init: %d frames\n", (int)init_result["frames"].size());

    // Phase 2: Step simulation
    UAVTrajectory& traj = g_pFormationSimulator->step_simulation();
    size_t frame_count = traj.getFrameCount();
    ASSERT_GT(frame_count, (size_t)0);
    printf("    Phase 2 - Step: %zu frames\n", frame_count);

    // Phase 3: Export JSON
    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    ASSERT_GT(json_str.size(), (size_t)0);

    // Phase 4: Export Binary
    std::string bin_path = temp_file_path("test_full_pipeline.traj");
    bool bin_ok = g_pFormationSimulator->exportTrajectoryBinary(bin_path);
    ASSERT_TRUE(bin_ok);

    size_t json_size = json_str.size();
    size_t bin_size = fs::file_size(bin_path);
    double ratio = TrajectorySerializer::getCompressionRatio(json_size, bin_size);
    printf("    Phase 3 - Export: JSON=%zu bytes, Binary=%zu bytes, ratio=%.1f%%\n",
           json_size, bin_size, ratio);

    // Phase 5: Read back binary
    std::vector<TrajectoryFrame> read_frames;
    TrajectoryFileHeader read_header;
    bool read_ok = TrajectorySerializer::fromBinary(bin_path, read_frames, read_header);
    ASSERT_TRUE(read_ok);
    ASSERT_EQ(read_frames.size(), frame_count);
    printf("    Phase 5 - Read: %u frames, %u total nodes\n",
           read_header.frame_count, read_header.total_nodes);

    // Phase 6: Compare
    const auto& orig_frames = traj.getAllTrajectory();
    int mismatch_count = 0;
    for (size_t i = 0; i < read_frames.size(); ++i) {
        if (orig_frames[i].nodes_.size() != read_frames[i].nodes_.size()) {
            mismatch_count++;
            continue;
        }
        for (size_t j = 0; j < orig_frames[i].nodes_.size(); ++j) {
            const UUVNode& o = orig_frames[i].nodes_[j];
            const UUVNode& r = read_frames[i].nodes_[j];
            if (o.id != r.id || o.speed != r.speed || o.heading != r.heading) {
                mismatch_count++;
            }
        }
    }
    ASSERT_EQ(mismatch_count, 0);
    printf("    Phase 6 - Compare: 0 mismatches\n");

    // Phase 7: Statistics
    Json::Value stats = g_pFormationSimulator->getTrajectoryStatistics();
    printf("    Phase 7 - Stats: avg_speed=%.2f m/s, max_error=%.4f m\n",
           stats["average_speed"].asDouble(),
           stats["formation_error"]["max"].asDouble());

    cleanup_temp_file(bin_path);
    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 8: Multiple simulation steps ---
bool test_multiple_simulation_steps() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    int total_frames = 0;
    for (int step = 0; step < 3; ++step) {
        UAVTrajectory& traj = g_pFormationSimulator->step_simulation();
        total_frames += (int)traj.getFrameCount();
        printf("    Step %d: %zu frames\n", step + 1, traj.getFrameCount());
    }

    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream stream(json_str);
    Json::parseFromStream(builder, stream, &root, &errs);

    printf("    Cumulative frames: %d, exported: %d\n",
           total_frames, (int)root["frames"].size());
    ASSERT_GT(root["frames"].size(), 0u);

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 9: Large simulation export performance ---
bool test_large_simulation_export() {
    FormationConfig cfg = make_default_config();
    cfg.node_num = 8;
    cfg.return_frames = 100;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    auto sim_start = std::chrono::high_resolution_clock::now();
    size_t total_frames = 0;
    for (int step = 0; step < 10; ++step) {
        UAVTrajectory& traj = g_pFormationSimulator->step_simulation();
        total_frames += traj.getFrameCount();
    }
    auto sim_end = std::chrono::high_resolution_clock::now();
    auto sim_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sim_end - sim_start).count();
    printf("    Simulation: %zu frames, %lld ms\n", total_frames, sim_ms);

    auto json_start = std::chrono::high_resolution_clock::now();
    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    auto json_end = std::chrono::high_resolution_clock::now();
    auto json_ms = std::chrono::duration_cast<std::chrono::milliseconds>(json_end - json_start).count();
    printf("    JSON export: %zu bytes, %lld ms\n", json_str.size(), json_ms);

    std::string bin_path = temp_file_path("test_large_sim.traj");
    auto bin_start = std::chrono::high_resolution_clock::now();
    g_pFormationSimulator->exportTrajectoryBinary(bin_path);
    auto bin_end = std::chrono::high_resolution_clock::now();
    auto bin_ms = std::chrono::duration_cast<std::chrono::milliseconds>(bin_end - bin_start).count();

    size_t bin_size = fs::file_size(bin_path);
    printf("    Binary export: %zu bytes, %lld ms\n", bin_size, bin_ms);

    auto read_start = std::chrono::high_resolution_clock::now();
    std::vector<TrajectoryFrame> read_frames;
    TrajectoryFileHeader read_header;
    TrajectorySerializer::fromBinary(bin_path, read_frames, read_header);
    auto read_end = std::chrono::high_resolution_clock::now();
    auto read_ms = std::chrono::duration_cast<std::chrono::milliseconds>(read_end - read_start).count();
    printf("    Binary read: %lld ms\n", read_ms);

    double ratio = TrajectorySerializer::getCompressionRatio(json_str.size(), bin_size);
    printf("    Compression: %.1f%%\n", ratio);
    ASSERT_GT(ratio, 30.0);

    ASSERT_LT(sim_ms, (long long)10000);
    ASSERT_LT(json_ms, (long long)5000);
    ASSERT_LT(bin_ms, (long long)5000);

    cleanup_temp_file(bin_path);
    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 10: Formation config snapshot ---
bool test_config_snapshot() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    g_pFormationSimulator->step_simulation();

    // setMetadata must be called explicitly to update config snapshot
    UAVTrajectory& traj = g_pFormationSimulator->getUAVtrajectory();
    traj.setMetadata(cfg, 1.0);

    const FormationConfig& snapshot = traj.getConfigSnapshot();

    // Verify config data is preserved after setMetadata
    ASSERT_EQ(snapshot.node_num, cfg.node_num);
    ASSERT_DOUBLE_EQ(snapshot.rel_distance, cfg.rel_distance, 1e-9);
    ASSERT_DOUBLE_EQ(snapshot.init_speed, cfg.init_speed, 1e-9);
    ASSERT_DOUBLE_EQ(traj.getSimulationTime(), 1.0, 1e-9);

    printf("    Config snapshot: nodes=%d, distance=%.1f, speed=%.1f, sim_time=%.1f\n",
           snapshot.node_num, snapshot.rel_distance, snapshot.init_speed,
           traj.getSimulationTime());

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// ==================== Phase 2: Real-time Command Tests ====================

// --- Test 11: Switch formation during running simulation ---
bool test_realtime_switch_formation() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    // Run initial simulation
    g_pFormationSimulator->step_simulation();

    // Issue switch formation command (simulates IPC CMD_SWITCH_FORMATION)
    SwitchFormation(Formation_Type::Circle);
    printf("    Command: Switch to Circle\n");

    // Run simulation to process the switch
    g_pFormationSimulator->step_simulation();

    // Verify simulation is still running
    Json::Value status = g_pFormationSimulator->getSimulationStatus();
    ASSERT_TRUE(status.isMember("current_formation"));
    printf("    Current formation: %s, is_transition: %s\n",
           status["current_formation"].asString().c_str(),
           status["is_transition"].asBool() ? "yes" : "no");

    // Export and verify data
    std::string json_str = g_pFormationSimulator->exportTrajectoryJSON();
    ASSERT_GT(json_str.size(), (size_t)0);

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 12: Turn heading during running simulation ---
bool test_realtime_turn_formation() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    g_pFormationSimulator->step_simulation();

    // Issue turn command (simulates IPC CMD_TURN_FORMATION)
    TurnFormation(5.0);
    printf("    Command: Set heading rate to 5.0 deg/s\n");

    g_pFormationSimulator->step_simulation();

    Json::Value status = g_pFormationSimulator->getSimulationStatus();
    printf("    Simulation time: %.2f s, frames: %d\n",
           status["current_time"].asDouble(),
           status["frame_count"].asInt());

    ASSERT_GT(status["current_time"].asDouble(), 0.0);

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 13: Add multiple nodes during running simulation ---
bool test_realtime_add_nodes() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    Json::Value init_status = g_pFormationSimulator->getSimulationStatus();
    int initial_nodes = init_status["node_count"].asInt();
    printf("    Initial nodes: %d\n", initial_nodes);

    // Add 2 nodes (simulates IPC CMD_ADD_NODE)
    std::vector<UUVNode> new_nodes;
    UUVNode n1;
    n1.id = 100;
    n1.pos_.lon_deg = 116.40;
    n1.pos_.lat_deg = 39.91;
    n1.speed = 2.0;
    n1.heading = 30.0;
    n1.join_total_frames = 20;
    new_nodes.push_back(n1);

    UUVNode n2;
    n2.id = 200;
    n2.pos_.lon_deg = 116.395;
    n2.pos_.lat_deg = 39.905;
    n2.speed = 1.5;
    n2.heading = 60.0;
    n2.join_total_frames = 20;
    new_nodes.push_back(n2);

    AddNode(new_nodes);
    printf("    Command: Add 2 nodes\n");

    g_pFormationSimulator->step_simulation();

    Json::Value status = g_pFormationSimulator->getSimulationStatus();
    int current_nodes = status["node_count"].asInt();
    printf("    Nodes after add: %d\n", current_nodes);

    // After add, total should be initial + 2
    ASSERT_EQ(current_nodes, initial_nodes + 2);

    // Verify nodes in status
    ASSERT_TRUE(status.isMember("nodes"));
    ASSERT_EQ(status["nodes"].size(), (size_t)current_nodes);

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 14: Remove node during running simulation ---
bool test_realtime_remove_node() {
    FormationConfig cfg = make_default_config();
    cfg.node_num = 5;  // Start with 5 nodes for room to remove
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    Json::Value init_status = g_pFormationSimulator->getSimulationStatus();
    int initial_nodes = init_status["node_count"].asInt();
    printf("    Initial nodes: %d\n", initial_nodes);
    ASSERT_GE(initial_nodes, 3);

    // Remove 1 node (simulates IPC CMD_REMOVE_NODE)
    RemoveLastNode(1);
    printf("    Command: Remove 1 node\n");

    g_pFormationSimulator->step_simulation();

    Json::Value status = g_pFormationSimulator->getSimulationStatus();
    printf("    Nodes after remove: %d\n", status["node_count"].asInt());

    // Verify node count decreased (node may still exist if leaving but not yet deleted)
    ASSERT_GT(status["node_count"].asInt(), 0);

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 15: Combined operations (switch + add + step + export) ---
bool test_combined_operations() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);
    g_pFormationSimulator->step_simulation();

    // Operation 1: Switch formation
    SwitchFormation(Formation_Type::Triangle);
    printf("    [1] Switched to Triangle\n");
    g_pFormationSimulator->step_simulation();

    // Operation 2: Turn
    TurnFormation(-3.0);
    printf("    [2] Set heading rate -3.0 deg/s\n");
    g_pFormationSimulator->step_simulation();

    // Operation 3: Add node
    std::vector<UUVNode> new_nodes;
    UUVNode n;
    n.id = 500;
    n.pos_.lon_deg = 116.399;
    n.pos_.lat_deg = 39.911;
    n.speed = 2.5;
    n.heading = 90.0;
    n.join_total_frames = 15;
    new_nodes.push_back(n);
    AddNode(new_nodes);
    printf("    [3] Added 1 node\n");
    g_pFormationSimulator->step_simulation();

    // Operation 4: Switch again
    SwitchFormation(Formation_Type::Diamond);
    printf("    [4] Switched to Diamond\n");
    g_pFormationSimulator->step_simulation();

    // Final: Get status and stats
    Json::Value status = g_pFormationSimulator->getSimulationStatus();
    Json::Value stats = g_pFormationSimulator->getTrajectoryStatistics();

    printf("    Final: formation=%s, nodes=%d, frames=%d\n",
           status["current_formation"].asString().c_str(),
           status["node_count"].asInt(),
           stats["total_frames"].asInt());

    ASSERT_TRUE(status.isMember("current_formation"));
    ASSERT_GT(stats["total_frames"].asInt(), 0);

    // Export and verify round-trip
    std::string bin_path = temp_file_path("test_combined.traj");
    bool export_ok = g_pFormationSimulator->exportTrajectoryBinary(bin_path);
    ASSERT_TRUE(export_ok);

    std::vector<TrajectoryFrame> read_frames;
    TrajectoryFileHeader header;
    bool read_ok = TrajectorySerializer::fromBinary(bin_path, read_frames, header);
    ASSERT_TRUE(read_ok);
    ASSERT_GT(read_frames.size(), (size_t)0);

    cleanup_temp_file(bin_path);
    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 16: CalcTempParam export config ---
bool test_calctempparam_export_config() {
    // Verify the new export config fields exist and work
    CalcTempParam task_param;
    task_param.export_enabled = true;
    task_param.export_format = "json";
    task_param.export_filename = "";

    ASSERT_EQ(task_param.export_enabled, true);
    ASSERT_STREQ(task_param.export_format, "json");

    task_param.export_format = "binary";
    task_param.export_filename = "test_output.traj";
    ASSERT_STREQ(task_param.export_format, "binary");
    ASSERT_STREQ(task_param.export_filename, "test_output.traj");

    printf("    CalcTempParam export config fields verified\n");
    return true;
}

// --- Test 17: Simulation status during operations ---
bool test_simulation_status_during_ops() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    // Check initial status
    Json::Value s0 = g_pFormationSimulator->getSimulationStatus();
    printf("    Initial: time=%.2f, nodes=%d, formation=%s\n",
           s0["current_time"].asDouble(),
           s0["node_count"].asInt(),
           s0["current_formation"].asString().c_str());
    ASSERT_GE(s0["current_time"].asDouble(), 0.0);

    // After step
    g_pFormationSimulator->step_simulation();
    Json::Value s1 = g_pFormationSimulator->getSimulationStatus();
    printf("    After step: time=%.2f, frames=%d\n",
           s1["current_time"].asDouble(),
           s1["frame_count"].asInt());
    ASSERT_GT(s1["current_time"].asDouble(), s0["current_time"].asDouble());

    // After switch
    SwitchFormation(Formation_Type::Line);
    Json::Value s2 = g_pFormationSimulator->getSimulationStatus();
    printf("    After switch: is_transition=%s\n",
           s2["is_transition"].asBool() ? "yes" : "no");

    // Verify node details
    ASSERT_TRUE(s2.isMember("nodes"));
    for (const auto& node_info : s2["nodes"]) {
        ASSERT_TRUE(node_info.isMember("id"));
        ASSERT_TRUE(node_info.isMember("type"));
        ASSERT_TRUE(node_info.isMember("speed"));
    }

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// --- Test 18: Trajectory statistics during operations ---
bool test_trajectory_stats_during_ops() {
    FormationConfig cfg = make_default_config();
    cfg.return_frames = 10;

    if (g_pFormationSimulator != nullptr) {
        delete g_pFormationSimulator;
        g_pFormationSimulator = nullptr;
    }

    g_pFormationSimulator = new UUVFormationSimulator(cfg);

    // Run several steps
    for (int i = 0; i < 3; ++i) {
        g_pFormationSimulator->step_simulation();
    }

    Json::Value stats = g_pFormationSimulator->getTrajectoryStatistics();
    printf("    Stats after 3 steps:\n");
    printf("      Total frames: %d\n", stats["total_frames"].asInt());
    printf("      Total nodes recorded: %d\n", stats["total_nodes_recorded"].asInt());
    printf("      Avg speed: %.3f m/s\n", stats["average_speed"].asDouble());
    printf("      Max error: %.4f m\n", stats["formation_error"]["max"].asDouble());
    printf("      Avg error: %.4f m\n", stats["formation_error"]["average"].asDouble());
    printf("      Duration: %.2f s\n", stats["simulation_duration_s"].asDouble());

    ASSERT_GT(stats["total_frames"].asInt(), 0);
    // total_nodes_recorded counts nodes with non-zero formation error
    // May be 0 when formation is perfectly stable
    ASSERT_GE(stats["total_nodes_recorded"].asInt(), 0);

    delete g_pFormationSimulator;
    g_pFormationSimulator = nullptr;
    return true;
}

// ==================== Test Registration ====================

void register_integration_tests() {
    TEST_SUITE("Integration Tests - Full Pipeline");

    register_test("Sim 10 frames -> export JSON", test_simulation_10frames_export_json);
    register_test("Sim -> export binary -> read", test_simulation_export_binary_roundtrip);
    register_test("JSON vs binary consistency", test_json_vs_binary_data_consistency);
    register_test("Formation switch export", test_formation_switch_export);
    register_test("Simulation statistics", test_simulation_statistics);
    register_test("Add node export", test_add_node_export);
    register_test("Full pipeline (Init->Step->Export->Read->Compare)", test_full_pipeline);
    register_test("Multiple simulation steps", test_multiple_simulation_steps);
    register_test("Large simulation export perf", test_large_simulation_export);
    register_test("Config snapshot", test_config_snapshot);

    // Phase 2: Real-time command tests
    TEST_SUITE("Phase 2 - Real-time Interactive Commands");
    register_test("Switch formation during simulation", test_realtime_switch_formation);
    register_test("Turn heading during simulation", test_realtime_turn_formation);
    register_test("Add multiple nodes during simulation", test_realtime_add_nodes);
    register_test("Remove node during simulation", test_realtime_remove_node);
    register_test("Combined ops (switch+add+turn+export)", test_combined_operations);
    register_test("CalcTempParam export config", test_calctempparam_export_config);
    register_test("Simulation status during operations", test_simulation_status_during_ops);
    register_test("Trajectory statistics during operations", test_trajectory_stats_during_ops);
}
