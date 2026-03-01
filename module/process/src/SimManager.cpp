#include "process/SimManager.hpp"
#include "transformation/transformation.hpp"
#include "barrage/barrage.hpp"
#include "deception/deception.hpp"

#include <thread>
#include <chrono>
// Windows 管道头文件
#include <windows.h>
#include <cstring>
#include <stdexcept>


using namespace Json;

namespace seven {

    SimManager::SimManager() : sim_state_(SimState::STOPPED), sim_time_(400) {}

    // 1. 仿真开始接口
    int SimManager::sim_start(const Json::Value& input, Json::Value& result) {
        std::lock_guard<std::mutex> lock(sim_mutex_);

        // 检查当前状态
        if (sim_state_ == SimState::RUNNING) {
            result["status"] = "error";
            result["message"] = "仿真已在运行中，无法重复开始";
            return -1;
        }

        try {
            
            // 初始化配置
            Json::Value& jammer_list = result["jammer_list"];
            init_sim_config(input, jammer_list);

            // 构造返回结果（干扰源位置JSON）
            result.clear();
            result["status"] = "success";
            result["message"] = "仿真启动成功";

            // 更新仿真状态
            sim_state_ = SimState::RUNNING;
        }
        catch (const std::exception& e) {
            result["status"] = "error";
            result["message"] = std::string("仿真启动失败: ") + e.what();
            return -1;
        }

        return 0;
    }

    int SimManager::sim_calc(const Json::Value& input, Json::Value& result) {
        std::lock_guard<std::mutex> lock(sim_mutex_);

        // 检查仿真状态
        if (sim_state_ != SimState::RUNNING) {
            result["status"] = "error";
            result["message"] = "仿真已结束，请重新开始";
            return -1;
        }

        try {
            result.clear();
            result["status"] = "success";

            // 解析多平台航迹数据
            const Json::Value& platform_tracks = input["platform_tracks"];
            Json::Value& platform_results = result["platform_results"];

            GNSSJammerSim sim;

            if (input.get("cmd", 4).asInt() == 4) {
                result["status"] = "error";  // 字符串值
                result["message"] = std::string("parser command fail：");  // 拼接字符串
                result["data"] = Json::nullValue;  // 空值（替代 nullptr，JsonCpp 专用）
                return -1;
            }
            Cmd_Type type = static_cast<Cmd_Type>(input.get("cmd", 4).asInt());
            if (type == Cmd_Type::Barrage)
            {
                Barrage_Test_1(input, result);
            }
            else if (type == Cmd_Type::Deception)
            {
                Deception_Use(input, result);
            }
            else if (type == Cmd_Type::Transformation)
            {
                Transformation_Test(input, result);
            }

            // 遍历每个平台的航迹
            //for (int i = 0; i < platform_tracks.size(); i++) {
            //    UINT platform_id = platform_tracks[i]["platform_id"].asUInt();
            //    const Json::Value& track_points_json = platform_tracks[i]["track_points"];

            //    // 转换JSON航迹点到内部格式
            //    std::vector<LLA> track_points;
            //    for (int j = 0; j < track_points_json.size(); j++) {
            //        LLA point;
            //        point.lon_deg = track_points_json[j]["lon_deg"].asDouble();
            //        point.lat_deg = track_points_json[j]["lat_deg"].asDouble();
            //        point.h_m = track_points_json[j]["h_m"].asDouble() / 1000.0; // 米转千米
            //        track_points.push_back(point);
            //    }

            //    // 执行仿真计算
            //    std::vector<BarrageTrackResult> calc_results;// = sim.batch_calc(track_points, config_);

            //    // 构造该平台的结果
            //    Json::Value platform_result;
            //    platform_result["platform_id"] = platform_id;

            //    Json::Value& track_results = platform_result["track_results"];
            //    for (int j = 0; j < calc_results.size(); j++) {
            //        Json::Value track_point;
            //        

            //        track_results.append(track_point);
            //    }

            //    platform_results.append(platform_result);
            //}

        }
        catch (const std::exception& e) {
            result["status"] = "error";
            result["message"] = std::string("计算失败: ") + e.what();
            return -1;
        }

        return 0;
    }

    // 3. 仿真结束接口
    int SimManager::sim_stop(Json::Value& result) {
        std::lock_guard<std::mutex> lock(sim_mutex_);

        result.clear();

        if (sim_state_ != SimState::RUNNING) {
            result["status"] = "warning";
            result["message"] = "仿真未运行，无需结束";
            return 0;
        }

        // 重置状态和配置
        sim_state_ = SimState::STOPPED;
        //config_ = SimConfig();

        result["status"] = "success";
        result["message"] = "仿真已成功结束";

        return 0;
    }

    // 初始化仿真配置
    void SimManager::init_sim_config(const Json::Value& input, Json::Value& result) {

        //干扰源位置初始化
        Cmd_Type type = static_cast<Cmd_Type>(input.get("cmd", 4).asInt());
        if (type == Cmd_Type::Barrage)
        {
            // 解析输入参数
            Jammer_Level jammer_strength = static_cast<Jammer_Level>(input["jammer_level"].asInt());
            int jammer_num = input.get("jammer_num", 1).asInt();

            // 设置beta参数
            if (jammer_strength == Jammer_Level::High) {
                barrage_config.beta = 5e4;
            }
            else if (jammer_strength == Jammer_Level::Middle) {
                barrage_config.beta = 9e4;
            }
            else if (jammer_strength == Jammer_Level::Low) {
                barrage_config.beta = 2e5;
            }

            //干扰源添加
            for (int i = 0; i < jammer_num; i++)
            {
                // 添加干扰源（每个干扰源位置略有偏移）
                JammerParam jammer;
                // 干扰源位置：基础位置 + 偏移，确保每个干扰源位置不同
                jammer.pos = { 120.0 + i * 0.2, 27.63 + i * 0.1, 8.3 };
                // 配置干扰源
                jammer.power = 10.0; // W
                jammer.type = "continuous_wave";
                jammer.bandwidth = 20e6;
                jammer.freq = GNSS_FC;
                barrage_config.jammers.push_back(jammer);
            }

            //失锁范围计算
            vector<JammerRangeResult> jammer_range_result;
            Barrage_CalcjammerArea(barrage_config, jammer_range_result);

            for (int cnt = 0;cnt < jammer_range_result.size();cnt++)
            {
                //干扰范围
                //Json::Value& jammer_area_json = trajectory_result["jammer area"];
                Json::Value jammer_mes;
                jammer_mes["jammer id"] = jammer_range_result[cnt].jammer_id;
                jammer_mes["jammer pos centra"]["lon_deg"] = jammer_range_result[cnt].jammer_centre_lla.lon_deg;
                jammer_mes["jammer pos centra"]["lat_deg"] = jammer_range_result[cnt].jammer_centre_lla.lat_deg;
                jammer_mes["jammer pos centra"]["h_m"] = jammer_range_result[cnt].jammer_centre_lla.h_m * 1000;
                jammer_mes["jammer r"] = jammer_range_result[cnt].jammer_radius; //m
                result.append(jammer_mes);
            }
        }
        else if (type == Cmd_Type::Deception)
        {
            // 解析输入参数
            Jammer_Level jammer_strength = static_cast<Jammer_Level>(input["jammer_level"].asInt());
            //int jammer_num = input.get("jammer_num", 1).asInt();

            // 设置beta参数
            if (jammer_strength == Jammer_Level::High) {
                
            }
            else if (jammer_strength == Jammer_Level::Middle) {
                
            }
            else if (jammer_strength == Jammer_Level::Low) {
                
            }
            
            for (int cnt = 0; cnt < deception_config.jammer_num; cnt++)
            {
                //干扰范围
                //Json::Value& jammer_area_json = trajectory_result["jammer area"];
                Json::Value jammer_mes;
                jammer_mes["jammer id"] = cnt + 1;
                jammer_mes["jammer pos centra"]["lon_deg"] = deception_config.jammer_pos[cnt].lon_deg;
                jammer_mes["jammer pos centra"]["lat_deg"] = deception_config.jammer_pos[cnt].lat_deg;
                jammer_mes["jammer pos centra"]["h_m"] = deception_config.jammer_pos[cnt].h_m * 1000;
                jammer_mes["jammer r"] = 0; //m
                result.append(jammer_mes);
            }

        }
        else if (type == Cmd_Type::Transformation)
        {
            formation_param_.num_uavs = input["num_uavs"].asInt();
            formation_param_.interval = input["interval"].asDouble();
            formation_param_.collision_radius = input["collision_radius"].asDouble();
            //formation_param_.switch_interval = input["switch_interval"].asDouble();
            formation_param_.max_frames = input["max_frames"].asInt();
            formation_param_.trans_formation = static_cast<Formation_Type>(input["formation"].asInt());
            formation_param_.pos_center = Point2D(input["pos_center_x"].asDouble(), input["pos_center_y"].asDouble());

            Init_formation(result);
        }
    }
    
}