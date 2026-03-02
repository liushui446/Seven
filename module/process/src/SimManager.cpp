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
#include <process/CalcThread.hpp>

using namespace Json;

namespace seven {

    SimManager::SimManager() : sim_state_(SimState::STOPPED), sim_time_(400) {}

    // 1. 仿真开始接口
    int SimManager::sim_start(const Json::Value& input, Json::Value& result) {
        std::lock_guard<std::mutex> lock(sim_mutex_);

        // 检查当前状态
        if (sim_state_ == SimState::RUNNING) {
            result["status"] = "error";
            result["message"] = "sim is running, do not restart";
            return -1;
        }

        try {
            // 构造返回结果（干扰源位置JSON）
            result.clear();
            // 初始化配置
            Json::Value& jammer_list = result["jammer_list"];
            init_sim_config(input, jammer_list);

            result["status"] = "success";
            result["message"] = "sim started success";

            // 更新仿真状态
            sim_state_ = SimState::RUNNING;
        }
        catch (const std::exception& e) {
            result["status"] = "error";
            result["message"] = std::string("sim started fail: ") + e.what();
            return -1;
        }

        return 0;
    }

    int SimManager::sim_calc(HANDLE hPipe, const Json::Value& input, Json::Value& result) {
        std::lock_guard<std::mutex> lock(sim_mutex_);

        // 检查仿真状态
        if (sim_state_ != SimState::IDLE || sim_state_ != SimState::STOPPED) {
            result["status"] = "error";
            result["message"] = "sim end, please restart";
            return -1;
        }

        try {
            result.clear();
            result["status"] = "success";

            // 解析多平台航迹数据
            const Json::Value& platform_tracks = input["platform_tracks"];
            Json::Value& platform_results = result["platform_results"];

            GNSSJammerSim sim;

            int cmd_int = input.get("cmd", 4).asInt();
            if (cmd_int != 1 || cmd_int != 2 || cmd_int != 3) {
                result["status"] = "error";  // 字符串值
                result["message"] = std::string("parser command fail:");  // 拼接字符串
                result["data"] = Json::nullValue;  // 空值（替代 nullptr，JsonCpp 专用）
                return -1;
            }

            // 1. 获取线程管理器实例
            auto calc_thread = seven::CalcProcessThread::GetInstance();

            // 2. 启动线程工作
            calc_thread->StartWork(true);

            // 3. 提交任务（内部会自动唤醒线程）
            Json::Value output;
            bool ret = calc_thread->SubmitTask(hPipe, input, output);

            if (ret) {

            }

            Cmd_Type type = static_cast<Cmd_Type>(cmd_int);
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
        }
        catch (const std::exception& e) {
            result["status"] = "error";
            result["message"] = std::string("calc fail: ") + e.what();
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
            result["message"] = "sim is not running, do not stop";
            return 0;
        }

        // 重置状态和配置
        sim_state_ = SimState::STOPPED;

        result["status"] = "success";
        result["message"] = "sim stop";

        return 0;
    }

    int SimManager::sim_end(Json::Value& result)
    {
        std::lock_guard<std::mutex> lock(sim_mutex_);

        result.clear();

        if (sim_state_ == SimState::ENDDING) {
            result["status"] = "warning";
            result["message"] = "sim done, need not end";
            return 0;
        }

        // 重置状态和配置
        sim_state_ = SimState::ENDDING;

        result["status"] = "success";
        result["message"] = "sim done";

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
            barrage_config.return_frames = input.get("return_frames", 100).asInt();

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
            barrage_config.jammers.clear();
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

            // 配置卫星参数
            barrage_config.satellite.carrier_power = 1e-16;
            barrage_config.satellite.sat_pos.clear();
            barrage_config.satellite.sat_pos = {
                {125.0, 30.0, 5000},
                {115.0, 35.0, 5000},
                {130.0, 25.0, 5000},
                {110.0, 28.0, 5000}
            };

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
            deception_config.return_frames = input.get("return_frames", 100).asInt();

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