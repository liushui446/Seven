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
#include "process/CalcParamRes.hpp"
#include <random>

using namespace Json;

namespace seven {

    SimManager::SimManager() : sim_state_(SimState::ENDDING), sim_time_(2000), calc_thread_ptr(nullptr){}

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
            Json::Value intial_data;
            Cmd_Type type = static_cast<Cmd_Type>(input.get("cmd", 4).asInt());
            init_sim_config(input, intial_data);

            //设置最大运行帧数
            CalcParamManager::Ins().SetSimTime(sim_time_);
            if (type == Cmd_Type::Transformation)
            {
                result["node_list"] = intial_data;
            }
            else
            {
                result["jammer_list"] = intial_data;
            }

            if (calc_thread_ptr == nullptr)
            {
                // 1. 获取线程管理器实例
                calc_thread_ptr = seven::CalcProcessThread::GetInstance();

                // 2. 启动线程工作
                calc_thread_ptr->StartWork(true);
            }
            

            result["status"] = "success";
            result["message"] = "sim started success";

            // 更新仿真状态
            sim_state_ = SimState::IDLE;
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

        int cmd_int = input.get("cmd", 4).asInt();
        if (cmd_int != 1 && cmd_int != 2 && cmd_int != 3) {
            result["status"] = "error";  // 字符串值
            result["message"] = std::string("parser command fail:");  // 拼接字符串
            result["data"] = Json::nullValue;  // 空值（替代 nullptr，JsonCpp 专用）
            return -1;
        }
        Cmd_Type type = static_cast<Cmd_Type>(cmd_int);
        //状态判断
        if (type == Cmd_Type::Transformation)
        {
            if (sim_state_ == SimState::ENDDING)
            {
                result["status"] = "error";
                result["message"] = "sim end, please restart";
                return -1;
            }
        }
        else
        {
            // 检查仿真状态
            if (sim_state_ != SimState::IDLE && sim_state_ != SimState::STOPPED) {
                if (sim_state_ == SimState::RUNNING)
                {
                    result["status"] = "error";
                    result["message"] = "sim is running";
                    return -1;
                }
                else {
                    result["status"] = "error";
                    result["message"] = "sim end, please restart";
                    return -1;
                }
            }
        }

        try {
            result.clear();
            result["status"] = "success";
            // 更新仿真状态
            sim_state_ == SimState::RUNNING;

            std::cerr << "仿真开始..." << std::endl;

            // 3. 提交任务（内部会自动唤醒线程）
            Json::Value output;
            bool ret;
            if (type == Cmd_Type::Barrage)
            {
                vector<InputPlatParam> temp_platsparam_ = ContextManager::Ins().GetBarrageParams().platsparam;
                CalcParamManager::Ins().SwapPlatform(temp_platsparam_);
                ret = calc_thread_ptr->SubmitTask(hPipe, input, output);
            }
            else if (type == Cmd_Type::Deception)
            {
                vector<InputPlatParam> temp_platsparam_ = ContextManager::Ins().GetDeceptionParams().platsparam;
                CalcParamManager::Ins().SwapPlatform(temp_platsparam_);
                ret = calc_thread_ptr->SubmitTask(hPipe, input, output);
            }
            else if (type == Cmd_Type::Transformation)
            {
                ret = calc_thread_ptr->SubmitTask(hPipe, input, output);
                /*UAVFormationParams params = ContextManager::Ins().GetFormationParams();
                vector<TrajectoryFrame> initial_trajectory = ContextManager::Ins().GetInitialTrajectory();
                vector<TrajectoryFrame> end_trajectory = ContextManager::Ins().GetEndTrajectory();
                Transformation_Use(params, initial_trajectory, end_trajectory, input, result);
                ContextManager::Ins().SetFormationParams(params);
                ContextManager::Ins().SetInitialTrajectory(initial_trajectory);
                ContextManager::Ins().SetEndTrajectory(end_trajectory);*/
            }
            if (ret) {}

        }
        catch (const std::exception& e) {

            calc_thread_ptr->UnInit();
            sim_state_ == SimState::ENDDING;
            result["status"] = "error";
            result["message"] = std::string("calc fail: ") + e.what();
            return -1;
        }

        return 0;
    }

    // 3. 仿真暂停接口
    int SimManager::sim_stop(Json::Value& result) {
        std::lock_guard<std::mutex> lock(sim_mutex_);

        result.clear();

        if (sim_state_ != SimState::RUNNING) {
            result["status"] = "warning";
            result["message"] = "sim is not running, do not stop";
            return 0;
        }

        calc_thread_ptr->Interrupted();

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

        //重置运行帧数
        CalcParamManager::Ins().SetRunFramesCnt(0);

        //calc_thread_ptr->UnInit();

        // 更新仿真状态
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
            SimConfig barrage_config;
            // 解析输入参数
            Jammer_Level jammer_strength = static_cast<Jammer_Level>(input["jammer_level"].asInt());
            int jammer_num = input.get("jammer_num", 1).asInt();
            UINT return_frames = input.get("return_frames", 100).asInt();
            CalcParamManager::Ins().SetReturnFramesCount(return_frames);

            // 1.设置beta参数
            if (jammer_strength == Jammer_Level::High) {
                barrage_config.beta = 5e4;
            }
            else if (jammer_strength == Jammer_Level::Middle) {
                barrage_config.beta = 9e4;
            }
            else if (jammer_strength == Jammer_Level::Low) {
                barrage_config.beta = 2e5;
            }

            // 2.干扰源添加
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

            // 3.配置卫星参数
            barrage_config.satellite.carrier_power = 1e-16;
            barrage_config.satellite.sat_pos.clear();
            barrage_config.satellite.sat_pos = {
                {125.0, 30.0, 5000},
                {115.0, 35.0, 5000},
                {130.0, 25.0, 5000},
                {110.0, 28.0, 5000}
            };

            // 4.失锁范围计算
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

            // 5.解析多平台目标数据
            // 解析多平台航迹数据
            
            const Json::Value& platform_tracks = input["platform_tracks"];
            for (int i = 0; i < platform_tracks.size(); i++) {
                const Json::Value& plat_json = platform_tracks[i];
                InputPlatParam plat_data;

                // ===== 解析平台ID（必选字段）=====
                if (!plat_json.isMember("platform_id") || !plat_json["platform_id"].isUInt()) {
                    std::cerr << "警告：第" << i << "个平台缺少有效platform_id，跳过该平台！" << std::endl;
                    continue;
                }
                plat_data.plat_id = plat_json["platform_id"].asUInt();

                // ===== 解析初始位置inital_pos（必选字段，数组）=====
                const Json::Value& inital_pos_arr = plat_json["inital_pos"];
                if (!inital_pos_arr.isArray() || inital_pos_arr.empty()) {
                    std::cerr << "警告：平台" << plat_data.plat_id << "缺少有效inital_pos数组，跳过！" << std::endl;
                    continue;
                }
                const Json::Value& inital_pos_json = inital_pos_arr[0];  // 取第一个位置点
                LLA initial_pos;
                // 字段默认值+存在性检查，避免缺失字段导致解析错误
                initial_pos.lon_deg = inital_pos_json.isMember("lon_deg") ? inital_pos_json["lon_deg"].asDouble() : 0.0;
                initial_pos.lat_deg = inital_pos_json.isMember("lat_deg") ? inital_pos_json["lat_deg"].asDouble() : 0.0;
                initial_pos.h_m = inital_pos_json.isMember("h_m") ? inital_pos_json["h_m"].asDouble() / 1000 : 0.0;  // 米转千米

                // ===== 解析速度velocity（必选字段，数组）=====
                const Json::Value& velocity_arr = plat_json["velocity"];
                if (!velocity_arr.isArray() || velocity_arr.empty()) {
                    std::cerr << "警告：平台" << plat_data.plat_id << "缺少有效velocity数组，跳过！" << std::endl;
                    continue;
                }
                const Json::Value& velocity_json = velocity_arr[0];  // 取第一个速度点
                LLA plat_vec;
                plat_vec.lon_deg = velocity_json.isMember("lon_deg") ? velocity_json["lon_deg"].asDouble() : 0.0;
                plat_vec.lat_deg = velocity_json.isMember("lat_deg") ? velocity_json["lat_deg"].asDouble() : 0.0;
                plat_vec.h_m = velocity_json.isMember("h_m") ? velocity_json["h_m"].asDouble() : 0.0;

                // ===== 赋值到平台参数 =====
                plat_data.plat_initial_pos = initial_pos;
                plat_data.cur_plat_pos = initial_pos;  // 初始位置=当前位置
                plat_data.cur_plat_vec = plat_vec;     // 速度=JSON中的velocity

                barrage_config.platsparam.push_back(plat_data);
            }
            ContextManager::Ins().SetBarrageParams(barrage_config);
            std::cout << "platsparam size:" << to_string(barrage_config.platsparam.size()) << std::endl;
        }
        else if (type == Cmd_Type::Deception)
        {
            // 解析输入参数
            Jammer_Level jammer_strength = static_cast<Jammer_Level>(input["jammer_level"].asInt());
            UINT return_frames = input.get("return_frames", 100).asInt();
            CalcParamManager::Ins().SetReturnFramesCount(return_frames);

            SimParams deception_config;

            // 1. 偏离角度：单位°，范围0~180，正值为顺时针偏离，负值为逆时针偏离（相对于原航迹方向）
            double deviation_angle_deg = 0;
            // 1.根据干扰强度设置参数（可在这里改偏移大小逻辑）
            if (jammer_strength == Jammer_Level::High) {
                
                deviation_angle_deg = -65;
            }
            else if (jammer_strength == Jammer_Level::Middle) {
                // 默认
                deviation_angle_deg = -45;
            }
            else if (jammer_strength == Jammer_Level::Low) {
                // 低干扰：
                deviation_angle_deg = -20;
            }

            // 2.干扰源添加
            for (int cnt = 0; cnt < deception_config.jammer_num; cnt++)
            {
                Json::Value jammer_mes;
                jammer_mes["jammer id"] = cnt + 1;
                jammer_mes["jammer pos centra"]["lon_deg"] = deception_config.jammer_pos[cnt].lon_deg;
                jammer_mes["jammer pos centra"]["lat_deg"] = deception_config.jammer_pos[cnt].lat_deg;
                jammer_mes["jammer pos centra"]["h_m"] = deception_config.jammer_pos[cnt].h_m * 1000;
                jammer_mes["jammer r"] = 0; //m
                result.append(jammer_mes);
            }

            // 3.解析多平台目标数据
            const Json::Value& platform_tracks = input["platform_tracks"];

            // ==============================
            // 新增：自定义欺骗航迹参数（可根据需求修改）
            // ==============================
            // 2. 航迹步数：step，每步对应原速度的一个时间单位，步数越多，欺骗点离初始位置越远
            const int step = 800;                       // 示例：5步，可自行修改
            // 3. 角度转弧度（用于三角函数计算）
            const double deviation_angle_rad = deviation_angle_deg * M_PI / 180.0;

            for (int i = 0; i < platform_tracks.size(); i++) {
                const Json::Value& plat_json = platform_tracks[i];
                InputPlatParam plat_data;

                // ===== 解析平台ID =====
                if (!plat_json.isMember("platform_id") || !plat_json["platform_id"].isUInt()) {
                    std::cerr << "警告：第" << i << "个平台缺少有效platform_id，跳过该平台！" << std::endl;
                    continue;
                }
                plat_data.plat_id = plat_json["platform_id"].asUInt();

                // ===== 解析初始位置 inital_pos =====
                const Json::Value& inital_pos_arr = plat_json["inital_pos"];
                if (!inital_pos_arr.isArray() || inital_pos_arr.empty()) {
                    std::cerr << "警告：平台" << plat_data.plat_id << "缺少有效inital_pos数组，跳过！" << std::endl;
                    continue;
                }
                const Json::Value& inital_pos_json = inital_pos_arr[0];
                LLA initial_pos;
                initial_pos.lon_deg = inital_pos_json.isMember("lon_deg") ? inital_pos_json["lon_deg"].asDouble() : 0.0;
                initial_pos.lat_deg = inital_pos_json.isMember("lat_deg") ? inital_pos_json["lat_deg"].asDouble() : 0.0;
                initial_pos.h_m = inital_pos_json.isMember("h_m") ? inital_pos_json["h_m"].asDouble() / 1000 : 0.0;

                // ===== 解析速度 velocity =====
                const Json::Value& velocity_arr = plat_json["velocity"];
                if (!velocity_arr.isArray() || velocity_arr.empty()) {
                    std::cerr << "警告：平台" << plat_data.plat_id << "缺少有效velocity数组，跳过！" << std::endl;
                    continue;
                }
                const Json::Value& velocity_json = velocity_arr[0];
                LLA plat_vec;
                plat_vec.lon_deg = velocity_json.isMember("lon_deg") ? velocity_json["lon_deg"].asDouble() : 0.0;
                plat_vec.lat_deg = velocity_json.isMember("lat_deg") ? velocity_json["lat_deg"].asDouble() : 0.0;
                plat_vec.h_m = velocity_json.isMember("h_m") ? velocity_json["h_m"].asDouble() : 0.0;

                // ===== 赋值平台参数 =====
                plat_data.plat_initial_pos = initial_pos;
                plat_data.cur_plat_pos = initial_pos;
                plat_data.cur_plat_vec = plat_vec;

                // ==============================
                // 核心修改：按偏离角度+航迹步数，计算欺骗点位置
                // 逻辑：1.计算原速度的大小和方向 2.按偏离角度计算欺骗速度 3.按步数计算总偏移量 4.推导欺骗点位置
                // ==============================
                LLA deception_pos;
                // 高度完全继承初始高度，不做任何修改（与原逻辑一致）
                deception_pos.h_m = initial_pos.h_m;

                // 1. 计算原航迹速度的大小（模长），避免除以0
                double original_vec_norm = std::sqrt(plat_vec.lon_deg * plat_vec.lon_deg + plat_vec.lat_deg * plat_vec.lat_deg);
                if (original_vec_norm < 1e-6) {
                    // 异常处理：原速度为0，无法计算偏离方向，退化为小幅随机偏移（避免欺骗点与初始位置重合）
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_real_distribution<double> small_offset(-0.0001, 0.0001);
                    deception_pos.lon_deg = initial_pos.lon_deg + small_offset(gen);
                    deception_pos.lat_deg = initial_pos.lat_deg + small_offset(gen);
                    std::cerr << "警告：平台" << plat_data.plat_id << "原速度为0，欺骗点退化为小幅随机偏移！" << std::endl;
                }
                else {
                    // 2. 计算原速度的方向单位向量（归一化）
                    double original_lon_dir = plat_vec.lon_deg / original_vec_norm;  // 经度方向单位向量
                    double original_lat_dir = plat_vec.lat_deg / original_vec_norm;  // 纬度方向单位向量

                    // 3. 按偏离角度，计算欺骗速度的方向（旋转原速度方向）
                    // 旋转公式：x' = x*cosθ - y*sinθ，y' = x*sinθ + y*cosθ（θ为偏离角度，弧度）
                    double deception_lon_dir = original_lon_dir * cos(deviation_angle_rad) - original_lat_dir * sin(deviation_angle_rad);
                    double deception_lat_dir = original_lon_dir * sin(deviation_angle_rad) + original_lat_dir * cos(deviation_angle_rad);

                    // 4. 计算欺骗速度（大小与原速度一致，方向为偏离后的方向）
                    double deception_vec_lon = deception_lon_dir * original_vec_norm;
                    double deception_vec_lat = deception_lat_dir * original_vec_norm;

                    // 5. 按航迹步数step，计算总偏移量（总偏移 = 欺骗速度 * 步数）
                    double total_lon_offset = deception_vec_lon * step;
                    double total_lat_offset = deception_vec_lat * step;

                    // 6. 计算最终欺骗点位置（初始位置 + 总偏移量）
                    deception_pos.lon_deg = initial_pos.lon_deg + total_lon_offset;
                    deception_pos.lat_deg = initial_pos.lat_deg + total_lat_offset;
                }

                // 存入欺骗点（确保 InputPlatParam 里有 deception_pos 字段，与原逻辑一致）
                plat_data.deception_pos = deception_pos;

                deception_config.platsparam.push_back(plat_data);

                // 调试打印（优化打印信息，新增偏离角度、步数、欺骗速度相关信息）
                std::cout << "平台 " << plat_data.plat_id << " 欺骗点初始化（按偏离角度+航迹步数）：" << std::endl;
                std::cout << "  初始经纬度：" << initial_pos.lon_deg << ", " << initial_pos.lat_deg << std::endl;
                std::cout << "  原速度（lon/lat）：" << plat_vec.lon_deg << ", " << plat_vec.lat_deg << std::endl;
                std::cout << "  自定义参数：偏离角度" << deviation_angle_deg << "°，航迹步数" << step << std::endl;
                std::cout << "  欺骗经纬度：" << deception_pos.lon_deg << ", " << deception_pos.lat_deg << std::endl;
                std::cout << "  高度不变：" << deception_pos.h_m << " km" << std::endl;
            }

            ContextManager::Ins().SetDeceptionParams(deception_config);
            std::cout << "platsparam size: " << deception_config.platsparam.size() << std::endl;
        }
        else if (type == Cmd_Type::Transformation)
        {
            vector<TrajectoryFrame> initial_trajectory;
            vector<TrajectoryFrame> end_trajectory;
            UAVFormationParams formation_param_;
            formation_param_.num_uavs = input["num_uavs"].asInt();
            formation_param_.interval = input["interval"].asDouble();
            formation_param_.collision_radius = input["collision_radius"].asDouble();
            //formation_param_.switch_interval = input["switch_interval"].asDouble();
            formation_param_.max_frames = input["max_frames"].asInt();
            formation_param_.trans_formation = static_cast<Formation_Type>(input["formation"].asInt());
            //formation_param_.pos_center = Point2D(input["pos_center_x"].asDouble(), input["pos_center_y"].asDouble());

            /*std::cout << "===== 解析JSON取值验证 =====" << std::endl;
            std::cout << "input[\"num_uavs\"] = " << input["num_uavs"].asInt() << std::endl;
            std::cout << "input[\"interval\"] = " << input["interval"].asDouble() << std::endl;
            std::cout << "input[\"formation\"] = " << input["formation"].asInt() << std::endl;
            std::cout << "===== 赋值后formation_param_数值 =====" << std::endl;
            std::cout << "num_uavs = " << formation_param_.num_uavs << std::endl;
            std::cout << "interval = " << formation_param_.interval << std::endl;
            std::cout << "formation = " << static_cast<int>(formation_param_.trans_formation) << std::endl;*/

            Init_formation(formation_param_, initial_trajectory, end_trajectory, result);
            ContextManager::Ins().SetFormationParams(formation_param_);
            ContextManager::Ins().SetInitialTrajectory(initial_trajectory);
            ContextManager::Ins().SetEndTrajectory(end_trajectory);
        }
    }
    
}