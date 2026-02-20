#ifndef BARRAGE_HPP
#define BARRAGE_HPP

#include "core/CommonCore.hpp"

// 宏定义常量
#define GNSS_FC 1575.42e6 // GNSS中心频率(GPS L1频段, Hz)

namespace seven {

    // ========================= 1. 结构体封装参数 =========================
    /*enum class Jammer_Level {
        Strong = 1,
        Middle = 2,
        Weak
    };*/

    // 干扰源参数结构体
    struct JammerParam {
        LLA pos;             // 干扰源经纬高(°/km)
        double power;        // 发射功率(W)
        std::string type;    // 干扰样式：continuous_wave/multi-tone/bandlimited_gaussian/pseudocode/pulse
        double bandwidth;    // 干扰带宽(Hz)
        double freq;         // 干扰频率(Hz)
        double pulse_width;  // 脉冲宽度(s)（仅脉冲干扰）
        double pulse_period; // 脉冲周期(s)（仅脉冲干扰）
    };

    // 卫星参数结构体
    struct SatelliteParam {
        std::vector<LLA> sat_pos;  // 卫星经纬高列表(°/km)
        double carrier_power;      // 卫星信号载波功率(W)
    };

    // 核心仿真配置结构体
    struct SimConfig {
        // 基础参数
        double Td = 0.02;          // 相关积分时间(s)
        double c = 3e8;        // 光速(m/s)
        double fc = GNSS_FC;       // GNSS中心频率(Hz)

        // 导航装备参数
        std::string combined_nav = "loose";  // 组合导航方式：loose/tight/deep
        std::string anti_jam_filter = "frequency"; // 抗干扰滤波方式
        std::string pseudocode = "C/A";      // 伪码类型：C/A/P(Y)/M
        double Tc = 9.77e-7;       // 伪码码元宽度(s)
        double fs = 10.23e6;      // M码副载频(Hz)
        double d = 1.0 / 8;          // 码跟踪误差系数
        double beta = 2e5;         // 接收机等效预相关带宽(Hz)2e6
        double ins_drift = 0.01;   // 惯导漂移率(km/s)

        // 跟踪环参数
        double Bp = 18;            // PLL带宽(Hz)
        double Bd = 18;            // DLL带宽(Hz)

        // 失锁判定阈值
        double pll_unlock_thresh = 15;  // 载波环失锁阈值(°)
        double dll_unlock_thresh = 1.0 / 8 / 6; // 码环失锁阈值

        // 标称载噪比
        double C_N0_nom = 45;      // dB-Hz
        double Bn = 2.046e6;       // 噪声带宽 (Hz)
        double G_ant = 10;         // 抗干扰波束成形增益 (dB)

        // 干扰源列表
        std::vector<JammerParam> jammers;
        //Jammer_Level jammer_lev = Jammer_Level::Weak;

        // 卫星参数
        SatelliteParam satellite;
    };

    // 输出结果结构体
    struct BarrageTrackResult {
        double C_NJ_dB;            // 载噪比(dB-Hz)
        double J_S_dB;             // 干信比(dB)
        double sigma_jpll;         // 载波环振荡器颤动(°)
        double sigma_jdll;         // 码环跟踪误差
        bool unlock_flag;          // 失锁标志(true=失锁)
        LLA pos_error;             // 定位误差(°/m)
        LLA target_pos;            // 目标位置
        ECEF pos_error_m;          // 定位误差(米)
        double gdop;               // 几何精度因子
    };

    // ========================= 2. 核心算法类封装 =========================
    class GNSSJammerSim {
    private:
        // WGS84椭球参数
        const double a = 6378137.0;    // 长半轴(m)
        const double f = 1.0 / 298.257223563; // 扁率
        const double e2 = 2 * f - f * f;   // 第一偏心率平方
        const double b = a * (1 - f);  // 短半轴(m)
        const double ep2 = (a * a - b * b) / (b * b); // 第二偏心率平方

    public:
        // 经纬度转ECEF
        ECEF lla_to_ecef(const LLA& lla);

        // ECEF转经纬度
        LLA ecef_to_lla(const ECEF& ecef);

    private:
        // 功率谱密度计算
        void calc_power_spectral_density(double f, const SimConfig& config, int jammer_idx, double& GJ, double& GS);

        // 数值积分（梯形法近似，替代scipy.integrate.quad）
        double integrate_quad(std::function<double(double)> func, double start, double end, int steps = 1000);

        double integrate_quad2(std::function<double(double)> func, double start, double end, int steps = 1000);

        // sinc函数实现
        double sinc(double x);

        // APM电磁传播模型计算干扰接收功率
        double calc_jam_power_apm(const JammerParam& jammer, const LLA& target_pos, const SimConfig& config);

        // 载噪比计算
        double calc_cnr(double Pj, const SimConfig& config, int jammer_idx);

        // 干信比计算
        double calc_jammer_to_signal_ratio(double Pj, const LLA& target_pos, const SimConfig& config);

        // 跟踪环误差计算
        void calc_tracking_errors(double C_NJ_dB, double J_S_dB, const SimConfig& config, int jammer_idx, double& sigma_jpll, double& sigma_jdll);

        // GDOP计算
        double calc_gdop(const LLA& target_pos, const std::vector<LLA>& satellite_pos);

        // 定位误差计算
        void calc_pos_error(bool unlock_flag, double sigma_jdll, double gdop, const SimConfig& config, LLA& pos_error, ECEF& pos_error_m);

    public:
        // 核心接口：输入航迹点和配置，输出结果
        BarrageTrackResult calc_track_result(const LLA& target_pos, const SimConfig& config);

        // 批量处理航迹数据接口
        std::vector<BarrageTrackResult> batch_calc(const std::vector<LLA>& track_points, const SimConfig& config);
    };

    int SEVEN_EXPORTS Barrage_Test(Json::Value input, Json::Value& trajectory_result);

    // 核心仿真管理器类
    class BarrageSimManager {
    public:
        enum class SimState {
            STOPPED = 0,  // 仿真已停止
            RUNNING = 1   // 仿真运行中
        };

        BarrageSimManager() : sim_state_(SimState::STOPPED), sim_time_(400) {}

        // 1. 仿真开始接口
        int sim_start(const Json::Value& input, Json::Value& result) {
            std::lock_guard<std::mutex> lock(sim_mutex_);

            // 检查当前状态
            if (sim_state_ == SimState::RUNNING) {
                result["status"] = "error";
                result["message"] = "仿真已在运行中，无法重复开始";
                return -1;
            }

            try {
                // 解析输入参数
                Jammer_Level jammer_strength = static_cast<Jammer_Level>(input["jammer_level"].asInt());
                int jammer_num = input.get("jammer_num", 1).asInt();

                // 初始化配置
                init_sim_config(jammer_strength, jammer_num);

                // 构造返回结果（干扰源位置JSON）
                result.clear();
                result["status"] = "success";
                result["message"] = "仿真启动成功";

                Json::Value& jammer_list = result["jammer_list"];
                for (int cnt = 0; cnt < config_.jammers.size(); cnt++) {
                    Json::Value jammer_mes;
                    jammer_mes["id"] = cnt + 1;
                    jammer_mes["pos_lla"]["lon_deg"] = config_.jammers[cnt].pos.lon_deg;
                    jammer_mes["pos_lla"]["lat_deg"] = config_.jammers[cnt].pos.lat_deg;
                    jammer_mes["pos_lla"]["h_m"] = config_.jammers[cnt].pos.h_m;
                    jammer_list.append(jammer_mes);
                }

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

        // 2. 多平台航迹计算接口
        int sim_calc(const Json::Value& input, Json::Value& result) {
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

                // 遍历每个平台的航迹
                for (int i = 0; i < platform_tracks.size(); i++) {
                    UINT platform_id = platform_tracks[i]["platform_id"].asUInt();
                    const Json::Value& track_points_json = platform_tracks[i]["track_points"];

                    // 转换JSON航迹点到内部格式
                    std::vector<LLA> track_points;
                    for (int j = 0; j < track_points_json.size(); j++) {
                        LLA point;
                        point.lon_deg = track_points_json[j]["lon_deg"].asDouble();
                        point.lat_deg = track_points_json[j]["lat_deg"].asDouble();
                        point.h_m = track_points_json[j]["h_m"].asDouble() / 1000.0; // 米转千米
                        track_points.push_back(point);
                    }

                    // 如果只有初始点，生成仿真轨迹
                    if (track_points.size() == 1) {
                        LLA target_velocity = { 0.005, 0.003, 0 };
                        for (int step = 0; step < sim_time_; ++step) {
                            LLA target_pos = track_points[0] + target_velocity * step;
                            track_points.push_back(target_pos);
                        }
                    }

                    // 执行仿真计算
                    std::vector<BarrageTrackResult> calc_results = sim.batch_calc(track_points, config_);

                    // 构造该平台的结果
                    Json::Value platform_result;
                    platform_result["platform_id"] = platform_id;

                    Json::Value& track_results = platform_result["track_results"];
                    for (int j = 0; j < calc_results.size(); j++) {
                        Json::Value track_point;
                        track_point["step"] = j + 1;
                        track_point["pos_tar_lla"]["lon_deg"] = track_points[j].lon_deg;
                        track_point["pos_tar_lla"]["lat_deg"] = track_points[j].lat_deg;
                        track_point["pos_tar_lla"]["h_m"] = track_points[j].h_m * 1000;
                        track_point["cn0_dbhz"] = calc_results[j].C_NJ_dB;
                        track_point["js_dB"] = calc_results[j].J_S_dB;
                        track_point["carrier_loop_error_deg"] = calc_results[j].sigma_jpll;
                        track_point["code_loop_error"] = calc_results[j].sigma_jdll;
                        track_point["unlock_flag_bool"] = calc_results[j].unlock_flag;
                        track_point["pos_error_lla"]["lon_deg"] = calc_results[j].pos_error.lon_deg;
                        track_point["pos_error_lla"]["lat_deg"] = calc_results[j].pos_error.lat_deg;
                        track_point["pos_error_lla"]["h_m"] = calc_results[j].pos_error.h_m * 1000;
                        track_point["pos_error_xyz_m"]["X"] = calc_results[j].pos_error_m.X;
                        track_point["pos_error_xyz_m"]["Y"] = calc_results[j].pos_error_m.Y;
                        track_point["pos_error_xyz_m"]["Z"] = calc_results[j].pos_error_m.Z;

                        track_results.append(track_point);
                    }

                    platform_results.append(platform_result);
                }

                // 添加干扰源信息到结果
                Json::Value& jammer_list = result["jammer_list"];
                for (int cnt = 0; cnt < config_.jammers.size(); cnt++) {
                    Json::Value jammer_mes;
                    jammer_mes["id"] = cnt + 1;
                    jammer_mes["pos_lla"]["lon_deg"] = config_.jammers[cnt].pos.lon_deg;
                    jammer_mes["pos_lla"]["lat_deg"] = config_.jammers[cnt].pos.lat_deg;
                    jammer_mes["pos_lla"]["h_m"] = config_.jammers[cnt].pos.h_m * 1000;
                    jammer_list.append(jammer_mes);
                }
            }
            catch (const std::exception& e) {
                result["status"] = "error";
                result["message"] = std::string("计算失败: ") + e.what();
                return -1;
            }

            return 0;
        }

        // 3. 仿真结束接口
        int sim_stop(Json::Value& result) {
            std::lock_guard<std::mutex> lock(sim_mutex_);

            result.clear();

            if (sim_state_ != SimState::RUNNING) {
                result["status"] = "warning";
                result["message"] = "仿真未运行，无需结束";
                return 0;
            }

            // 重置状态和配置
            sim_state_ = SimState::STOPPED;
            config_ = SimConfig();

            result["status"] = "success";
            result["message"] = "仿真已成功结束";

            return 0;
        }

    private:
        // 初始化仿真配置
        void init_sim_config(Jammer_Level jammer_strength, int jammer_num) {
            // 设置beta参数
            if (jammer_strength == Jammer_Level::High) {
                config_.beta = 5e4;
            }
            else if (jammer_strength == Jammer_Level::Middle) {
                config_.beta = 9e4;
            }
            else if (jammer_strength == Jammer_Level::Low) {
                config_.beta = 2e5;
            }

            // 添加干扰源
            config_.jammers.clear();
            for (int i = 0; i < jammer_num; i++) {
                JammerParam jammer;
                // 为不同干扰源生成不同位置（基础位置 + 偏移）
                jammer.pos = { 120.0 + i * 0.1, 27.63 + i * 0.05, 8.3 };
                jammer.power = 10.0;
                jammer.type = "continuous_wave";
                jammer.bandwidth = 20e6;
                jammer.freq = GNSS_FC;
                config_.jammers.push_back(jammer);
            }

            // 配置卫星参数
            config_.satellite.carrier_power = 1e-16;
            config_.satellite.sat_pos = {
                {125.0, 30.0, 5000},
                {115.0, 35.0, 5000},
                {130.0, 25.0, 5000},
                {110.0, 28.0, 5000}
            };
        }

        SimState sim_state_;          // 仿真状态
        SimConfig config_;            // 仿真配置
        UINT sim_time_;               // 仿真时长
        std::mutex sim_mutex_;        // 线程安全锁
    };

}

#endif
