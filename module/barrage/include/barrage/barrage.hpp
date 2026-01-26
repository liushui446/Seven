#ifndef BARRAGE_HPP
#define BARRAGE_HPP

#include "core/CommonCore.hpp"

// 宏定义常量
#define GNSS_FC 1575.42e6 // GNSS中心频率(GPS L1频段, Hz)

namespace seven {

    // ========================= 1. 结构体封装参数 =========================

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
        double beta = 2e6;         // 接收机等效预相关带宽(Hz)
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

        // 卫星参数
        SatelliteParam satellite;
    };

    // 输出结果结构体
    struct BarrageTrackResult {
        double C_NJ_dB;            // 载噪比(dB-Hz)
        double J_S_dB;             // 干信比(dB)
        double sigma_jpll;         // 载波环振荡器颤动(°)
        double sigma_jdll;         // 码环跟踪误差
        bool unlock_flag;          // 失锁标志（true=失锁）
        LLA pos_error;             // 定位误差(°/km)
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

        // 经纬度转ECEF
        ECEF lla_to_ecef(const LLA& lla);

        // ECEF转经纬度
        LLA ecef_to_lla(const ECEF& ecef);

        // 功率谱密度计算
        void calc_power_spectral_density(double f, const SimConfig& config, int jammer_idx, double& GJ, double& GS);

        // 数值积分（梯形法近似，替代scipy.integrate.quad）
        double integrate_quad(std::function<double(double)> func, double start, double end, int steps = 1000);

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

}

#endif
