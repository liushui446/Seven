#ifndef DECEPTION_HPP
#define DECEPTION_HPP

#include "core/CommonCore.hpp"

namespace seven {

    // 仿真参数结构体
    struct SimParams {
        // 基础阈值
        double GDOP_threshold = 3.0;
        double power_ratio_threshold = 5.0;  // 功率比阈值(dB)

        // 干扰装备参数
        int jammer_num = 4;
        std::vector<LLA> jammer_pos = {
            {115.2, 29.0, 1.0},
            {115.3, 29.1, 1.0},
            {115.1, 29.2, 1.0},
            {115.4, 29.0, 1.0}
        };

        // 欺骗点位置
        LLA deception_pos = { 115.32, 29.15, 0.5 };

        // 卫星参数(默认10颗)
        std::vector<LLA> satellite_pos = {
            {120.0, 30.0, 20000},
            {110.0, 35.0, 20000},
            {130.0, 25.0, 20000},
            {105.0, 28.0, 20000},
            {140.0, 32.0, 20000},
            {100.0, 22.0, 20000},
            {125.0, 40.0, 20000},
            {115.0, 20.0, 20000},
            {135.0, 27.0, 20000},
            {95.0, 33.0, 20000}
        };
    };

    // 航迹计算结果结构体
    struct TrackResult {
        LLA target_pos;          // 目标位置
        LLA error_pos;           // 受干扰后位置
        LLA pos_error;           // 定位误差(经纬度高)
        bool deception_valid;    // 欺骗信号是否有效
    };

    // GNSS欺骗干扰误差计算类
    class GNSSDeceptionError {
    private:
        SimParams params;
        double C_LIGHT = 3e8;            // 光速(m/s)
        double A_length = 6378137.0;      // 地球长半轴(m)
        double F_ratio  = 1.0 / 298.257223563;  // 地球扁率
        double FREQ = 1561.09e6;   // 北斗B1载波频率(Hz)

        // 角度转弧度
        double deg2rad(double deg) const;

        // 弧度转角度
        double rad2deg(double rad) const;

        // LLA转ECEF(米)
        ECEF lla_to_ecef(const LLA& lla) const;

        // ECEF转LLA
        LLA ecef_to_lla(const ECEF& ecef) const;

        // 计算两点间ECEF距离(米)
        double calc_ecef_distance(const ECEF& p1, const ECEF& p2) const;

        // 筛选GDOP最优的4颗卫星
        std::vector<LLA> select_optimal_satellites(const LLA& target_pos) const;

        // 计算信号相对功率
        double calc_signal_power(const LLA& trans_pos, const LLA& recv_pos) const;

        // 判断欺骗信号是否有效
        bool is_deception_valid(const LLA& target_pos, const std::vector<LLA>& satellite_pos) const;

        // 计算欺骗时延
        std::vector<double> calculate_deception_delay(const LLA& target_pos,
            const std::vector<LLA>& satellite_pos) const;

        // 求解定位误差
        LLA solve_position_error(const std::vector<LLA>& satellite_pos,
            const LLA& target_pos,
            const std::vector<double>& delays) const;

    public:
        // 构造函数
        GNSSDeceptionError() = default;
        explicit GNSSDeceptionError(const SimParams& custom_params) : params(custom_params) {}

        // 设置自定义参数
        void set_params(const SimParams& custom_params);

        // 核心计算接口：输入航迹点，输出误差结果
        TrackResult calculate_error(const LLA& target_pos);

        // 批量处理航迹数据
        std::vector<TrackResult> batch_calculate(const std::vector<LLA>& track_points);
    };

    int SEVEN_EXPORTS Deception_Test(Json::Value input, Json::Value& trajectory_result);
    
}

#endif
