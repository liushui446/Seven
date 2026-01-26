#include "deception/deception.hpp"
//#include "core/json.hpp"
//
//using namespace Json;
#include <numeric>
#include <cmath>
#include <Eigen/Dense>  // 需Eigen库处理矩阵运算（GDOP计算）

namespace seven {

    // 角度转弧度
    double GNSSDeceptionError::deg2rad(double deg) const {
        return deg * M_PI / 180.0;
    }

    // 弧度转角度
    double GNSSDeceptionError::rad2deg(double rad) const {
        return rad * 180.0 / M_PI;
    }

    // LLA转ECEF(米)
    ECEF GNSSDeceptionError::lla_to_ecef(const LLA& lla) const {
        double lat_rad = deg2rad(lla.lat_deg);
        double lon_rad = deg2rad(lla.lon_deg);
        double h_m = lla.h_km * 1000.0;

        double e2 = 2 * F_ratio - F_ratio * F_ratio;
        double N = A_length / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));

        ECEF ecef;
        ecef.X = (N + h_m) * cos(lat_rad) * cos(lon_rad);
        ecef.Y = (N + h_m) * cos(lat_rad) * sin(lon_rad);
        ecef.Z = ((1 - e2) * N + h_m) * sin(lat_rad);
        return ecef;
    }

    // ECEF转LLA
    LLA GNSSDeceptionError::ecef_to_lla(const ECEF& ecef) const {
        double x = ecef.X;
        double y = ecef.Y;
        double z = ecef.Z;

        double e2 = 2 * F_ratio - F_ratio * F_ratio;
        double b = A_length * (1 - F_ratio);
        double ep2 = (A_length * A_length - b * b) / (b * b);

        double p = sqrt(x * x + y * y);
        LLA lla;

        // 处理极区情况
        if (p < 1e-12) {
            lla.lon_deg = 0.0;
            lla.lat_deg = rad2deg(M_PI / 2 * (z >= 0 ? 1 : -1));
            lla.h_km = (fabs(z) - b) / 1000.0;
            return lla;
        }

        double theta = atan2(z * A_length, p * b);
        double lon_rad = atan2(y, x);
        double lat_rad = atan2(z + ep2 * b * pow(sin(theta), 3),
            p - e2 * A_length * pow(cos(theta), 3));

        double N = A_length / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));
        double h_m = (p / cos(lat_rad)) - N;

        lla.lon_deg = rad2deg(lon_rad);
        lla.lat_deg = rad2deg(lat_rad);
        lla.h_km = h_m / 1000.0;

        return lla;
    }

    // 计算两点间ECEF距离(米)
    double GNSSDeceptionError::calc_ecef_distance(const ECEF& p1, const ECEF& p2) const {
        double dx = p1.X - p2.X;
        double dy = p1.Y - p2.Y;
        double dz = p1.Z - p2.Z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    // 筛选GDOP最优的4颗卫星
    std::vector<LLA> GNSSDeceptionError::select_optimal_satellites(const LLA& target_pos) const {
        // 转换目标到ECEF
        ECEF target_ecef = lla_to_ecef(target_pos);
        std::vector<ECEF> sat_ecef_list;
        for (const auto& sat : params.satellite_pos) {
            sat_ecef_list.push_back(lla_to_ecef(sat));
        }

        double min_gdop = INFINITY;
        std::vector<int> best_idx = { 0, 1, 2, 3 };  // 默认前4颗

        // 生成所有4颗卫星组合(简化实现：仅遍历前10选4，实际可优化)
        std::vector<int> indices(params.satellite_pos.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::vector<bool> mask(indices.size(), false);
        std::fill(mask.end() - 4, mask.end(), true);

        do {
            std::vector<int> combo;
            for (int i = 0; i < indices.size(); ++i) {
                if (mask[i]) combo.push_back(indices[i]);
            }
            if (combo.size() != 4) continue;

            // 构建几何矩阵G
            Eigen::MatrixXd G(4, 4);
            bool valid = true;
            for (int i = 0; i < 4; ++i) {
                ECEF sat_ecef = sat_ecef_list[combo[i]];
                double dx = sat_ecef.X - target_ecef.X;
                double dy = sat_ecef.Y - target_ecef.Y;
                double dz = sat_ecef.Z - target_ecef.Z;
                double dist = sqrt(dx * dx + dy * dy + dz * dz);

                if (dist < 1e-6 || isnan(dist)) {
                    valid = false;
                    break;
                }

                G(i, 0) = dx / dist;
                G(i, 1) = dy / dist;
                G(i, 2) = dz / dist;
                G(i, 3) = 1.0;
            }

            if (!valid) continue;

            // 计算GDOP
            Eigen::MatrixXd A = G.transpose() * G;
            //if (A.conditionNumber() > 1e12) continue;  // 条件数过大

            try {
                Eigen::MatrixXd A_inv = A.inverse();
                double gdop = sqrt(A_inv.trace());
                if (gdop < min_gdop) {
                    min_gdop = gdop;
                    best_idx = combo;
                }
            }
            catch (...) {
                continue;
            }
        } while (std::next_permutation(mask.begin(), mask.end()));

        // 提取最优卫星
        std::vector<LLA> optimal_sats;
        for (int idx : best_idx) {
            optimal_sats.push_back(params.satellite_pos[idx]);
        }
        return optimal_sats;
    }

    // 计算信号相对功率
    double GNSSDeceptionError::calc_signal_power(const LLA& trans_pos, const LLA& recv_pos) const {
        ECEF trans_ecef = lla_to_ecef(trans_pos);
        ECEF recv_ecef = lla_to_ecef(recv_pos);
        double dist = calc_ecef_distance(trans_ecef, recv_ecef);

        if (dist < 1e-6) return 0.0;

        // 自由空间损耗公式
        double loss = pow(4 * M_PI * dist * FREQ / C_LIGHT, 2);
        return 1.0 / loss;
    }

    // 判断欺骗信号是否有效
    bool GNSSDeceptionError::is_deception_valid(const LLA& target_pos, const std::vector<LLA>& satellite_pos) const {
        // 计算真实卫星平均功率
        double real_power_sum = 0.0;
        for (const auto& sat : satellite_pos) {
            real_power_sum += calc_signal_power(sat, target_pos);
        }
        double real_power = real_power_sum / satellite_pos.size();

        // 计算欺骗信号平均功率
        double deception_power_sum = 0.0;
        for (const auto& jammer : params.jammer_pos) {
            deception_power_sum += calc_signal_power(jammer, target_pos);
        }
        double deception_power = deception_power_sum / params.jammer_pos.size();

        if (real_power < 1e-12) return false;

        // 功率比转dB
        double power_ratio_dB = 10 * log10(deception_power / real_power);
        return power_ratio_dB > params.power_ratio_threshold;
    }

    // 计算欺骗时延
    std::vector<double> GNSSDeceptionError::calculate_deception_delay(const LLA& target_pos,
        const std::vector<LLA>& satellite_pos) const {
        ECEF deception_ecef = lla_to_ecef(params.deception_pos);
        ECEF target_ecef = lla_to_ecef(target_pos);
        std::vector<double> delays;

        for (int i = 0; i < params.jammer_num; ++i) {
            ECEF jammer_ecef = lla_to_ecef(params.jammer_pos[i]);
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);

            double R_SD = calc_ecef_distance(sat_ecef, deception_ecef);
            double R_SJ = calc_ecef_distance(sat_ecef, jammer_ecef);
            double R_JR = calc_ecef_distance(jammer_ecef, target_ecef);

            double tau = (R_SD - R_SJ - R_JR) / C_LIGHT;
            delays.push_back(tau);
        }

        // 时延修正：确保非负
        double min_tau = *std::min_element(delays.begin(), delays.end());
        if (min_tau < 0) {
            for (auto& tau : delays) {
                tau += fabs(min_tau);
            }
        }

        return delays;
    }

    // 求解定位误差
    LLA GNSSDeceptionError::solve_position_error(const std::vector<LLA>& satellite_pos,
        const LLA& target_pos,
        const std::vector<double>& delays) const {
        ECEF target_ecef = lla_to_ecef(target_pos);
        int n = params.jammer_num;

        // 构建观测矩阵M
        Eigen::MatrixXd M(n, 4);
        std::vector<double> R_SR_list;

        for (int i = 0; i < n; ++i) {
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);
            double dx = sat_ecef.X - target_ecef.X;
            double dy = sat_ecef.Y - target_ecef.Y;
            double dz = sat_ecef.Z - target_ecef.Z;
            double R_SR = sqrt(dx * dx + dy * dy + dz * dz);

            if (R_SR < 1e-6 || isnan(R_SR)) {
                return { 0.0, 0.0, 0.0 };
            }

            R_SR_list.push_back(R_SR);
            double unit_x = dx / R_SR;
            double unit_y = dy / R_SR;
            double unit_z = dz / R_SR;

            M(i, 0) = unit_x / C_LIGHT;
            M(i, 1) = unit_y / C_LIGHT;
            M(i, 2) = unit_z / C_LIGHT;
            M(i, 3) = 1.0;
        }

        // 构建时延向量T
        Eigen::VectorXd T(n);
        for (int i = 0; i < n; ++i) {
            T(i) = delays[i];
        }

        // 构建修正向量A
        Eigen::VectorXd A(n);
        for (int i = 0; i < n; ++i) {
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);
            ECEF jammer_ecef = lla_to_ecef(params.jammer_pos[i]);

            double R_SJ = calc_ecef_distance(sat_ecef, jammer_ecef);
            double R_JR = calc_ecef_distance(jammer_ecef, target_ecef);
            double R_SR = R_SR_list[i];

            A(i) = (R_SJ + R_JR - R_SR) / C_LIGHT;
        }

        // 最小二乘求解
        try {
            Eigen::MatrixXd MtM = M.transpose() * M;
            /*if (MtM.conditionNumber() > 1e12) {
                return { 0.0, 0.0, 0.0 };
            }*/

            Eigen::MatrixXd MtM_inv = MtM.inverse();
            Eigen::VectorXd delta_X = MtM_inv * M.transpose() * (T + A);

            // 提取位置误差(米)
            ECEF error_ecef;
            error_ecef.X = delta_X(0);
            error_ecef.Y = delta_X(1);
            error_ecef.Z = delta_X(2);

            // 计算受干扰后的位置
            ECEF new_target_ecef;
            new_target_ecef.X = target_ecef.X + error_ecef.X;
            new_target_ecef.Y = target_ecef.Y + error_ecef.Y;
            new_target_ecef.Z = target_ecef.Z + error_ecef.Z;

            LLA new_target_lla = ecef_to_lla(new_target_ecef);

            // 计算误差
            LLA pos_error;
            pos_error.lon_deg = new_target_lla.lon_deg - target_pos.lon_deg;
            pos_error.lat_deg = new_target_lla.lat_deg - target_pos.lat_deg;
            pos_error.h_km = new_target_lla.h_km - target_pos.h_km;

            return pos_error;
        }
        catch (...) {
            return { 0.0, 0.0, 0.0 };
        }
    }

    // 设置自定义参数
    void GNSSDeceptionError::set_params(const SimParams& custom_params) {
        params = custom_params;
    }

    // 核心计算接口：输入航迹点，输出误差结果
    TrackResult GNSSDeceptionError::calculate_error(const LLA& target_pos) {
        TrackResult result;
        result.target_pos = target_pos;
        result.deception_valid = false;

        // 1. 筛选最优卫星
        std::vector<LLA> optimal_sats = select_optimal_satellites(target_pos);

        // 2. 判定欺骗有效性
        if (!is_deception_valid(target_pos, optimal_sats)) {
            result.error_pos = target_pos;
            result.pos_error = { 0.0, 0.0, 0.0 };
            return result;
        }
        result.deception_valid = true;

        // 3. 计算欺骗时延
        std::vector<double> delays = calculate_deception_delay(target_pos, optimal_sats);

        // 4. 求解定位误差
        result.pos_error = solve_position_error(optimal_sats, target_pos, delays);

        // 5. 计算受干扰后的位置
        result.error_pos.lon_deg = target_pos.lon_deg - result.pos_error.lon_deg;
        result.error_pos.lat_deg = target_pos.lat_deg - result.pos_error.lat_deg;
        result.error_pos.h_km = target_pos.h_km - result.pos_error.h_km;

        return result;
    }

    // 批量处理航迹数据
    std::vector<TrackResult> GNSSDeceptionError::batch_calculate(const std::vector<LLA>& track_points) {
        std::vector<TrackResult> results;
        for (const auto& point : track_points) {
            results.push_back(calculate_error(point));
        }
        return results;
    }


    // 示例使用
    int Deception_Test(Json::Value input, Json::Value& trajectory_result) {
        
        // 1. 创建计算实例
        GNSSDeceptionError gnss_error;

        // 2. (可选)自定义参数
        SimParams custom_params;
        custom_params.deception_pos = { 115.35, 29.18, 0.5 };  // 修改欺骗点
        gnss_error.set_params(custom_params);

        // 3. 输入航迹数据
        std::vector<LLA> track_points = {
            {115.193, 29.027, 0.5},
            {115.194, 29.028, 0.5},
            {115.195, 29.029, 0.5}
        };

        // 4. 批量计算
        std::vector<TrackResult> results = gnss_error.batch_calculate(track_points);

        // 5. 输出结果
        /*for (size_t i = 0; i < results.size(); ++i) {
            const auto& res = results[i];
            std::cout << "航迹点 " << i + 1 << ":" << std::endl;
            std::cout << "  目标位置: 经度=" << res.target_pos.lon_deg
                << "°, 纬度=" << res.target_pos.lat_deg
                << "°, 高度=" << res.target_pos.h_km << "km" << std::endl;
            std::cout << "  欺骗有效: " << (res.deception_valid ? "是" : "否") << std::endl;
            std::cout << "  定位误差: 经度=" << res.pos_error.lon_deg
                << "°, 纬度=" << res.pos_error.lat_deg
                << "°, 高度=" << res.pos_error.h_km << "km" << std::endl;
            std::cout << "-------------------------" << std::endl;
        }*/

        return 0;
    }

}