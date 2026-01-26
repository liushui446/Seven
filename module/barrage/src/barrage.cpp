#include "barrage/barrage.hpp"

#include <cmath>
#include <Eigen/Dense>  // 需Eigen库处理矩阵运算（GDOP计算）
#include <numeric>      // 数值计算



namespace seven {

    // ========================= 2. 核心算法类封装 =========================
    // 经纬度转ECEF
    ECEF GNSSJammerSim::lla_to_ecef(const LLA& lla) {
        ECEF ecef;
        double lat_deg_rad = lla.lat_deg * M_PI / 180.0;
        double lon_deg_rad = lla.lon_deg * M_PI / 180.0;
        double h_m = lla.h_km * 1000.0; // km -> m

        double N = a / sqrt(1 - e2 * pow(sin(lat_deg_rad), 2));
        ecef.X = (N + h_m) * cos(lat_deg_rad) * cos(lon_deg_rad);
        ecef.Y = (N + h_m) * cos(lat_deg_rad) * sin(lon_deg_rad);
        ecef.Z = ((1 - e2) * N + h_m) * sin(lat_deg_rad);
        return ecef;
    }

    // ECEF转经纬度
    LLA GNSSJammerSim::ecef_to_lla(const ECEF& ecef) {
        LLA lla;
        double p = sqrt(ecef.X * ecef.X + ecef.Y * ecef.Y);

        // 处理极区情况
        if (p < 1e-12) {
            lla.lon_deg = 0.0;
            lla.lat_deg = (ecef.Z >= 0) ? 90.0 : -90.0;
            lla.h_km = (fabs(ecef.Z) - b) / 1000.0; // m -> km
            return lla;
        }

        double theta = atan2(ecef.Z * a, p * b);
        double lon_deg_rad = atan2(ecef.Y, ecef.X);
        double lat_deg_rad = atan2(
            ecef.Z + ep2 * b * pow(sin(theta), 3),
            p - e2 * a * pow(cos(theta), 3)
        );
        double N = a / sqrt(1 - e2 * pow(sin(lat_deg_rad), 2));
        lla.lon_deg = lon_deg_rad * 180.0 / M_PI;
        lla.lat_deg = lat_deg_rad * 180.0 / M_PI;
        lla.h_km = (p / cos(lat_deg_rad) - N) / 1000.0; // m -> km
        return lla;
    }

    // 功率谱密度计算
    void GNSSJammerSim::calc_power_spectral_density(double f, const SimConfig& config, int jammer_idx, double& GJ, double& GS) {
        // 卫星信号功率谱密度GS(f)
        if (config.pseudocode == "C/A" || config.pseudocode == "P(Y)") {
            GS = config.Tc * pow(sinc(M_PI * f * config.Tc), 2);
        }
        else if (config.pseudocode == "M") {
            GS = config.Tc * pow(sinc(M_PI * f * config.Tc / 2), 2) * pow(tan(M_PI * f / 2 * config.fs), 2);
        }
        else {
            GS = 0.0;
        }

        // 干扰信号功率谱密度GJ(f)
        const JammerParam& jammer = config.jammers[jammer_idx];
        double jam_freq = jammer.freq;
        GJ = 0.0;

        if (jammer.type == "continuous_wave") {
            GJ = 1.0;
        }
        else if (jammer.type == "multi-tone") {
            // 多音干扰简化处理：匹配中心频率±1e3Hz
            if (fabs(f - jam_freq) < 1e3) {
                GJ = 1.0;
            }
        }
        else if (jammer.type == "bandlimited_gaussian") {
            if (fabs(f - jam_freq) < jammer.bandwidth / 2) {
                GJ = 1.0 / jammer.bandwidth;
            }
        }
        else if (jammer.type == "pseudocode") {
            if (fabs(f - jam_freq) < jammer.bandwidth / 2) {
                GJ = 1.0;
            }
        }
        else if (jammer.type == "pulse") {
            double tau = jammer.pulse_width;
            double T = jammer.pulse_period;
            double sa_term = sinc((f - jam_freq) * tau);
            if (fabs(f - jam_freq) < jammer.bandwidth / 2) {
                GJ = (tau / T) * pow(sa_term, 2);
            }
        }
    }

    // 数值积分（梯形法近似，替代scipy.integrate.quad）
    double GNSSJammerSim::integrate_quad(std::function<double(double)> func, double start, double end, int steps) {
        double h_km = (end - start) / steps;
        double sum = 0.5 * (func(start) + func(end));
        for (int i = 1; i < steps; ++i) {
            sum += func(start + i * h_km);
        }
        return sum * h_km;
    }

    // sinc函数实现
    double GNSSJammerSim::sinc(double x) {
        if (fabs(x) < 1e-12) return 1.0;
        return sin(x) / x;
    }

    // APM电磁传播模型计算干扰接收功率
    double GNSSJammerSim::calc_jam_power_apm(const JammerParam& jammer, const LLA& target_pos, const SimConfig& config) {
        // 转换为ECEF计算距离
        ECEF jam_ecef = lla_to_ecef(jammer.pos);
        ECEF target_ecef = lla_to_ecef(target_pos);
        double dist = sqrt(
            pow(jam_ecef.X - target_ecef.X, 2) +
            pow(jam_ecef.Y - target_ecef.Y, 2) +
            pow(jam_ecef.Z - target_ecef.Z, 2)
        ); // 总距离(米)

        // 垂直/水平距离
        double dist_vertical = fabs((jammer.pos.h_km - target_pos.h_km) * 1000);
        double dist_horizontal = sqrt(pow(dist, 2) - pow(dist_vertical, 2));

        // 天线仰角(°)
        double antenna_elevation = atan2(dist_vertical, dist_horizontal) * 180.0 / M_PI;

        // 路径损耗计算
        double loss = 0.0;
        if (antenna_elevation > 5 || dist < 5000) {
            // FE模型（自由空间损耗）
            loss = pow(4 * M_PI * dist * config.fc / config.c, 2);
        }
        else if (dist < 20000) {
            // RO模型（射线光学）
            double refraction_factor = 1.0003;
            loss = pow(4 * M_PI * dist * config.fc * refraction_factor / config.c, 2);
        }
        else if (target_pos.h_km * 1000 < 10000) {
            // PE模型（抛物方程）
            loss = pow(4 * M_PI * dist * config.fc / config.c, 2) * 1.2;
        }
        else {
            // XO模型（扩展光学）
            loss = pow(4 * M_PI * dist * config.fc / config.c, 2) * 0.8;
        }

        // 干扰接收功率(W)
        double jam_power_w = jammer.power;
        // 若输入为dBm，转换为W（可选逻辑）
        if (jam_power_w < 100) {
            jam_power_w = pow(10, jam_power_w / 10) / 1000;
        }
        double Pj = jam_power_w * 1.0 / loss; // 天线增益简化为1

        return Pj;
    }

    // 载噪比计算
    double GNSSJammerSim::calc_cnr(double Pj, const SimConfig& config, int jammer_idx) {
        // 积分计算∫GJ(f)GS(f)df（积分范围：fc - beta/2 到 fc + beta/2）
        auto integrand = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, config, jammer_idx, GJ, GS);
            return GJ * GS;
            };

        double integral_result = integrate_quad(
            integrand,
            config.fc - config.beta / 2,
            config.fc + config.beta / 2,
            1000
        );
        double integral_total = Pj * integral_result;

        // 载噪比计算（线性值转dB-Hz）
        double C_NJ_linear = config.satellite.carrier_power / integral_total;
        double C_NJ_dB = (C_NJ_linear > 0) ? 10 * log10(C_NJ_linear) : -100;

        return C_NJ_dB;
    }

    // 干信比计算
    double GNSSJammerSim::calc_jammer_to_signal_ratio(double Pj, const LLA& target_pos, const SimConfig& config) {
        // 计算卫星信号总接收功率Ps(W)
        double Ps_total = 0.0;
        for (const auto& sat_pos : config.satellite.sat_pos) {
            // 星地距离计算
            ECEF sat_ecef = lla_to_ecef(sat_pos);
            ECEF target_ecef = lla_to_ecef(target_pos);
            double dist_sat_target = sqrt(
                pow(sat_ecef.X - target_ecef.X, 2) +
                pow(sat_ecef.Y - target_ecef.Y, 2) +
                pow(sat_ecef.Z - target_ecef.Z, 2)
            );

            // 自由空间损耗
            double loss_sat = pow(4 * M_PI * dist_sat_target * config.fc / config.c, 2);
            // GPS卫星典型EIRP：48.5 dBW = 70794.58 W
            double sat_eirp = 70794.58;
            double Ps_sat = sat_eirp * 100 / loss_sat;
            Ps_total += Ps_sat;
        }

        // 干信比(dB)
        if (Ps_total < 1e-20) {
            return 200.0; // 避免除零，设为极大值
        }
        double J_S = Pj / Ps_total;
        return 10 * log10(J_S);
    }

    // 跟踪环误差计算
    void GNSSJammerSim::calc_tracking_errors(double C_NJ_dB, double J_S_dB, const SimConfig& config, int jammer_idx, double& sigma_jpll, double& sigma_jdll) {
        double J_S = pow(10, J_S_dB / 10); // 转线性值
        double C_NJ_linear = pow(10, C_NJ_dB / 10);

        // 1. 载波环振荡器颤动σ_JPLL(°)
        double term = (config.Bp / C_NJ_linear) * (1 + 1 / (2 * config.Td * C_NJ_linear));
        sigma_jpll = (360.0 / (2 * M_PI)) * sqrt(term);
        sigma_jpll = fmod(sigma_jpll, 360.0); // 限制在0-360°

        // 2. 码环跟踪误差σ_JDLL
        // 积分项1：∫GJ(f)GS(f)·sin?(πf d Tc) df
        auto integrand_sin2 = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, config, jammer_idx, GJ, GS);
            double sin_term = sin(M_PI * f * config.d * config.Tc);
            return GJ * GS * pow(sin_term, 2);
            };
        double integral_sin2 = integrate_quad(
            integrand_sin2,
            config.fc - config.beta / 2,
            config.fc + config.beta / 2
        );

        // 积分项2：∫f·GJ(f)GS(f)·sin(πf d Tc) df
        auto integrand_fsin = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, config, jammer_idx, GJ, GS);
            return f * GJ * GS * sin(M_PI * f * config.d * config.Tc);
            };
        double integral_fsin = integrate_quad(
            integrand_fsin,
            config.fc - config.beta / 2,
            config.fc + config.beta / 2
        );

        // 积分项3：∫GJ(f)GS(f)·cos?(πf d Tc) df
        auto integrand_cos2 = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, config, jammer_idx, GJ, GS);
            double cos_term = cos(M_PI * f * config.d * config.Tc);
            return GJ * GS * pow(cos_term, 2);
            };
        double integral_cos2 = integrate_quad(
            integrand_cos2,
            config.fc - config.beta / 2,
            config.fc + config.beta / 2
        );

        // 积分项4：∫GJ(f)GS(f)·cos(πf d Tc) df
        auto integrand_cos = [&](double f) {
            double GJ, GS;
            calc_power_spectral_density(f, config, jammer_idx, GJ, GS);
            return GJ * GS * cos(M_PI * f * config.d * config.Tc);
            };
        double integral_cos = integrate_quad(
            integrand_cos,
            config.fc - config.beta / 2,
            config.fc + config.beta / 2
        );

        // 码环误差计算
        sigma_jdll = 0.0;
        if (integral_fsin != 0) {
            double numerator = sqrt(config.Bd * J_S * integral_sin2);
            double denominator = 2 * M_PI * integral_fsin;
            double term2 = (J_S * integral_cos2) / (config.Td * pow(integral_cos, 2));
            sigma_jdll = (numerator / denominator) * sqrt(1 + term2);
        }
    }

    // GDOP计算
    double GNSSJammerSim::calc_gdop(const LLA& target_pos, const std::vector<LLA>& satellite_pos) {
        // 构建几何矩阵G
        Eigen::MatrixXd G(satellite_pos.size(), 4);
        for (int i = 0; i < satellite_pos.size(); ++i) {
            ECEF sat_ecef = lla_to_ecef(satellite_pos[i]);
            ECEF target_ecef = lla_to_ecef(target_pos);
            Eigen::Vector3d delta(
                sat_ecef.X - target_ecef.X,
                sat_ecef.Y - target_ecef.Y,
                sat_ecef.Z - target_ecef.Z
            );
            double dist = delta.norm();
            if (dist < 1e-6) dist = 1e-6;
            Eigen::Vector3d unit_vec = delta / dist;
            G(i, 0) = unit_vec.x();
            G(i, 1) = unit_vec.y();
            G(i, 2) = unit_vec.z();
            G(i, 3) = 1.0;
        }

        // 计算GDOP = sqrt(trace((G^T G)^-1))
        double gdop = 10.0; // 默认最差值
        try {
            Eigen::MatrixXd GtG = G.transpose() * G;
            Eigen::MatrixXd GtG_inv = GtG.inverse();
            gdop = sqrt(GtG_inv.trace());
        }
        catch (...) {
            // 矩阵不可逆时返回默认值
        }
        return gdop;
    }

    // 定位误差计算
    void GNSSJammerSim::calc_pos_error(bool unlock_flag, double sigma_jdll, double gdop, const SimConfig& config, LLA& pos_error, ECEF& pos_error_m) {
        if (unlock_flag) {
            // 失锁：惯导漂移误差
            pos_error.lon_deg = config.ins_drift / 111;  // 1°≈111km，转换为度
            pos_error.lat_deg = config.ins_drift / 111;
            pos_error.h_km = 0.1;                       // 高度误差(km)

            pos_error_m.X = config.ins_drift * 1000; // 米
            pos_error_m.Y = config.ins_drift * 1000;
            pos_error_m.Z = config.ins_drift * 1000;
        }
        else {
            // 未失锁：伪距误差×GDOP
            double pseudo_range_error = sigma_jdll * config.Tc * config.c; // 伪距误差(米)
            double error_m = pseudo_range_error * gdop;

            // 转换为经纬高误差（度/km）
            pos_error.lon_deg = error_m / (1000 * 111); // 米→度
            pos_error.lat_deg = error_m / (1000 * 111);
            pos_error.h_km = error_m / 1000;           // 米→km

            pos_error_m.X = error_m;
            pos_error_m.Y = error_m;
            pos_error_m.Z = error_m;
        }
    }

    BarrageTrackResult GNSSJammerSim::calc_track_result(const LLA& target_pos, const SimConfig& config) {
        BarrageTrackResult result;
        result.C_NJ_dB = 0.0;
        result.J_S_dB = 0.0;
        result.sigma_jpll = 0.0;
        result.sigma_jdll = 0.0;
        result.unlock_flag = false;
        result.pos_error = { 0.0, 0.0, 0.0 };
        result.pos_error_m = { 0.0, 0.0, 0.0 };
        result.gdop = calc_gdop(target_pos, config.satellite.sat_pos);

        int jammer_count = config.jammers.size();
        if (jammer_count == 0) {
            return result; // 无干扰源，返回默认值
        }

        // 累加所有干扰源的影响
        double sum_C_NJ = 0.0;
        double sum_J_S = 0.0;
        double sum_sigma_jpll = 0.0;
        double sum_sigma_jdll = 0.0;
        int unlock_num = 0;

        for (int i = 0; i < jammer_count; ++i) {
            // 1. 计算干扰接收功率
            double Pj = calc_jam_power_apm(config.jammers[i], target_pos, config);

            // 2. 计算载噪比
            double C_NJ = calc_cnr(Pj, config, i);
            sum_C_NJ += C_NJ;

            // 3. 计算干信比
            double J_S = calc_jammer_to_signal_ratio(Pj, target_pos, config);
            sum_J_S += J_S;

            // 4. 计算跟踪环误差
            double sigma_jpll, sigma_jdll;
            calc_tracking_errors(C_NJ, J_S, config, i, sigma_jpll, sigma_jdll);
            sum_sigma_jpll += sigma_jpll;
            sum_sigma_jdll += sigma_jdll;

            // 5. 判定失锁
            bool pll_unlock = (sigma_jpll > config.pll_unlock_thresh);
            bool dll_unlock = (sigma_jdll > config.dll_unlock_thresh);
            if (pll_unlock || dll_unlock) {
                unlock_num++;
            }
        }

        // 平均所有干扰源的结果
        result.C_NJ_dB = sum_C_NJ / jammer_count;
        result.J_S_dB = sum_J_S / jammer_count;
        result.sigma_jpll = sum_sigma_jpll / jammer_count;
        result.sigma_jdll = sum_sigma_jdll / jammer_count;
        result.unlock_flag = (unlock_num > 0); // 任意干扰源导致失锁则整体失锁

        // 计算定位误差
        calc_pos_error(
            result.unlock_flag,
            result.sigma_jdll,
            result.gdop,
            config,
            result.pos_error,
            result.pos_error_m
        );

        return result;
    }

    // 批量处理航迹数据接口
    std::vector<BarrageTrackResult> GNSSJammerSim::batch_calc(const std::vector<LLA>& track_points, const SimConfig& config) {
        std::vector<BarrageTrackResult> results;
        for (const auto& point : track_points) {
            results.push_back(calc_track_result(point, config));
        }
        return results;
    }

    // ========================= 3. 使用示例 =========================
    int Barrage_Test(Json::Value input, Json::Value& trajectory_result) {
        // 1. 初始化配置
        SimConfig config;

        // 配置卫星参数
        config.satellite.carrier_power = 1e-16;
        config.satellite.sat_pos = {
            {125.0, 30.0, 5000},
            {115.0, 35.0, 5000},
            {130.0, 25.0, 5000},
            {110.0, 28.0, 5000}
        };

        // 配置干扰源
        JammerParam jammer1;
        jammer1.pos = { 120.25, 28.0, 8.3 };
        jammer1.power = 10.0; // W
        jammer1.type = "continuous_wave";
        jammer1.bandwidth = 20e6;
        jammer1.freq = GNSS_FC;
        config.jammers.push_back(jammer1);

        JammerParam jammer2;
        jammer2.pos = { 119.75, 27.64, 8.3 };
        jammer2.power = 10.0;
        jammer2.type = "continuous_wave";
        jammer2.bandwidth = 1e6;
        jammer2.freq = GNSS_FC;
        config.jammers.push_back(jammer2);

        // 2. 待处理的航迹数据
        std::vector<LLA> track_points = {
            {119.045, 27.2233, 1.3},   // 航迹点1
            {119.050, 27.2263, 1.3},   // 航迹点2
            {119.055, 27.2293, 1.3}    // 航迹点3
        };

        // 3. 初始化仿真类并计算
        GNSSJammerSim sim;
        std::vector<BarrageTrackResult> results = sim.batch_calc(track_points, config);

        // 4. 输出结果
        /*for (int i = 0; i < results.size(); ++i) {
            std::cout << "===== 航迹点 " << i + 1 << " 结果 =====" << std::endl;
            std::cout << "载噪比(dB-Hz): " << results[i].C_NJ_dB << std::endl;
            std::cout << "干信比(dB): " << results[i].J_S_dB << std::endl;
            std::cout << "载波环误差(°): " << results[i].sigma_jpll << std::endl;
            std::cout << "码环误差: " << results[i].sigma_jdll << std::endl;
            std::cout << "失锁标志: " << (results[i].unlock_flag ? "是" : "否") << std::endl;
            std::cout << "定位误差(经/纬/高): "
                << results[i].pos_error.lon_deg << "° / "
                << results[i].pos_error.lat_deg << "° / "
                << results[i].pos_error.h_km << "km" << std::endl;
            std::cout << "定位误差(米, X/Y/Z): "
                << results[i].pos_error_m.X << "m / "
                << results[i].pos_error_m.Y << "m / "
                << results[i].pos_error_m.Z << "m" << std::endl;
            std::cout << "GDOP: " << results[i].gdop << std::endl;
            std::cout << std::endl;
        }*/

        return 0;
    }

}