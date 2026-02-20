#include "transformation/transformation.hpp"

using namespace Json;

namespace seven {

    string formationToStr(Formation_Type type)
    {
        string formation_str = "";
        switch (type)
        {
        case Formation_Type::Circle:
            formation_str = "Circle";
            break;
        case Formation_Type::Diamond:
            formation_str = "Diamond";
            break;
        case Formation_Type::Line:
            formation_str = "Line";
            break;
        case Formation_Type::Rectangle:
            formation_str = "Rectangle";
            break;
        case Formation_Type::Triangle:
            formation_str = "Triangle";
            break;
        default:
            break;
        }
        return formation_str;
    }

    /**
         * @brief 添加单帧轨迹数据
         */
    void UAVTrajectory::addFrame(int frame, double time, int uav_id, const Formation_Type formation, const Point2D& pos) {
        trajectory_data.push_back({ frame, time, uav_id, formation, pos });
    }

    /**
     * @brief 记录队形变换帧数
     */
    void UAVTrajectory::addFormationChangeFrame(int frame) {
        formation_change_frames.push_back(frame);
    }

    /**
     * @brief 获取指定UAV的完整轨迹
     * @param uav_id 无人机ID
     * @return 该无人机的所有轨迹帧
     */
    std::vector<TrajectoryFrame> UAVTrajectory::getUAVTrajectory(int uav_id) const {
        std::vector<TrajectoryFrame> res;
        for (const auto& frame : trajectory_data) {
            if (frame.uav_id == uav_id) {
                res.push_back(frame);
            }
        }
        return res;
    }

    /**
     * @brief 获取所有轨迹数据
     */
    const std::vector<TrajectoryFrame>& UAVTrajectory::getAllTrajectory() const {
        return trajectory_data;
    }

    /**
     * @brief 获取队形变换帧数记录
     */
    const std::vector<int>& UAVTrajectory::getFormationChangeFrames() const {
        return formation_change_frames;
    }

    /**
         * @brief 生成指定类型的队形位置
         */
    std::vector<Point2D> UAVFormationTransformer::generateFormation(const Formation_Type& formation_type) {
        std::vector<Point2D> positions(params_.num_uavs);
        Point2D center(0, 0);  // 队形中心固定在原点

        if (formation_type == Formation_Type::Triangle) {
        //    // 等腰三角形队形（底边严格水平）
        //    double base_ratio = 1.6;
        //    double height_ratio = 1;
        //    double base_length = params_.interval * base_ratio * std::ceil(std::sqrt(params_.num_uavs));
        //    double height = params_.interval * height_ratio * std::ceil(std::sqrt(params_.num_uavs));
        //    double base_y = -2 * height / 3;

        //    // 三角形顶点
        //    Point2D vertex_top(0, height / 3);
        //    Point2D vertex_left(-base_length / 2, base_y);
        //    Point2D vertex_right(base_length / 2, base_y);
        //    std::vector<Point2D> vertices = { vertex_top, vertex_left, vertex_right };

        //    // 计算边长
        //    double side1_len = (vertex_left - vertex_top).norm();
        //    double side2_len = (vertex_right - vertex_left).norm();
        //    double side3_len = (vertex_top - vertex_right).norm();
        //    double total_len = side1_len + side2_len + side3_len;

        //    // 按边长比例分配无人机
        //    int uav_side1 = static_cast<int>(std::ceil(params_.num_uavs * side1_len / total_len));
        //    int uav_side2 = static_cast<int>(std::ceil(params_.num_uavs * side2_len / total_len));
        //    int uav_side3 = params_.num_uavs - uav_side1 - uav_side2;
        //    std::vector<int> uavs_per_side = { uav_side1, uav_side2, uav_side3 };

        //    int idx = 0;
        //    for (int side = 0; side < 3; ++side) {
        //        Point2D start = vertices[side];
        //        Point2D end = vertices[(side + 1) % 3];
        //        int num_uav = uavs_per_side[side];

        //        if (num_uav <= 0 || idx >= params_.num_uavs) continue;

        //        for (int i = 0; i < num_uav; ++i) {
        //            if (idx >= params_.num_uavs) break;

        //            double t = (num_uav > 1) ? static_cast<double>(i) / (num_uav - 1) : 0.0;
        //            double x = start.x * (1 - t) + end.x * t;
        //            double y = start.y * (1 - t) + end.y * t;

        //            // 底边强制统一y坐标
        //            if (side == 1) {
        //                y = base_y;
        //            }

        //            positions[idx++] = Point2D(x, y) + center;
        //        }
        //    }

            positions.resize(0);
            // ------------- 核心优化1：参数适配（保证间距=interval） -------------
            int total_vertex = 3; // 三角形3个顶点
            int edge_uavs_total = std::max(0, params_.num_uavs - total_vertex); // 边的无人机总数（扣除顶点）
            int uav_per_side_base = edge_uavs_total / 3;                        // 每条边基础数量
            int remaining_uavs = edge_uavs_total % 3;                           // 剩余未分配数量

            // 计算三角形尺寸：保证边的无人机间隔=params_.interval
            // 底边长度 = 间隔 × (底边无人机数 + 1)
            int base_edge_uavs = uav_per_side_base + (remaining_uavs > 1 ? 1 : 0); // 底边多分配（如果有剩余）
            double base_length = params_.interval * (base_edge_uavs + 1);
            // 等腰三角形高：h = √(腰长² - (底/2)²)，保证腰长=间隔×(腰边无人机数+1)
            int waist_edge_uavs = uav_per_side_base + (remaining_uavs > 0 ? 1 : 0);
            double waist_length = params_.interval * (waist_edge_uavs + 1);
            double height = std::sqrt(waist_length * waist_length - (base_length / 2) * (base_length / 2));

            // ------------- 步骤1：定义三角形顶点（底边严格水平） -------------
            // 顶点坐标：以center为中心，底边水平，顶点在上
            double base_y = center.y - height * 2.0 / 3.0; // 底边y坐标（固定）
            Point2D vertex_top(center.x, center.y + height / 3.0);    // 上顶点
            Point2D vertex_left(center.x - base_length / 2, base_y);     // 左底角
            Point2D vertex_right(center.x + base_length / 2, base_y);    // 右底角
            std::vector<Point2D> vertices = { vertex_top, vertex_left, vertex_right };

            // ------------- 步骤2：分配顶点无人机（先占3个位置） -------------
            int pos_idx = 0;
            int vertex_count = std::min(total_vertex, params_.num_uavs);
            for (int i = 0; i < vertex_count; ++i) {
                positions.push_back(vertices[i]);
                pos_idx++;
            }
            if (pos_idx >= params_.num_uavs)
            {
                positions.clear();
                return positions; // 无人机数量≤3时直接返回
            }

            // ------------- 步骤3：定义三角形的3条边（顶点两两连接） -------------
            // 边0：上顶点→左底角（左腰）
            // 边1：左底角→右底角（底边，水平）
            // 边2：右底角→上顶点（右腰）
            std::vector<std::pair<Point2D, Point2D>> edges = {
                {vertices[0], vertices[1]},
                {vertices[1], vertices[2]},
                {vertices[2], vertices[0]}
            };

            // ------------- 步骤4：为每条边分配无人机（核心优化：单层、无重叠） -------------
            // 每条边的无人机数量（扣除顶点）
            std::vector<int> uavs_per_side = {
                waist_edge_uavs,   // 左腰
                base_edge_uavs,    // 底边
                waist_edge_uavs    // 右腰
            };
            // 调整数量，保证总和=edge_uavs_total
            int sum_edge = uavs_per_side[0] + uavs_per_side[1] + uavs_per_side[2];
            if (sum_edge > edge_uavs_total) {
                // 超出时，优先减少底边数量
                uavs_per_side[1] -= (sum_edge - edge_uavs_total);
            }
            else if (sum_edge < edge_uavs_total) {
                // 不足时，优先增加底边数量
                uavs_per_side[1] += (edge_uavs_total - sum_edge);
            }

            for (int side = 0; side < 3; ++side) {
                if (pos_idx >= params_.num_uavs) break;

                auto& edge = edges[side];
                int num_uav = uavs_per_side[side];
                if (num_uav <= 0) continue;

                // ------------- 核心优化2：插值步长（避免重叠/分层） -------------
                // t范围：1/(num_uav+1) ~ num_uav/(num_uav+1)
                // 保证边的无人机在两个顶点之间，且不与顶点重叠
                for (int i = 1; i <= num_uav; ++i) {
                    if (pos_idx >= params_.num_uavs) break;

                    double t = static_cast<double>(i) / (num_uav + 1);
                    double x = edge.first.x * (1 - t) + edge.second.x * t;
                    double y = edge.first.y * (1 - t) + edge.second.y * t;

                    // 强制底边水平（避免插值误差导致y坐标偏移）
                    if (side == 1) {
                        y = base_y;
                    }

                    positions.push_back(Point2D(x, y));
                    pos_idx++;
                }
            }

            // ------------- 兜底：极端情况（数量不匹配） -------------
            while (pos_idx < params_.num_uavs) {
                positions.push_back(center); // 剩余无人机放在中心（可选）
                pos_idx++;
            }
        }
        else if (formation_type == Formation_Type::Circle) {
            // 圆形队形
            /*for (int i = 0; i < params_.num_uavs; ++i) {
                double angle = 2 * M_PI * i / params_.num_uavs - M_PI / 6;
                double radius = params_.interval * std::sqrt(params_.num_uavs) / 2;
                positions[i] = center + Point2D(std::cos(angle) * radius, std::sin(angle) * radius);
            }*/

            // 方案1：无重叠的最小半径（推荐）
            double radius = (params_.interval * params_.num_uavs) / (2 * M_PI);
            // 可选：添加安全系数，进一步拉开间距
            //radius *= params_.safety_factor;

            for (int i = 0; i < params_.num_uavs; ++i) {
                // 计算第i架无人机的角度（均匀分布 + 30°偏移）
                double angle = 2 * M_PI * i / params_.num_uavs - M_PI / 6;
                // 极坐标转笛卡尔坐标（相对圆心）
                double dx = std::cos(angle) * radius;
                double dy = std::sin(angle) * radius;
                // 叠加中心点坐标，得到最终位置
                positions[i] = center + Point2D(dx, dy);
            }
        }
        else if (formation_type == Formation_Type::Diamond) {
            // 菱形队形
            //double axis_length = params_.interval * (std::ceil(std::sqrt(params_.num_uavs)) / 2);
            //std::vector<Point2D> diamond_vertices = {
            //    Point2D(axis_length, 0),
            //    Point2D(0, axis_length),
            //    Point2D(-axis_length, 0),
            //    Point2D(0, -axis_length)
            //};

            //int pos_idx = 0;
            //// 填充顶点
            //for (const auto& vertex : diamond_vertices) {
            //    if (pos_idx < params_.num_uavs) {
            //        positions[pos_idx++] = vertex;
            //    }
            //}

            //// 填充边
            //std::vector<std::pair<Point2D, Point2D>> edges = {
            //    {diamond_vertices[0], diamond_vertices[1]},
            //    {diamond_vertices[1], diamond_vertices[2]},
            //    {diamond_vertices[2], diamond_vertices[3]},
            //    {diamond_vertices[3], diamond_vertices[0]}
            //};

            //int uav_per_edge = (params_.num_uavs - 4) / 4;
            //int remaining_uavs = (params_.num_uavs - 4) % 4;

            //for (int edge_idx = 0; edge_idx < 4; ++edge_idx) {
            //    auto& edge = edges[edge_idx];
            //    int num_on_edge = uav_per_edge + (edge_idx < remaining_uavs ? 1 : 0);

            //    if (num_on_edge <= 0 || pos_idx >= params_.num_uavs) continue;

            //    for (int step = 1; step <= num_on_edge; ++step) {
            //        if (pos_idx >= params_.num_uavs) break;

            //        double t = static_cast<double>(step) / (num_on_edge + 1);
            //        positions[pos_idx++] = edge.first * (1 - t) + edge.second * t;
            //    }
            //}

            positions.clear();
            positions.reserve(params_.num_uavs); // 预分配内存

            // ------------- 核心优化1：重新计算菱形轴长 -------------
            // 菱形的轴长（从中心到顶点的距离）：保证边的无人机间隔=interval
            // 步骤1：计算每条边需要分配的无人机数量（扣除4个顶点）
            int total_vertex = 4;
            int total_edge_uavs = std::max(0, params_.num_uavs - total_vertex);
            int uav_per_edge = total_edge_uavs / 4;       // 每条边基础数量
            int remaining_uavs = total_edge_uavs % 4;     // 剩余未分配的无人机

            // 步骤2：计算轴长（保证边的无人机间隔=interval）
            // 菱形单边的长度 = 间隔 × (边的无人机数量 + 1) → 轴长 = 单边长度 / √2（菱形边是对角线的√2倍）
            double edge_length = params_.interval * (uav_per_edge + 1); // 单边总长度
            double axis_length = edge_length / std::sqrt(2.0);          // 菱形轴长（中心到顶点）

            // ------------- 步骤1：生成菱形4个顶点 -------------
            std::vector<Point2D> diamond_vertices = {
                center + Point2D(axis_length, 0),       // 右顶点
                center + Point2D(0, axis_length),       // 上顶点
                center + Point2D(-axis_length, 0),      // 左顶点
                center + Point2D(0, -axis_length)       // 下顶点
            };

            // 添加顶点（若无人机数量不足4，只添加对应数量的顶点）
            int pos_idx = 0;
            int vertex_count = std::min(total_vertex, params_.num_uavs);
            for (int i = 0; i < vertex_count; ++i) {
                positions.push_back(diamond_vertices[i]);
                pos_idx++;
            }

            if (pos_idx >= params_.num_uavs)
            {
                positions.clear();
                return positions; // 无人机数量≤4时，直接返回
            }

            // ------------- 步骤2：定义菱形的4条边（顶点两两连接） -------------
            std::vector<std::pair<Point2D, Point2D>> edges = {
                {diamond_vertices[0], diamond_vertices[1]}, // 右→上
                {diamond_vertices[1], diamond_vertices[2]}, // 上→左
                {diamond_vertices[2], diamond_vertices[3]}, // 左→下
                {diamond_vertices[3], diamond_vertices[0]}  // 下→右
            };

            // ------------- 步骤3：为每条边分配无人机（核心优化：无分层） -------------
            for (int edge_idx = 0; edge_idx < 4; ++edge_idx) {
                if (pos_idx >= params_.num_uavs) break;

                auto& edge = edges[edge_idx];
                // 每条边的无人机数量：基础数 + 剩余分配（前remaining_uavs条边多1架）
                int num_on_edge = uav_per_edge + (edge_idx < remaining_uavs ? 1 : 0);
                if (num_on_edge <= 0) continue;

                // ------------- 核心优化2：插值步长计算（避免重叠/分层） -------------
                // t的范围：1/(num_on_edge+1) ~ num_on_edge/(num_on_edge+1)
                // 保证边的无人机在两个顶点之间均匀分布，且不与顶点重叠
                for (int step = 1; step <= num_on_edge; ++step) {
                    if (pos_idx >= params_.num_uavs) break;

                    double t = static_cast<double>(step) / (num_on_edge + 1);
                    // 线性插值：从edge.first到edge.second的t位置
                    Point2D pos = edge.first * (1 - t) + edge.second * t;
                    positions.push_back(pos);
                    pos_idx++;
                }
            }

            // ------------- 兜底：若仍有剩余无人机（极端情况），均匀填充到中心 -------------
            while (pos_idx < params_.num_uavs) {
                positions.push_back(center); // 剩余无人机放在中心（可选：也可随机偏移）
                pos_idx++;
            }
        }
        else if (formation_type == Formation_Type::Line) {
            // 直线队形（沿X轴）
            double start_x = -params_.interval * (params_.num_uavs - 1) / 2;
            for (int i = 0; i < params_.num_uavs; ++i) {
                positions[i] = Point2D(start_x + i * params_.interval, 0);
            }
        }
        else if (formation_type == Formation_Type::Rectangle) {
            // 矩形队形
            int cols = static_cast<int>(std::ceil(std::sqrt(params_.num_uavs)));
            int rows = static_cast<int>(std::ceil(static_cast<double>(params_.num_uavs) / cols));
            double start_x = -params_.interval * (cols - 1) / 2;
            double start_y = -params_.interval * (rows - 1) / 2;

            int idx = 0;
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    if (idx < params_.num_uavs) {
                        positions[idx++] = Point2D(start_x + x * params_.interval, start_y + y * params_.interval);
                    }
                }
            }
        }

        return positions;
    }

    // 核心优化：动态计算α值
    double UAVFormationTransformer::calculateDynamicAlpha(int uav_idx, double current_dist) {
        // 1. 基础参数（可通过params_配置）
        double alpha_max = params_.transition_alpha_max;    // 最大α（远距时）
        double alpha_min = params_.transition_alpha_base;   // 最小α（近距时）
        double far_dist_threshold = initial_distances_[uav_idx] * params_.far_dist_ratio; // 远距阈值
        double near_dist_threshold = 0.06 * 2; // 近距阈值（避免α突变）

        // 2. 边界处理：初始距离为0时直接返回最小α
        if (initial_distances_[uav_idx] < 1e-6) {
            return alpha_min;
        }

        // 3. 动态计算α
        double dynamic_alpha = 0.0;
        if (current_dist > far_dist_threshold) {
            // 远距：使用最大α，快速移动
            dynamic_alpha = alpha_max;
        }
        else if (current_dist <= near_dist_threshold) {
            // 近距：使用最小α，精准收敛
            dynamic_alpha = alpha_min;
        }
        else {
            // 中距：α线性递减（从max到min）
            //double ratio = (current_dist - near_dist_threshold) / (far_dist_threshold - near_dist_threshold);
            double ratio = 0.7;
            dynamic_alpha = alpha_min + (alpha_max - alpha_min) * ratio;
        }

        // 4. 限制α范围（避免异常值）
        dynamic_alpha = std::clamp(dynamic_alpha, 0.01, 0.8); // α∈[0.01, 0.8]，防止超调/停滞

        return dynamic_alpha;
    }


    // ========== 新增核心函数：为每个无人机匹配最近的目标位置（一对一） ==========
    std::vector<int> UAVFormationTransformer::matchClosestTarget() {
        int n = params_.num_uavs;
        std::vector<int> target_mapping(n, -1);  // 存储：当前无人机i → 匹配的目标索引
        std::vector<bool> target_used(n, false); // 标记目标位置是否已被分配

        // 第一步：计算所有当前位置到目标位置的距离矩阵
        std::vector<std::vector<double>> dist_matrix(n, std::vector<double>(n));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                dist_matrix[i][j] = (current_positions_[i] - target_positions_[j]).norm();
            }
        }

        // 第二步：贪心算法实现一对一匹配（优先分配距离最近的）
        // 1. 对每个当前无人机，按距离排序目标位置
        for (int i = 0; i < n; ++i) {
            // 生成目标索引列表，并按到当前无人机i的距离升序排序
            std::vector<int> target_indices(n);
            for (int j = 0; j < n; ++j) target_indices[j] = j;
            std::sort(target_indices.begin(), target_indices.end(),
                [&](int a, int b) { return dist_matrix[i][a] < dist_matrix[i][b]; });

            // 2. 为当前无人机分配第一个未被使用的目标位置
            for (int j : target_indices) {
                if (!target_used[j]) {
                    target_mapping[i] = j;
                    target_used[j] = true;
                    break;
                }
            }
        }

        // 兜底：理论上不会触发，防止极端情况未匹配到目标
        for (int i = 0; i < n; ++i) {
            if (target_mapping[i] == -1) {
                target_mapping[i] = i; // 默认匹配同索引目标
                std::cerr << "警告：无人机" << i << "未匹配到目标位置，使用默认匹配" << std::endl;
            }
        }

        // 首次匹配目标，缓存初始距离
        initial_distances_.resize(params_.num_uavs);
        for (int i = 0; i < params_.num_uavs; ++i) {
            initial_distances_[i] = (current_positions_[i] - target_positions_[target_mapping[i]]).norm();
        }

        return target_mapping;
    }

    /**
    * @brief 碰撞检测与位置调整
    */
    std::vector<Point2D> UAVFormationTransformer::checkCollision(const std::vector<Point2D>& positions) {
        std::vector<Point2D> adjusted = positions;

        for (int i = 0; i < params_.num_uavs; ++i) {
            for (int j = i + 1; j < params_.num_uavs; ++j) {
                Point2D diff = adjusted[i] - adjusted[j];
                double distance = diff.norm();

                if (distance < params_.collision_radius) {
                    // 碰撞规避：沿连线方向分离
                    Point2D dir = diff.normalized();
                    double adjust_step = (params_.collision_radius - distance) / 2;
                    adjusted[i] = adjusted[i] + dir * adjust_step;
                    adjusted[j] = adjusted[j] - dir * adjust_step;
                }
            }
        }

        return adjusted;
    }

    std::vector<Point2D> UAVFormationTransformer::checkCollision1(const std::vector<Point2D>& positions) {
        std::vector<Point2D> adjusted = positions;
        int iter = 0;
        bool has_collision = true;
        //int max_collision_iter = 10;

        // ========== 优化2：迭代调整直到无碰撞或达到最大迭代次数 ==========
        while (has_collision && iter < params_.max_collision_iter) {
            has_collision = false;
            // 存储所有碰撞对（间距，i，j），按间距升序排序
            std::vector<std::tuple<double, int, int>> collision_pairs;

            // 第一步：找出所有碰撞对
            for (int i = 0; i < params_.num_uavs; ++i) {
                for (int j = i + 1; j < params_.num_uavs; ++j) {
                    Point2D diff = adjusted[i] - adjusted[j];
                    double distance = diff.norm();

                    if (distance <= (params_.collision_radius * 2)) {
                        collision_pairs.emplace_back(distance, i, j);
                        has_collision = true;
                    }
                }
            }

            if (!has_collision) break;

            // ========== 优化3：优先处理间距最小的碰撞对 ==========
            std::sort(collision_pairs.begin(), collision_pairs.end(),
                [](const auto& a, const auto& b) {
                    return std::get<0>(a) < std::get<0>(b);
                });

            // 第二步：处理每个碰撞对（按间距从小到大）
            for (const auto& pair : collision_pairs) {
                double distance = std::get<0>(pair);
                int i = std::get<1>(pair);
                int j = std::get<2>(pair);

                Point2D diff = adjusted[i] - adjusted[j];
                // 数值稳定性：避免零向量
                Point2D dir = diff.normalized();

                // ========== 优化4：阻尼限制，避免单次调整过大 ==========
                double need_adjust = params_.collision_radius * 2 - distance;
                double adjust_step = std::min(need_adjust / 2, params_.max_adjust_step);

                // 沿连线方向分离
                adjusted[i] = adjusted[i] + dir * adjust_step;
                adjusted[j] = adjusted[j] - dir * adjust_step;

                // 验证调整后是否仍碰撞（可选，进一步优化）
                double new_dist = (adjusted[i] - adjusted[j]).norm();
                if (new_dist <= (params_.collision_radius * 2)) {
                    // 若仍碰撞，小幅增加调整步长（不超过最大步长）
                    double extra_step = std::min(0.01, params_.max_adjust_step - adjust_step);
                    adjusted[i] = adjusted[i] + dir * extra_step;
                    adjusted[j] = adjusted[j] - dir * extra_step;
                }
            }

            iter++;
        }

        if (iter >= params_.max_collision_iter && has_collision) {
            std::cout << "警告：避碰达到最大迭代次数，仍存在碰撞！" << std::endl;
        }

        return adjusted;
    }

    /**
    * @brief 切换到下一个队形
    */
    void UAVFormationTransformer::switchFormation() {

        Formation_Type new_formation = params_.trans_formation;
        target_positions_ = generateFormation(new_formation);
        target_positions_ = checkCollision(target_positions_);
        trajectory_.addFormationChangeFrame(0);

        string new_for_str = formationToStr(new_formation);
        std::cout << "Switched to " << new_for_str << " formation at frame " << 0 << std::endl;
    }

    /**
     * @brief 更新无人机位置（平滑过渡）
     */
    bool UAVFormationTransformer::updatePositions() {
        // 位置差阈值
        const double POSITION_THRESHOLD = 0.06;
        bool isFormationCompleted = true;

        std::vector<int> target_mapping = matchClosestTarget();

        std::vector<Point2D> temp_positions = current_positions_;
        // 指数平滑过渡
        for (int i = 0; i < params_.num_uavs; ++i) {
            // 获取当前无人机i匹配到的目标位置索引
            int target_idx = target_mapping[i];

            // 计算当前到目标的距离
            double current_dist = (current_positions_[i] - target_positions_[target_idx]).norm();
            // 计算动态α值（核心优化）
            double dynamic_alpha = calculateDynamicAlpha(i, current_dist);

            // 动态α的指数平滑：前期快、后期精准
            temp_positions[i] = current_positions_[i] * (1 - dynamic_alpha) + target_positions_[target_idx] * dynamic_alpha;

            //原来的代码
            /*temp_positions[i] = current_positions_[i] * (1 - params_.transition_alpha)
                + target_positions_[target_idx] * params_.transition_alpha;*/

            //判断
            Point2D diff = temp_positions[i] - target_positions_[target_idx];
            if (diff.norm() > POSITION_THRESHOLD) {
                isFormationCompleted = false;
            }
        }

        // 实时避碰
        //current_positions_ = checkCollision(current_positions_);
        current_positions_ = checkCollision1(temp_positions);

        return isFormationCompleted;
    }

    /**
    * @brief 构造函数（初始化参数）
    */
    UAVFormationTransformer::UAVFormationTransformer(){
        // 初始化位置（默认第一个队形）
        params_ = formation_param_;
        current_positions_.resize(params_.num_uavs);
    }

    void UAVFormationTransformer::InitialFormation(){

        current_positions_ = generateFormation(params_.trans_formation);
        current_positions_ = checkCollision(current_positions_);
        target_positions_ = current_positions_;
        trajectory_.addFormationChangeFrame(0);

        // 记录轨迹
        double current_time = 0;
        Formation_Type current_formation = params_.trans_formation;
        formation_param_.current_formation = current_formation;

        for (int uav_id = 0; uav_id < params_.num_uavs; ++uav_id) {
            trajectory_.addFrame(0, current_time, uav_id, current_formation, current_positions_[uav_id]);
        }

        Formation_Type new_formation = params_.trans_formation;
        string new_for_str = formationToStr(new_formation);
        std::cout << "Switched to " << new_for_str << " formation at frame " << 0 << std::endl;
    }

    /**
        * @brief 运行编队变换计算（生成轨迹）
        */
    void UAVFormationTransformer::runTransformation() {
        //int switch_frame_interval = static_cast<int>(params_.switch_interval * params_.fps);

        for (frame_count_ = 0; frame_count_ < params_.max_frames; ++frame_count_) {
            // 定期切换队形
            //switchFormation();

            // 更新位置
            bool isCompleted = updatePositions();

            // 记录轨迹
            double current_time = static_cast<double>(frame_count_) / params_.fps;
            Formation_Type current_formation = params_.trans_formation;
            for (int uav_id = 0; uav_id < params_.num_uavs; ++uav_id) {
                trajectory_.addFrame(frame_count_, current_time, uav_id, current_formation, current_positions_[uav_id]);
            }

            if (isCompleted || frame_count_ == params_.max_frames - 1) {
                end_trajectory.clear();
                formation_param_.current_formation = current_formation;
                for (int uav_id = 0; uav_id < params_.num_uavs; ++uav_id) {
                    TrajectoryFrame frame_;
                    frame_.frame = frame_count_;
                    frame_.position = current_positions_[uav_id];
                    frame_.formation = current_formation;
                    frame_.time = current_time;
                    frame_.uav_id = uav_id;
                    end_trajectory.push_back(frame_);
                }
                std::cout << "Formation transformation completed at frame: " << frame_count_ << std::endl;
                break;
            }
        }
    }

    /**
        * @brief 获取轨迹数据
        */
    const UAVTrajectory& UAVFormationTransformer::getTrajectory() const {
        return trajectory_;
    }

    /**
        * @brief 获取当前队形名称
        */
    Formation_Type UAVFormationTransformer::getCurrentFormation() const {
        return params_.current_formation;
    }

    /**
        * @brief 获取当前所有无人机位置
        */
    std::vector<Point2D> UAVFormationTransformer::getCurrentPositions() const {
        return current_positions_;
    }

    void UAVFormationTransformer::setCurrentPositions(vector<TrajectoryFrame> positions_){
        for (int i= 0; i < positions_.size(); i++)
        {
            current_positions_[i] = positions_[i].position;
        }
        //current_positions_ = positions_;
    }

    /*string UAVFormationTransformer::formationToStr(Formation_Type type) const
    {
        string formation_str = "";
        switch (type)
        {
        case Formation_Type::Circle:
            formation_str = "Circle";
            break;
        case Formation_Type::Diamond:
            formation_str = "Diamond";
            break;
        case Formation_Type::Line:
            formation_str = "Line";
            break;
        case Formation_Type::Rectangle:
            formation_str = "Rectangle";
            break;
        case Formation_Type::Triangle:
            formation_str = "Triangle";
            break;
        default:
            break;
        }
        return formation_str;
    }*/

    //Json::Value
    void Transformation_Test(Json::Value input, Json::Value& trajectory_result)
    {
        // 1. 初始化参数
        UAVFormationParams params;
        /*params.num_uavs = 20;
        params.interval = 5.0;
        params.collision_radius = 2.5;
        params.switch_interval = 8.0;
        params.max_frames = 1500;*/
        bool isInitial = false;      //是否是初始化

        isInitial = input["is_initial"].asBool();
        formation_param_.num_uavs = input["num_uavs"].asInt();
        formation_param_.interval = input["interval"].asDouble();
        formation_param_.collision_radius = input["collision_radius"].asDouble();
        //formation_param_.switch_interval = input["switch_interval"].asDouble();
        formation_param_.max_frames = input["max_frames"].asInt();
        formation_param_.trans_formation = static_cast<Formation_Type>(input["formation"].asInt());
        formation_param_.pos_center = Point2D(input["pos_center_x"].asDouble(), input["pos_center_y"].asDouble());

        // 2. 创建编队变换实例
        UAVFormationTransformer transformer;

        if (isInitial)      //初始化编队位置
        {
            transformer.InitialFormation();

            auto all_trajectory = transformer.getTrajectory().getAllTrajectory();
            initial_trajectory.clear();
            end_trajectory.clear();
            if (all_trajectory.empty()) return;
            for (auto uav_data : all_trajectory)
            {
                initial_trajectory.push_back(uav_data);
                end_trajectory.push_back(uav_data);
                // 外层：帧IDhxk
                std::string frame_key = "frame_" + std::to_string(uav_data.frame);
                // 内层：节点ID
                std::string node_key = "node_" + std::to_string(uav_data.uav_id);
                trajectory_result[frame_key][node_key]["pos_x"] = uav_data.position.x;
                trajectory_result[frame_key][node_key]["pos_y"] = uav_data.position.y;
            }

            return;
        }
        else                //队形变换
        {
            initial_trajectory = end_trajectory;
            transformer.setCurrentPositions(end_trajectory);
            transformer.switchFormation();
        }

        // 3. 运行编队变换计算
        transformer.runTransformation();

        // 4. 获取并打印轨迹数据
        //const UAVTrajectory& trajectory = transformer.getTrajectory();
        //trajectory.printSummary(params);

        // 5. 示例：获取单个UAV的轨迹
        //Json::Value intial_pos = trajectory_result["intial pos"];
        for (auto uav_data : initial_trajectory)
        {
            // 外层：帧ID
            std::string frame_key = "frame_0";
            // 内层：节点ID
            std::string node_key = "node_" + std::to_string(uav_data.uav_id);
            trajectory_result[frame_key][node_key]["pos_x"] = uav_data.position.x;
            trajectory_result[frame_key][node_key]["pos_y"] = uav_data.position.y;
        }

        auto all_trajectory = transformer.getTrajectory().getAllTrajectory();
        if (all_trajectory.empty()) return;
        for (auto uav_data : all_trajectory)
        {
            // 外层：帧ID
            std::string frame_key = "frame_" + std::to_string(uav_data.frame + 1);
            // 内层：节点ID
            std::string node_key = "node_" + std::to_string(uav_data.uav_id);
            trajectory_result[frame_key][node_key]["pos_x"] = uav_data.position.x;
            trajectory_result[frame_key][node_key]["pos_y"] = uav_data.position.y;
        } 
        /*initial_trajectory.clear();
        initial_trajectory.push_back(all_trajectory.back());*/
    }
}