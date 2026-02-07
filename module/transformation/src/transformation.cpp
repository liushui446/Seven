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
            // 等腰三角形队形（底边严格水平）
            double base_ratio = 1.6;
            double height_ratio = 1;
            double base_length = params_.interval * base_ratio * std::ceil(std::sqrt(params_.num_uavs));
            double height = params_.interval * height_ratio * std::ceil(std::sqrt(params_.num_uavs));
            double base_y = -2 * height / 3;

            // 三角形顶点
            Point2D vertex_top(0, height / 3);
            Point2D vertex_left(-base_length / 2, base_y);
            Point2D vertex_right(base_length / 2, base_y);
            std::vector<Point2D> vertices = { vertex_top, vertex_left, vertex_right };

            // 计算边长
            double side1_len = (vertex_left - vertex_top).norm();
            double side2_len = (vertex_right - vertex_left).norm();
            double side3_len = (vertex_top - vertex_right).norm();
            double total_len = side1_len + side2_len + side3_len;

            // 按边长比例分配无人机
            int uav_side1 = static_cast<int>(std::ceil(params_.num_uavs * side1_len / total_len));
            int uav_side2 = static_cast<int>(std::ceil(params_.num_uavs * side2_len / total_len));
            int uav_side3 = params_.num_uavs - uav_side1 - uav_side2;
            std::vector<int> uavs_per_side = { uav_side1, uav_side2, uav_side3 };

            int idx = 0;
            for (int side = 0; side < 3; ++side) {
                Point2D start = vertices[side];
                Point2D end = vertices[(side + 1) % 3];
                int num_uav = uavs_per_side[side];

                if (num_uav <= 0 || idx >= params_.num_uavs) continue;

                for (int i = 0; i < num_uav; ++i) {
                    if (idx >= params_.num_uavs) break;

                    double t = (num_uav > 1) ? static_cast<double>(i) / (num_uav - 1) : 0.0;
                    double x = start.x * (1 - t) + end.x * t;
                    double y = start.y * (1 - t) + end.y * t;

                    // 底边强制统一y坐标
                    if (side == 1) {
                        y = base_y;
                    }

                    positions[idx++] = Point2D(x, y) + center;
                }
            }
        }
        else if (formation_type == Formation_Type::Circle) {
            // 圆形队形
            for (int i = 0; i < params_.num_uavs; ++i) {
                double angle = 2 * M_PI * i / params_.num_uavs - M_PI / 6;
                double radius = params_.interval * std::sqrt(params_.num_uavs) / 2;
                positions[i] = center + Point2D(std::cos(angle) * radius, std::sin(angle) * radius);
            }
        }
        else if (formation_type == Formation_Type::Diamond) {
            // 菱形队形
            double axis_length = params_.interval * (std::ceil(std::sqrt(params_.num_uavs)) / 2);
            std::vector<Point2D> diamond_vertices = {
                Point2D(axis_length, 0),
                Point2D(0, axis_length),
                Point2D(-axis_length, 0),
                Point2D(0, -axis_length)
            };

            int pos_idx = 0;
            // 填充顶点
            for (const auto& vertex : diamond_vertices) {
                if (pos_idx < params_.num_uavs) {
                    positions[pos_idx++] = vertex;
                }
            }

            // 填充边
            std::vector<std::pair<Point2D, Point2D>> edges = {
                {diamond_vertices[0], diamond_vertices[1]},
                {diamond_vertices[1], diamond_vertices[2]},
                {diamond_vertices[2], diamond_vertices[3]},
                {diamond_vertices[3], diamond_vertices[0]}
            };

            int uav_per_edge = (params_.num_uavs - 4) / 4;
            int remaining_uavs = (params_.num_uavs - 4) % 4;

            for (int edge_idx = 0; edge_idx < 4; ++edge_idx) {
                auto& edge = edges[edge_idx];
                int num_on_edge = uav_per_edge + (edge_idx < remaining_uavs ? 1 : 0);

                if (num_on_edge <= 0 || pos_idx >= params_.num_uavs) continue;

                for (int step = 1; step <= num_on_edge; ++step) {
                    if (pos_idx >= params_.num_uavs) break;

                    double t = static_cast<double>(step) / (num_on_edge + 1);
                    positions[pos_idx++] = edge.first * (1 - t) + edge.second * t;
                }
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
        const double POSITION_THRESHOLD = 0.01;
        bool isFormationCompleted = true;

        // 指数平滑过渡
        for (int i = 0; i < params_.num_uavs; ++i) {
            current_positions_[i] = current_positions_[i] * (1 - params_.transition_alpha)
                + target_positions_[i] * params_.transition_alpha;

            //判断
            Point2D diff = current_positions_[i] - target_positions_[i];
            if (diff.norm() > POSITION_THRESHOLD) {
                isFormationCompleted = false;
            }
        }

        // 实时避碰
        current_positions_ = checkCollision(current_positions_);

        return isFormationCompleted;
    }

    /**
    * @brief 构造函数（初始化参数）
    */
    UAVFormationTransformer::UAVFormationTransformer(){
        // 初始化位置（默认第一个队形）
        params_ = formation_param_;
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
            if (isCompleted) {
                std::cout << "Formation transformation completed at frame: " << frame_count_ << std::endl;
                break;
            }

            // 记录轨迹
            double current_time = static_cast<double>(frame_count_) / params_.fps;
            Formation_Type current_formation = params_.trans_formation;
            formation_param_.current_formation = current_formation;

            for (int uav_id = 0; uav_id < params_.num_uavs; ++uav_id) {
                trajectory_.addFrame(frame_count_, current_time, uav_id, current_formation, current_positions_[uav_id]);
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

            if (all_trajectory.empty()) return;
            for (auto uav_data : all_trajectory)
            {
                // 外层：帧ID
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
            transformer.switchFormation();
        }

        // 3. 运行编队变换计算
        transformer.runTransformation();

        // 4. 获取并打印轨迹数据
        //const UAVTrajectory& trajectory = transformer.getTrajectory();
        //trajectory.printSummary(params);

        // 5. 示例：获取单个UAV的轨迹
        //Json::Value intial_pos = trajectory_result["intial pos"];

        auto all_trajectory = transformer.getTrajectory().getAllTrajectory();

        if (all_trajectory.empty()) return;
        for (auto uav_data : all_trajectory)
        {
            // 外层：帧ID
            std::string frame_key = "frame_" + std::to_string(uav_data.frame);
            // 内层：节点ID
            std::string node_key = "node_" + std::to_string(uav_data.uav_id);
            trajectory_result[frame_key][node_key]["pos_x"] = uav_data.position.x;
            trajectory_result[frame_key][node_key]["pos_y"] = uav_data.position.y;
        } 
    }
}