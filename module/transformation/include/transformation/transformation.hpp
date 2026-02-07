#ifndef TRANS_HPP
#define TRANS_HPP

#include "core/CommonCore.hpp"

namespace seven {

    string formationToStr(Formation_Type type);

    /**
     * @brief 二维坐标结构体
     */
    struct Point2D {
        double x = 0.0;
        double y = 0.0;

        Point2D() = default;
        Point2D(double x_, double y_) : x(x_), y(y_) {}

        // 运算符重载：向量运算
        Point2D operator+(const Point2D& other) const {
            return { x + other.x, y + other.y };
        }

        Point2D operator-(const Point2D& other) const {
            return { x - other.x, y - other.y };
        }

        Point2D operator*(double scalar) const {
            return { x * scalar, y * scalar };
        }

        Point2D operator/(double scalar) const {
            return { x / scalar, y / scalar };
        }

        Point2D& operator=(const Point2D& other) {
            if (this == &other) {  // 处理自赋值
                return *this;
            }
            x = other.x;
            y = other.y;
            return *this;
        }

        // 向量长度
        double norm() const {
            return std::sqrt(x * x + y * y);
        }

        // 单位向量
        Point2D normalized() const {
            double n = norm();
            return n < 1e-6 ? Point2D(0, 0) : Point2D(x / n, y / n);
        }
    };

    /**
    * @brief 初始化参数结构体
    */
    struct UAVFormationParams {
        int num_uavs = 8;                // 节点数量
        double interval = 5.0;           // 队形节点间间隔（米）
        double collision_radius = 2.0;   // 避碰半径（米）
        //double switch_interval = 5.0;    // 队形切换间隔（秒）
        double transition_alpha = 0.05;  // 位置过渡系数（越小越平滑）
        int fps = 30;                    // 帧率（用于时间换算）
        int max_frames = 1500;           // 最大运行帧数
        bool isInitial = false;          // 是否进行编队初始化

        // 队形序列（默认：矩形→三角形→圆形→菱形→直线）
        //std::vector<std::string> formation_sequence = { "rectangle", "triangle", "circle", "diamond", "line" };
        Formation_Type trans_formation;  // 需要变换的队形
        Formation_Type current_formation;// 当前队形
        Point2D pos_center;              // 队形中心点位置


        UAVFormationParams& operator=(const UAVFormationParams& other) {
            if (this == &other) {
                return *this;
            }

            num_uavs = other.num_uavs;
            interval = other.interval;
            collision_radius = other.collision_radius;
            //switch_interval = other.switch_interval;
            transition_alpha = other.transition_alpha;
            fps = other.fps;
            max_frames = other.max_frames;
            isInitial = other.isInitial;

            trans_formation = other.trans_formation;
            current_formation = other.current_formation;
            pos_center = other.pos_center;

            return *this;
        }

        //UAVFormationParams(const UAVFormationParams& other) {
        //    *this = other;  // 复用operator=，避免代码冗余
        //}
    };
    

    /**
     * @brief 单帧轨迹数据结构体
     */
    struct TrajectoryFrame {
        int frame = 0;          // 帧数
        double time = 0.0;      // 时间（秒）
        int uav_id = 0;         // 节点ID
        Formation_Type formation;  // 当前队形
        Point2D position;       // 位置坐标
    };

    /**
    * @brief 轨迹存储类
    */
    class UAVTrajectory {
    private:
        std::vector<TrajectoryFrame> trajectory_data;  // 所有轨迹数据
        std::vector<int> formation_change_frames;      // 队形变换帧数记录

    public:
        /**
         * @brief 添加单帧轨迹数据
         */
        void addFrame(int frame, double time, int uav_id, const Formation_Type formation, const Point2D& pos);

        /**
         * @brief 记录队形变换帧数
         */
        void addFormationChangeFrame(int frame);

        /**
         * @brief 获取指定UAV的完整轨迹
         * @param uav_id 无人机ID
         * @return 该无人机的所有轨迹帧
         */
        std::vector<TrajectoryFrame> getUAVTrajectory(int uav_id) const;

        /**
         * @brief 获取所有轨迹数据
         */
        const std::vector<TrajectoryFrame>& getAllTrajectory() const;

        /**
         * @brief 获取队形变换帧数记录
         */
        const std::vector<int>& getFormationChangeFrames() const;
    };

    /**
    * @brief 无人机编队变换核心类
    */
    class UAVFormationTransformer {
    private:
        UAVFormationParams params_;              // 初始化参数
        UAVTrajectory trajectory_;               // 轨迹数据
        std::vector<Point2D> current_positions_; // 当前位置
        std::vector<Point2D> target_positions_;  // 目标位置
        //int current_formation_idx_ = 0;        // 当前队形索引
        Formation_Type current_formation;        // 当前队形
        int frame_count_ = 0;                    // 当前帧数

        /**
         * @brief 生成指定类型的队形位置
         */
        std::vector<Point2D> generateFormation(const Formation_Type& formation_type);

        /**
        * @brief 碰撞检测与位置调整
        */
        std::vector<Point2D> checkCollision(const std::vector<Point2D>& positions);

        /**
         * @brief 更新无人机位置（平滑过渡）
         */
        bool updatePositions();

    public:
        /**
         * @brief 构造函数（初始化参数）
         */
        UAVFormationTransformer();

        //初始化队形
        void InitialFormation();

        /**
        * @brief 切换到下一个队形
        */
        void switchFormation();

        /**
         * @brief 运行编队变换计算（生成轨迹）
         */
        void runTransformation();

        /**
         * @brief 获取轨迹数据
         */
        const UAVTrajectory& getTrajectory() const;

        /**
         * @brief 获取当前队形名称
         */
        Formation_Type getCurrentFormation() const;

        /**
         * @brief 获取当前所有无人机位置
         */
        std::vector<Point2D> getCurrentPositions() const;

        /**
         * @brief 队形类型转string
         */
        //string formationToStr(Formation_Type type) const;

    };

    void SEVEN_EXPORTS Transformation_Test(Json::Value input, Json::Value& trajectory_result);

    static UAVFormationParams formation_param_;
}

#endif
