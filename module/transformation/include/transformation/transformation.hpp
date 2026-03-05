#ifndef TRANS_HPP
#define TRANS_HPP

#include "core/CommonCore.hpp"

namespace seven {

    string formationToStr(Formation_Type type);

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
        std::vector<double> initial_distances_;   // 缓存每架无人机到匹配目标的初始距离
        //int current_formation_idx_ = 0;        // 当前队形索引
        Formation_Type current_formation;        // 当前队形
        int frame_count_ = 0;                    // 当前帧数

        /**
         * @brief 生成指定类型的队形位置
         */
        std::vector<Point2D> generateFormation(const Formation_Type& formation_type);

        //动态调整平缓指数
        double calculateDynamicAlpha(int uav_idx, double current_dist);

        /**
         * @brief 匹配最近的目标位置
         */
        std::vector<int> matchClosestTarget();

        /**
        * @brief 碰撞检测与位置调整
        */
        std::vector<Point2D> checkCollision(const std::vector<Point2D>& positions);

        std::vector<Point2D> checkCollision1(const std::vector<Point2D>& positions);

        /**
         * @brief 更新无人机位置（平滑过渡）
         */
        bool updatePositions();

    public:
        /**
         * @brief 构造函数（初始化参数）
         */
        UAVFormationTransformer();

        //初始化参数
        void InitialParams(UAVFormationParams& forparams_);

        //初始化队形
        Formation_Type InitialFormation();

        /**
        * @brief 切换到下一个队形
        */
        void switchFormation();

        /**
         * @brief 运行编队变换计算（生成轨迹）
         */
        Formation_Type runTransformation(vector<TrajectoryFrame>& end_trajectory);

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

        void setCurrentPositions(vector<TrajectoryFrame> positions_);

        /**
         * @brief 队形类型转string
         */
        //string formationToStr(Formation_Type type) const;

    };

    //void SEVEN_EXPORTS Transformation_Test(Json::Value input, Json::Value& trajectory_result);
    void SEVEN_EXPORTS Transformation_Use(UAVFormationParams& params, vector<TrajectoryFrame>& initial_trajectory,
        vector<TrajectoryFrame>& end_trajectory, Json::Value input, Json::Value& trajectory_result);
    void SEVEN_EXPORTS Init_formation(UAVFormationParams& formation_param_, vector<TrajectoryFrame>& initial_trajectory,
        vector<TrajectoryFrame>& end_trajectory, Json::Value& trajectory_result);

    //static UAVFormationParams formation_param_;
    //static vector<TrajectoryFrame> initial_trajectory;
    //static vector<TrajectoryFrame> end_trajectory;
}

#endif
