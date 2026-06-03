#ifndef TRANS_HPP
#define TRANS_HPP

#include "core/CommonCore.hpp"

// transformation 模块导出宏
#ifdef TRANSFORMATION_EXPORTS_DLL
#   define TRANSFORMATION_EXPORTS __declspec(dllexport)
#else
#   define TRANSFORMATION_EXPORTS __declspec(dllimport)
#endif

namespace seven{

    // 全局常量定义
    const double R_EARTH = 6378137.0;
    const double TRANSITION_SPEED = 0.08;
    const double COLLISION_RADIUS = 4.0;
    const int MAX_COLLISION_ITER = 15;
    const double MAX_ADJUST_STEP = 1.0;
    const double ERROR_STABLE_THRESHOLD = 0.02;

    // 运动学物理约束
    const double MAX_SPEED = 6.0;

    struct UUVNode;
    struct FormationConfig;
    class UUVFormationSimulator;

    TRANSFORMATION_EXPORTS string formationToStr(Formation_Type type);

    /**
    * @brief 轨迹存储类
    */
    class TRANSFORMATION_EXPORTS UAVTrajectory {
    private:
        std::vector<TrajectoryFrame> trajectory_data;  // 所有轨迹数据
        std::vector<int> formation_change_frames;      // 队形变换帧数记录
        FormationConfig config_snapshot;               // 仿真配置快照
        double simulation_end_time;                    // 总仿真时长

    public:
        /**
         * @brief 添加单帧轨迹数据
         */
        void addFrame(int frame, const Formation_Type formation, const vector<UUVNode>& nodes);

        /**
         * @brief 记录队形变换帧数
         */
        void addFormationChangeFrame(int frame);

        /**
         * @brief 清空所有轨迹数据
         */
        void clearAllTrajectory();

        /**
         * @brief 获取所有轨迹数据
         */
        const std::vector<TrajectoryFrame>& getAllTrajectory() const;

        /**
         * @brief 获取队形变换帧数记录
         */
        const std::vector<int>& getFormationChangeFrames() const;

        /**
         * @brief 设置配置快照和仿真时长
         */
        void setMetadata(const FormationConfig& cfg, double sim_time) {
            config_snapshot = cfg;
            simulation_end_time = sim_time;
        }

        /**
         * @brief 获取配置快照
         */
        const FormationConfig& getConfigSnapshot() const { return config_snapshot; }

        /**
         * @brief 获取总仿真时长
         */
        double getSimulationTime() const { return simulation_end_time; }

        /**
         * @brief 获取总帧数
         */
        size_t getFrameCount() const { return trajectory_data.size(); }

        /**
         * @brief 序列化为JSON字符串
         */
        std::string serializeToJSON() const;

        /**
         * @brief 序列化为二进制文件
         */
        bool serializeToBinary(const std::string& filename) const;
    };

    // UUV编队仿真核心类
    class TRANSFORMATION_EXPORTS UUVFormationSimulator {
    private:
        FormationConfig config;
        std::vector<UUVNode> nodes;
        UAVTrajectory trajectory_;  //轨迹数据
        double current_time;
        double last_output_time;
        bool is_transition;
        Formation_Type last_formation;
        int max_id; // 跟踪最大节点ID
        std::mutex sim_mutex;  // 线程安全锁

        // 私有方法
        void _validate_config();
        void _init_nodes();
        void _set_target_formation();
        void _set_initial_position();
        std::vector<std::pair<double, double>> _generate_formation_positions(int cnt);
        void _transition_formation();
        std::vector<Point2D> checkCollision1(const std::vector<Point2D>& positions);
        void apply_collision_avoidance();
        std::pair<double, double> _geo2enu(double lon, double lat, double rlon, double rlat);
        std::pair<double, double> _enu2geo(double x, double y, double rlon, double rlat);
        void _update_maneuver();
        void _record_transition_step();

    public:
        // 构造函数
        UUVFormationSimulator(const FormationConfig& cfg);

        // 队形切换
        void switch_formation(const Formation_Type& cmd);

        void add_node(vector<UUVNode>& input);

        void remove_last_node(int num);

        // 仿真步进
        UAVTrajectory& step_simulation();

        // 获取当前配置
        FormationConfig get_config() const { return config; }

        double getRunframe() const { return current_time; }

        // 获取当前轨迹数据
        UAVTrajectory& getUAVtrajectory();

        double _calculate_formation_error(const UUVNode& node);

        // 设置航向变化率
        void set_heading_rate(double rate) { config.heading_rate = rate; }

        void InitialParams(FormationConfig& forparams_);

        /**
         * @brief 导出轨迹数据为JSON字符串
         */
        std::string exportTrajectoryJSON() const;

        /**
         * @brief 导出轨迹数据为二进制文件
         * @param filename 输出文件路径
         * @return 是否成功
         */
        bool exportTrajectoryBinary(const std::string& filename) const;

        /**
         * @brief 获取当前仿真状态
         */
        Json::Value getSimulationStatus() const;

        /**
         * @brief 获取当前轨迹数据的详细统计信息
         */
        Json::Value getTrajectoryStatistics();
    };

    // 外部接口声明（集成到服务端的核心接口）
    // 初始化编队
    void SEVEN_EXPORTS Init_formation(const FormationConfig& config, Json::Value& trajectory_result);

    // 编队变换执行（每帧调用）
    void SEVEN_EXPORTS Transformation_Use(CalcTempParam& task_param);

    void SEVEN_EXPORTS SwitchFormation(const Formation_Type& type);

    void SEVEN_EXPORTS TurnFormation(double heading_rate);

    void SEVEN_EXPORTS AddNode(vector<UUVNode>& input);

    void SEVEN_EXPORTS RemoveLastNode(int num);

    // 全局仿真器实例（服务端单例使用）
    extern TRANSFORMATION_EXPORTS UUVFormationSimulator* g_pFormationSimulator;

}


#endif
