#ifndef TRANS_HPP
#define TRANS_HPP

#include "core/CommonCore.hpp"
#include <unordered_map>

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

    string formationToStr(Formation_Type type);

    /**
    * @brief 轨迹存储类
    */
    class UAVTrajectory {
    private:
        std::vector<TrajectoryFrame> trajectory_data; // 所有轨迹数据
        std::vector<int> formation_change_frames;   // 队形变换帧数记录

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
    };

    // UUV编队仿真核心类
    class UUVFormationSimulator {
    private:
        FormationConfig config;
        std::vector<UUVNode> nodes;
        UAVTrajectory trajectory_;
        double current_time;
        double last_output_time;
        bool is_transition;
        Formation_Type last_formation;
        int max_id;
        std::mutex sim_mutex;

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
    };

    // ====================== 多编队仿真器管理 ======================
    // 全局仿真器映射表：key = formation_id, value = 仿真器实例
    extern std::unordered_map<int, UUVFormationSimulator*> g_FormationSimulators;

    // 根据 formation_id 获取对应仿真器（带空指针检查）
    inline UUVFormationSimulator* GetFormationSimulator(int formation_id) {
        auto it = g_FormationSimulators.find(formation_id);
        if (it != g_FormationSimulators.end()) {
            return it->second;
        }
        return nullptr;
    }

    // 清理所有编队仿真器
    void SEVEN_EXPORTS Cleanup_All_Formations();

    // ====================== 外部接口声明 ======================
    // 初始化单个编队（兼容旧接口，内部使用 formation_id = 0）
    void SEVEN_EXPORTS Init_formation(const FormationConfig& config, Json::Value& trajectory_result);

    // 初始化多个编队（新接口）
    void SEVEN_EXPORTS Init_Multi_Formation(const MultiFormationContext& context, Json::Value& result);

    // 编队变换执行（每帧调用，遍历所有编队并合并输出）
    void SEVEN_EXPORTS Transformation_Use(CalcTempParam& task_param);

    // 多编队变换执行（每帧调用，仅执行指定编队）
    void SEVEN_EXPORTS Transformation_Use_Multi(int formation_id, CalcTempParam& task_param);

    // 队形切换（指定编队ID）
    void SEVEN_EXPORTS SwitchFormation(int formation_id, const Formation_Type& type);

    // 转向控制（指定编队ID）
    void SEVEN_EXPORTS TurnFormation(int formation_id, double heading_rate);

    // 添加节点（指定编队ID）
    void SEVEN_EXPORTS AddNode(int formation_id, vector<UUVNode>& input);

    // 删除末尾节点（指定编队ID）
    void SEVEN_EXPORTS RemoveLastNode(int formation_id, int num);

}
#endif
