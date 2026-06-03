#ifndef IMGUI_VISUALIZER_HPP
#define IMGUI_VISUALIZER_HPP

#include "core/CommonCore.hpp"
#include "transformation/transformation.hpp"

// 前置声明 GLFW 窗口句柄（避免在头文件中引入 GLFW）
struct GLFWwindow;

namespace seven {

/**
 * @brief UUV 编队仿真 ImGui 可视化器
 *
 * 提供 GLFW + ImGui + OpenGL3 的实时可视化窗口：
 *   - 2D 俯视图（ENU 坐标系）绘制节点位置、目标、轨迹
 *   - 队形类型、节点状态、仿真统计面板
 *   - 回放控制（播放/暂停、速度调节、帧选择）
 */
class ImGuiVisualizer {
public:
    ImGuiVisualizer();
    ~ImGuiVisualizer();

    bool initialize(const std::string& title = "UUV Formation Visualizer",
                    int width = 1400, int height = 900);

    void run(UUVFormationSimulator* sim);
    void requestClose();
    bool isRunning() const;
    void setTrajectoryData(const UAVTrajectory& traj);

private:
    GLFWwindow* window_ = nullptr;
    std::string window_title_;
    int window_width_  = 1400;
    int window_height_ = 900;
    bool running_ = false;

    float view_offset_x_ = 0.0f;
    float view_offset_y_ = 0.0f;
    float view_scale_    = 1.0f;

    bool  playback_paused_ = false;
    float playback_speed_  = 1.0f;
    int   current_frame_   = 0;
    bool  show_trails_     = true;
    bool  show_targets_    = true;
    bool  show_grid_       = true;
    bool  auto_fit_view_   = true;

    unsigned int node_colors_[10];  // ImU32 color values

    void renderFrame(UUVFormationSimulator* sim);
    void drawTopDownView(UUVFormationSimulator* sim);
    void drawControlPanel(UUVFormationSimulator* sim);
    void drawNodePanel(UUVFormationSimulator* sim);
    void drawStatsPanel(UUVFormationSimulator* sim);
    void setupImGuiStyle();
    void computeViewBounds(const std::vector<UUVNode>& nodes);
};

}  // namespace seven

#endif  // IMGUI_VISUALIZER_HPP
