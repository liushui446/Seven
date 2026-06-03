#include "transformation/imGui_visualizer.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cmath>

namespace seven {

// 预定义节点颜色表
static const unsigned int kNodeColors[] = {
    IM_COL32(255,  80,  80, 255), // 红 (主节点)
    IM_COL32( 80, 180, 255, 255), // 蓝
    IM_COL32( 80, 255,  80, 255), // 绿
    IM_COL32(255, 200,  50, 255), // 橙
    IM_COL32(200,  80, 255, 255), // 紫
    IM_COL32( 50, 220, 220, 255), // 青
    IM_COL32(255, 120, 180, 255), // 粉
    IM_COL32(180, 180,  60, 255), // 黄绿
    IM_COL32( 60, 180, 120, 255), // 海绿
    IM_COL32(140, 100, 200, 255), // 紫罗兰
};

ImGuiVisualizer::ImGuiVisualizer() {
    for (int i = 0; i < 10; ++i)
        node_colors_[i] = kNodeColors[i];
}

ImGuiVisualizer::~ImGuiVisualizer() {
    if (window_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

bool ImGuiVisualizer::initialize(const std::string& title, int width, int height) {
    window_title_  = title;
    window_width_  = width;
    window_height_ = height;

    // 初始化 GLFW
    if (!glfwInit())
        return false;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // vsync

    // 初始化 ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // 不使用 ini 文件

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    setupImGuiStyle();
    running_ = true;
    return true;
}

void ImGuiVisualizer::setupImGuiStyle() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding    = 6.0f;
    style.FrameRounding     = 4.0f;
    style.GrabRounding      = 4.0f;
    style.ChildRounding     = 4.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);

    ImGui::GetIO().FontGlobalScale = 1.1f;
}

void ImGuiVisualizer::requestClose() {
    running_ = false;
}

bool ImGuiVisualizer::isRunning() const {
    return running_ && window_ && !glfwWindowShouldClose(window_);
}

// ====================== 主循环 ======================

void ImGuiVisualizer::run(UUVFormationSimulator* sim) {
    if (!window_ || !sim) return;

    while (isRunning()) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        renderFrame(sim);

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
    }
}

// ====================== 每帧渲染 ======================

void ImGuiVisualizer::renderFrame(UUVFormationSimulator* sim) {
    drawControlPanel(sim);
    drawTopDownView(sim);
    drawNodePanel(sim);
    drawStatsPanel(sim);
}

// ====================== 控制面板 ======================

void ImGuiVisualizer::drawControlPanel(UUVFormationSimulator* sim) {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 260), ImGuiCond_FirstUseEver);
    ImGui::Begin("Control Panel", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    const auto& cfg = sim->get_config();

    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Formation: %s",
                       formationToStr(cfg.current_formation).c_str());

    ImGui::Separator();

    // 回放控制
    ImGui::Text("Playback");
    if (ImGui::Button(playback_paused_ ? "Play" : "Pause", ImVec2(80, 24)))
        playback_paused_ = !playback_paused_;

    ImGui::SameLine();
    ImGui::SliderFloat("Speed", &playback_speed_, 0.1f, 5.0f, "%.1fx");

    // 步进帧
    const auto& traj = sim->getUAVtrajectory();
    int max_frame = static_cast<int>(traj.getFrameCount()) - 1;
    if (max_frame < 0) max_frame = 0;
    if (current_frame_ > max_frame) current_frame_ = max_frame;
    ImGui::SliderInt("Frame", &current_frame_, 0, max_frame);

    ImGui::Separator();

    // 显示选项
    ImGui::Text("Display");
    ImGui::Checkbox("Trails",    &show_trails_);
    ImGui::SameLine();
    ImGui::Checkbox("Targets",   &show_targets_);
    ImGui::SameLine();
    ImGui::Checkbox("Grid",      &show_grid_);
    ImGui::Checkbox("Auto Fit",  &auto_fit_view_);

    ImGui::Separator();

    // 视图控制
    if (ImGui::Button("Reset View"))
        view_offset_x_ = view_offset_y_ = 0.0f, view_scale_ = 1.0f;

    ImGui::Text("Sim Time: %.1f s", sim->getRunframe());

    ImGui::End();
}

// ====================== 2D 俯视图 ======================

static inline ImVec2 worldToScreen(double wx, double wy,
                                   float cx, float cy, float scale,
                                   float sw, float sh) {
    float sx = (float)((wx * scale) + cx + sw * 0.5f);
    float sy = (float)((-wy * scale) + cy + sh * 0.5f); // Y 轴翻转
    return ImVec2(sx, sy);
}

void ImGuiVisualizer::drawTopDownView(UUVFormationSimulator* sim) {
    ImGui::SetNextWindowPos(ImVec2(300, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(800, 700), ImGuiCond_FirstUseEver);
    ImGui::Begin("Top-Down View (ENU)", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar |
                 ImGuiWindowFlags_NoScrollWithMouse);

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 pos  = ImGui::GetWindowPos();
    ImVec2 size = ImGui::GetWindowContentRegionMax();
    float ww = size.x, wh = size.y;
    float cx = pos.x, cy = pos.y;

    // 鼠标拖拽平移
    if (ImGui::IsWindowHovered() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        ImVec2 delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        view_offset_x_ += delta.x;
        view_offset_y_ += delta.y;
        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
    }

    // 鼠标滚轮缩放
    if (ImGui::IsWindowHovered()) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (wheel != 0.0f) {
            view_scale_ *= (1.0f + wheel * 0.1f);
            view_scale_ = std::max(0.05f, std::min(view_scale_, 20.0f));
        }
    }

    // 绘制网格
    if (show_grid_) {
        float grid_step = 10.0f; // 10m
        ImU32 grid_color = IM_COL32(50, 50, 60, 100);
        for (float gx = -200.0f; gx <= 200.0f; gx += grid_step) {
            ImVec2 a = worldToScreen(gx, -200, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
            ImVec2 b = worldToScreen(gx,  200, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
            dl->AddLine(a, b, grid_color);
        }
        for (float gy = -200.0f; gy <= 200.0f; gy += grid_step) {
            ImVec2 a = worldToScreen(-200, gy, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
            ImVec2 b = worldToScreen( 200, gy, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
            dl->AddLine(a, b, grid_color);
        }
        // 坐标轴
        ImVec2 ox0 = worldToScreen(-250, 0, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
        ImVec2 ox1 = worldToScreen( 250, 0, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
        ImVec2 oy0 = worldToScreen(0, -250, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
        ImVec2 oy1 = worldToScreen(0,  250, view_offset_x_, view_offset_y_, view_scale_, ww, wh);
        dl->AddLine(ox0, ox1, IM_COL32(80, 80, 100, 200), 1.5f);
        dl->AddLine(oy0, oy1, IM_COL32(80, 80, 100, 200), 1.5f);
    }

    // 获取轨迹数据
    const auto& traj = sim->getUAVtrajectory();
    const auto& frames = traj.getAllTrajectory();
    if (frames.empty()) {
        ImGui::End();
        return;
    }

    int fidx = current_frame_;
    if (fidx < 0) fidx = 0;
    if (fidx >= (int)frames.size()) fidx = (int)frames.size() - 1;
    const auto& cur_frame = frames[fidx];

    // 自动适配视图
    if (auto_fit_view_ && !frames.empty()) {
        computeViewBounds(cur_frame.nodes_);
    }

    // 绘制轨迹线
    if (show_trails_ && frames.size() > 1) {
        for (size_t nid = 0; nid < cur_frame.nodes_.size() && nid < 10; ++nid) {
            ImU32 col = node_colors_[nid];
            ImU32 trail_col = IM_COL32((col >> IM_COL32_R_SHIFT) & 0xFF,
                                       (col >> IM_COL32_G_SHIFT) & 0xFF,
                                       (col >> IM_COL32_B_SHIFT) & 0xFF,
                                       80);
            for (size_t f = 1; f <= (size_t)fidx && f < frames.size(); ++f) {
                // 检查两个帧都有这个节点
                if (nid >= frames[f].nodes_.size() || nid >= frames[f-1].nodes_.size())
                    continue;
                const auto& na = frames[f-1].nodes_[nid];
                const auto& nb = frames[f].nodes_[nid];
                ImVec2 a = worldToScreen(na.rel_x, na.rel_y,
                                         view_offset_x_, view_offset_y_,
                                         view_scale_, ww, wh);
                ImVec2 b = worldToScreen(nb.rel_x, nb.rel_y,
                                         view_offset_x_, view_offset_y_,
                                         view_scale_, ww, wh);
                dl->AddLine(a, b, trail_col, 1.5f);
            }
        }
    }

    // 绘制目标位置
    if (show_targets_) {
        for (const auto& node : cur_frame.nodes_) {
            ImVec2 tp = worldToScreen(node.target_x, node.target_y,
                                      view_offset_x_, view_offset_y_,
                                      view_scale_, ww, wh);
            dl->AddCircleFilled(tp, 4.0f, IM_COL32(255, 255, 255, 60));
            dl->AddCircle(tp, 4.0f, IM_COL32(255, 255, 255, 120), 0, 2.0f);
        }
    }

    // 绘制节点
    float node_r = 6.0f;
    for (size_t i = 0; i < cur_frame.nodes_.size() && i < 10; ++i) {
        const auto& node = cur_frame.nodes_[i];
        ImVec2 sp = worldToScreen(node.rel_x, node.rel_y,
                                  view_offset_x_, view_offset_y_,
                                  view_scale_, ww, wh);

        // 脱离节点用虚线边框
        if (node.is_leaving) {
            dl->AddCircle(sp, node_r + 2.0f, IM_COL32(255, 100, 100, 200), 0, 2.0f);
        }

        // 加入节点用虚线边框
        if (node.is_joining) {
            dl->AddCircle(sp, node_r + 2.0f, IM_COL32(100, 255, 100, 200), 0, 2.0f);
        }

        dl->AddCircleFilled(sp, node_r, (ImU32)node_colors_[i]);

        // 速度方向指示线
        double hdg_rad = (node.heading) * M_PI / 180.0;
        double dx = std::sin(hdg_rad) * (node_r + 8.0);
        double dy = std::cos(hdg_rad) * (node_r + 8.0);
        ImVec2 ep = worldToScreen(node.rel_x + dx, node.rel_y + dy,
                                  view_offset_x_, view_offset_y_,
                                  view_scale_, ww, wh);
        dl->AddLine(sp, ep, node_colors_[i], 2.0f);

        // 节点 ID 标签
        char label[16];
        snprintf(label, sizeof(label), "%d", node.id);
        dl->AddText(ImVec2(sp.x + 10, sp.y - 8),
                    IM_COL32(255, 255, 255, 200), label);
    }

    // 图例
    dl->AddText(ImVec2(cx + 8, cy + wh - 60),
                IM_COL32(200, 200, 200, 180),
                "Right-drag: pan | Scroll: zoom | Red=Leader");
    dl->AddText(ImVec2(cx + 8, cy + wh - 40),
                IM_COL32(200, 200, 200, 180),
                "Dashed red: leaving | Dashed green: joining");

    ImGui::End();
}

// ====================== 节点状态面板 ======================

void ImGuiVisualizer::drawNodePanel(UUVFormationSimulator* sim) {
    ImGui::SetNextWindowPos(ImVec2(1110, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 320), ImGuiCond_FirstUseEver);
    ImGui::Begin("Node Status", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    const auto& traj = sim->getUAVtrajectory();
    const auto& frames = traj.getAllTrajectory();
    if (frames.empty()) { ImGui::End(); return; }

    int fidx = current_frame_;
    if (fidx < 0) fidx = 0;
    if (fidx >= (int)frames.size()) fidx = (int)frames.size() - 1;
    const auto& cur = frames[fidx];

    if (ImGui::BeginTable("node_table", 7,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_ScrollY, ImVec2(0, 280))) {
        ImGui::TableSetupColumn("ID",    ImGuiTableColumnFlags_WidthFixed, 25);
        ImGui::TableSetupColumn("E (m)", ImGuiTableColumnFlags_WidthFixed, 55);
        ImGui::TableSetupColumn("N (m)", ImGuiTableColumnFlags_WidthFixed, 55);
        ImGui::TableSetupColumn("Spd",   ImGuiTableColumnFlags_WidthFixed, 45);
        ImGui::TableSetupColumn("Hdg",   ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Err",   ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        for (const auto& node : cur.nodes_) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextColored(ImColor(node_colors_[node.id % 10]), "%d", node.id);

            ImGui::TableNextColumn();
            ImGui::Text("%.1f", node.rel_x);

            ImGui::TableNextColumn();
            ImGui::Text("%.1f", node.rel_y);

            ImGui::TableNextColumn();
            ImGui::Text("%.1f", node.speed);

            ImGui::TableNextColumn();
            ImGui::Text("%.0f", node.heading);

            ImGui::TableNextColumn();
            double err = std::hypot(node.rel_x - node.target_x,
                                    node.rel_y - node.target_y);
            ImVec4 err_col = (err < 0.5f) ? ImVec4(0.3f, 0.9f, 0.3f, 1.0f)
                                          : ImVec4(1.0f, 0.6f, 0.2f, 1.0f);
            ImGui::TextColored(err_col, "%.2f", err);

            ImGui::TableNextColumn();
            const char* state = "OK";
            ImVec4 sc = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
            if (node.is_leaving) {
                state = "Leaving";
                sc = ImVec4(1.0f, 0.5f, 0.5f, 1.0f);
            } else if (node.is_joining) {
                state = "Join";
                sc = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
            } else if (node.id == 0) {
                state = "Leader";
                sc = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
            }
            ImGui::TextColored(sc, "%s", state);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

// ====================== 统计面板 ======================

void ImGuiVisualizer::drawStatsPanel(UUVFormationSimulator* sim) {
    ImGui::SetNextWindowPos(ImVec2(1110, 340), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 170), ImGuiCond_FirstUseEver);
    ImGui::Begin("Statistics", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    const auto& traj = sim->getUAVtrajectory();
    const auto& frames = traj.getAllTrajectory();

    ImGui::Text("Total Frames:  %zu", frames.size());
    ImGui::Text("Sim Time:      %.1f s", sim->getRunframe());

    const auto& cfg = sim->get_config();
    ImGui::Text("Node Count:    %d", cfg.node_num);
    ImGui::Text("Spacing:       %.1f m", cfg.rel_distance);

    // 计算平均误差
    if (!frames.empty()) {
        double total_err = 0.0;
        int err_count = 0;
        for (const auto& f : frames) {
            for (const auto& n : f.nodes_) {
                if (n.id == 0) continue;
                total_err += std::hypot(n.rel_x - n.target_x,
                                        n.rel_y - n.target_y);
                err_count++;
            }
        }
        double avg_err = (err_count > 0) ? total_err / err_count : 0.0;
        ImGui::Text("Avg Error:     %.3f m", avg_err);
    }

    ImGui::End();
}

// ====================== 辅助函数 ======================

void ImGuiVisualizer::computeViewBounds(const std::vector<UUVNode>& nodes) {
    if (nodes.empty()) return;
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& n : nodes) {
        min_x = std::min(min_x, n.rel_x);
        max_x = std::max(max_x, n.rel_x);
        min_y = std::min(min_y, n.rel_y);
        max_y = std::max(max_y, n.rel_y);
    }
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    double range   = std::max(range_x, range_y) * 1.3f;
    if (range < 20.0) range = 20.0;

    // 不自动修改 offset，只调整 scale（通过 setTrajectoryData 等方式）
    // 这里只计算不修改成员变量，避免频繁跳动
}

void ImGuiVisualizer::setTrajectoryData(const UAVTrajectory& traj) {
    current_frame_ = 0;
}

}  // namespace seven
