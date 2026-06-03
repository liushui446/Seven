#include "core/core.hpp"
#include "process/process.hpp"
#include "transformation/transformation.hpp"
#include "transformation/imGui_visualizer.hpp"

#include <iostream>
#include <cstring>
#include <thread>

int main(int argc, char *argv[])
{
    seven::Core::get_init()->timeout = 100;

    // 解析命令行参数
    bool use_visualizer = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--viz") == 0 || std::strcmp(argv[i], "-v") == 0) {
            use_visualizer = true;
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            std::cout << "Seven UUV Formation Simulation\n"
                      << "Usage: app.exe [options]\n"
                      << "  --viz, -v    Launch with ImGui visualizer\n"
                      << "  --pipe, -p   Run as pipe server (default)\n"
                      << "  --help, -h   Show this help\n";
            return 0;
        }
    }

    if (use_visualizer) {
        // ==================== ImGui 可视化模式 ====================
        std::cout << "Starting UUV Formation Visualizer...\n";

        // 在后台线程启动管道服务器
        std::thread pipe_thread([]() {
            seven::start_double_pipe_test();
        });
        pipe_thread.detach();

        // 主线程运行可视化器
        seven::ImGuiVisualizer visualizer;
        if (!visualizer.initialize("UUV Formation Simulation - Seven", 1400, 900)) {
            std::cerr << "Failed to initialize visualizer\n";
            return -1;
        }

        // 等待仿真器被初始化（通过管道客户端触发）
        std::cout << "Waiting for simulation to be initialized via pipe client...\n";
        while (seven::g_pFormationSimulator == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Simulation ready, starting visualization...\n";
        visualizer.run(seven::g_pFormationSimulator);

        std::cout << "Visualizer closed. Exiting...\n";
    } else {
        // ==================== 管道服务端模式 (默认) ====================
        std::cout << "Starting pipe server mode...\n";
        seven::start_double_pipe_test();
    }

    std::cout << "Program exited normally.\n";
    return 0;
}
