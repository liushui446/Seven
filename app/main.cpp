#include "core/core.hpp"
#include "process/process.hpp"

// ===================== 跨平台初始化 =====================
#ifdef _WIN32
    // Windows 版本：简单初始化
    int main(int argc, char* argv[])
    {
        seven::Core::get_init()->timeout = 100;
        double x = seven::Core::get_init()->timeout;

        //seven::Transformation_Test();

        //seven::start_pipe_server();
        //seven::start_pipe_server_test();
        seven::start_double_pipe_test();

        // ========== 关键步骤4：正常退出时销毁线程 ==========
        //calc_thread_ptr->UnInit();
        std::cout << "程序正常退出，线程资源已清理" << std::endl;

        return 0;
    }

#else
    // Linux / 麒麟V10 版本：适配中文控制台编码
    #include <cstdio>
    #include <locale.h>

    // 麒麟V10系统宏（由CMake定义）
    #ifdef KYLIN_V10_OS
    #define KYLIN_OS_MSG "当前运行在银河麒麟V10系统"
    #else
    #define KYLIN_OS_MSG "非麒麟V10系统"
    #endif

    int main(int argc, char* argv[]) {
        // 适配麒麟V10中文控制台编码
        setlocale(LC_ALL, "zh_CN.UTF-8");

        // 控制台输出（示例业务逻辑）
        std::cout << "=====================================" << std::endl;
        std::cout << "        Seven 控制台程序（Linux版）        " << std::endl;
        std::cout << "=====================================" << std::endl;
        std::cout << KYLIN_OS_MSG << std::endl;
        std::cout << "程序参数个数：" << argc << std::endl;

        // 调用业务模块
        std::cout << "\n调用业务模块：" << std::endl;
        seven::start_double_pipe_test();

        std::cout << "\n程序执行完成！" << std::endl;
        return 0;
    }

#endif
