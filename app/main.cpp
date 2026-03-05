#include "core/core.hpp"
#include "process/process.hpp"


int main(int argc, char *argv[])
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
