#include "core/core.hpp"
#include "process/process.hpp"
#include "transformation/transformation.hpp"

int main(int argc, char *argv[])
{
    seven::Core::get_init()->timeout = 100;
    double x = seven::Core::get_init()->timeout;

    //seven::Transformation_Test();

    seven::start_pipe_server();

    return 0;
}
