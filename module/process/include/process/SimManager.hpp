#ifndef SIMMANAGER_HPP
#define SIMMANAGER_HPP

#include "core/CommonCore.hpp"
//#include "json/json.h"
#include <windows.h>

namespace seven {

    // 核心仿真管理器类
    class SimManager {
    public:
        enum class SimState {
            ENDDING = 0,  // 仿真终止
            STOPPED = 1,  // 仿真暂停
            RUNNING = 2,   // 仿真运行中
            IDLE    = 3   // 仿真准备状态
        };

        SimManager();

        // 1. 仿真准备接口
        int sim_start(const Json::Value& input, Json::Value& result);

        // 2. 仿真开始(多平台航迹计算接口)
        int sim_calc(HANDLE hPipe, const Json::Value& input, Json::Value& result);

        // 3. 仿真暂停接口
        int sim_stop(Json::Value& result);

        // 3. 仿真结束接口
        int sim_end(Json::Value& result);

    private:
        // 初始化仿真配置
        void init_sim_config(const Json::Value& input, Json::Value& result);

        SimState sim_state_;          // 仿真状态
        //SimConfig config_;            // 仿真配置
        UINT sim_time_;               // 仿真时长
        std::mutex sim_mutex_;        // 线程安全锁
    };

    // 全局仿真管理器实例（也可以根据需要改为局部/成员变量）
    static SimManager g_sim_manager;

}

#endif
