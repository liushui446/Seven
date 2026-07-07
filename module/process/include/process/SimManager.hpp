#ifndef SIMMANAGER_HPP
#define SIMMANAGER_HPP

#include "core/CommonCore.hpp"
#include <windows.h>
#include "CalcThread.hpp"

namespace seven {

    class SimManager {
    public:

        SimManager();

        int sim_start(const Json::Value& input, Json::Value& result);

        int sim_calc(HANDLE hPipe, const Json::Value& input, Json::Value& result);

        int sim_stop(Json::Value& result);

        int sim_end(Json::Value& result);

    private:
        void init_sim_config(const Json::Value& input, Json::Value& result);

        SimState sim_state_;
        UINT sim_time_;
        std::mutex sim_mutex_;

        std::shared_ptr<CalcProcessThread> calc_thread_ptr;
    };

    static SimManager g_sim_manager;

}

#endif
