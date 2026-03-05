#pragma once

#include <vector>
#include <string>
#include <map>
#include "core/CommonCore.hpp"

namespace seven
{
    

    class SEVEN_EXPORTS CalcParamManager
    {
    public:
        static CalcParamManager& Ins();

    private:
        CalcParamManager();
        ~CalcParamManager();

    public:
        bool ClearAllData();

        bool SetReturnFramesCount(UINT vRes);
        bool SetRunFramesCnt(UINT vRes);
        bool SetSimTime(UINT vRes);
        CalcParam GetCalcParam();

        void SwapPlatform(std::vector<InputPlatParam>& vRes);
        void PushPlatform(InputPlatParam vRes);
        std::vector<InputPlatParam>& GetPlatform();

    private:
        CalcParam vCalcParam_;
        std::vector<InputPlatParam> vServerPlatformData_;
        std::mutex mMutex_;

    public:
        CalcParamManager(const CalcParamManager&) = delete;
        CalcParamManager(CalcParamManager&&) = delete;
        CalcParamManager& operator=(const CalcParamManager&) = delete;
        CalcParamManager& operator=(CalcParamManager&&) = delete;
    };
}