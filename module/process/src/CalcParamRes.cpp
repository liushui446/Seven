#include "process/CalcParamRes.hpp"

namespace seven
{
    CalcParamManager &CalcParamManager::Ins()
    {
        static CalcParamManager instance;
        return instance;
    }

    CalcParamManager::CalcParamManager()
        : mMutex_(), vCalcParam_()
    {
        vServerPlatformData_.clear();
    }

    CalcParamManager::~CalcParamManager()
    {
    }

    bool CalcParamManager::ClearAllData()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.run_frames_cnt = 0;
        vServerPlatformData_.clear();
        return true;
    }

    bool CalcParamManager::SetReturnFramesCount(UINT vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.return_frames = vRes;
        return true;
    }

    bool CalcParamManager::SetRunFramesCnt(UINT vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.run_frames_cnt = vRes;
        return true;
    }

    bool CalcParamManager::SetSimTime(UINT vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vCalcParam_.sim_time_ = vRes;
        return true;
    }

    CalcParam CalcParamManager::GetCalcParam()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        return vCalcParam_;
    }

    void CalcParamManager::SwapPlatform(std::vector<InputPlatParam>& vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vServerPlatformData_.clear();
        vServerPlatformData_.swap(vRes);
    }

    void CalcParamManager::PushPlatform(InputPlatParam vRes)
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        vServerPlatformData_.push_back(vRes);
    }

    std::vector<InputPlatParam>& CalcParamManager::GetPlatform()
    {
        std::unique_lock<std::mutex> lk(mMutex_);
        return vServerPlatformData_;
    }
}