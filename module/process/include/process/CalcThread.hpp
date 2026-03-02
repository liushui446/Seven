#pragma once

#include <ppl.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <windows.h>
#include <winnt.h>

#include "core/CommonCore.hpp"
#include "process/AtomicArray.hpp"

namespace seven
{
	class SEVEN_EXPORTS CalcProcessThread
	{
	public:
		CalcProcessThread();
		~CalcProcessThread();

	public:
		static std::shared_ptr<CalcProcessThread> GetInstance();

	public:
		void Init();
		void UnInit();
		void ReloadAllThread();
		unsigned int GetThreadNum();
		void StartWork(bool work);
		bool SubmitTask(HANDLE hPipe, const Json::Value& input, Json::Value& output);
		bool WakeUpAThread(int noThread);
		bool Interrupted(int noThread);
		bool WaitForSingleThreadFinish(unsigned int _noThread, unsigned int _numFovs = 0, int _dwMilliseconds = 10000);

	private:
		void ThreadFunc(int noThread);
		
	private:
		struct Pimple;
		std::shared_ptr<Pimple> pMem_;
		std::condition_variable m_con_var;
		std::mutex m_mut;
		AtomicIntArray vFinishFovNum_;

	private:
		static std::once_flag flag_;
		static std::shared_ptr<CalcProcessThread> instance_;	// 静态成员变量，保存单例实例  

	public:
		CalcProcessThread(const CalcProcessThread&) = delete;
		CalcProcessThread(CalcProcessThread&&) = delete;
		CalcProcessThread& operator=(const CalcProcessThread&) = delete;
		CalcProcessThread& operator=(CalcProcessThread&&) = delete;
	};
}