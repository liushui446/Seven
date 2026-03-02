#include <windows.h>
#include <processthreadsapi.h>
#include <timeapi.h>
#include <winnt.h>

#include "process/CalcThread.hpp"
#include "barrage/barrage.hpp"

#pragma comment(lib, "winmm.lib") 

namespace seven
{
	// 静态成员变量初始化
	std::shared_ptr<CalcProcessThread> CalcProcessThread::instance_ = nullptr;    // 静态成员变量，保存单例实例  
	std::once_flag CalcProcessThread::flag_;					// 静态成员变量，用于同步访问 

	// 新增：线程计算任务的参数结构体
	struct CalcTaskParam {
		HANDLE hPipe = nullptr;                  // 管道句柄
		Json::Value input;                       // 输入JSON数据
		Json::Value trajectory_result;           // 输出结果
		std::atomic<bool> task_finished{ false };  // 任务是否完成
	};

	static std::vector<std::shared_ptr<CalcTaskParam>> g_task_queue;
	// 全局任务队列（线程安全）
	static std::mutex g_task_mutex;
	static std::condition_variable g_task_cv;

	static size_t GetNumThread()
	{
		return 1;
	}

	struct CalcProcessThread::Pimple
	{
	public:
		enum class ThreadStatus
		{
			UNINIT = -1,
			DORMANT = 0,	 // 休眠
			READY = 1,		 // 准备
			BUSY = 2,		 // 忙碌
			INTERRUPTED = 3, // 打断
			QUIT = 4,		 // 退出
		};

		std::atomic<int> iThreadNum_;			  // 线程数
		std::vector<std::thread*> pThreads_;     // 线程指针
		AtomicIntArray ThdStats_;		  // 各个线程的状态
		std::atomic<bool> bStartWork_;			  // 是否线程工作

		Pimple()
			: iThreadNum_(GetNumThread())
			, pThreads_(iThreadNum_, nullptr)
			, ThdStats_(iThreadNum_)
			, bStartWork_(false)
		{
			for (int i = 0; i < iThreadNum_; ++i)
			{
				ThdStats_[i].SetValue(static_cast<int>(ThreadStatus::UNINIT));
			}
		}

		~Pimple()
		{
		}
	};

	CalcProcessThread::CalcProcessThread()
		: pMem_(std::make_shared<Pimple>())
		, vFinishFovNum_(pMem_->iThreadNum_)
	{
		for (int i = 0; i < vFinishFovNum_.size(); ++i)
		{
			vFinishFovNum_[i].SetValue(0);
		}
		this->Init();
	}

	CalcProcessThread::~CalcProcessThread()
	{
		UnInit();

		for (auto itr = pMem_->pThreads_.begin(); itr != pMem_->pThreads_.end(); ++itr)
		{
			if ((*itr) != nullptr)
			{
				delete (*itr);
				*itr = nullptr;
			}
		}
	}

	std::shared_ptr<CalcProcessThread> CalcProcessThread::GetInstance()
	{
		std::call_once(flag_, [&]() {
			instance_.reset(new CalcProcessThread());
			});
		return instance_;
	}

	void CalcProcessThread::Init()
	{
		for (size_t i = 0; i < pMem_->iThreadNum_; i++)
		{
			if (pMem_->pThreads_[i] != nullptr)
			{
				continue;
			}
			pMem_->pThreads_[i] = new std::thread(std::bind(&CalcProcessThread::ThreadFunc, this, i));
			SetThreadDescription(pMem_->pThreads_[i]->native_handle(), L"ImageProcessThread_" + i);
		}
		return;
	}

	void CalcProcessThread::UnInit()
	{
		for (int noThread = 0; noThread < pMem_->pThreads_.size(); ++noThread)
		{
			if (pMem_->pThreads_[noThread] == nullptr)
			{
				continue;
			}

			{
				std::unique_lock<std::mutex> lk(m_mut);
				pMem_->ThdStats_[noThread].SetValue(static_cast<int>(CalcProcessThread::Pimple::ThreadStatus::QUIT));
				m_con_var.notify_all();
			}

			if (pMem_->pThreads_[noThread]->joinable())
			{
				pMem_->pThreads_[noThread]->join();
			}
		}
		return;
	}

	void CalcProcessThread::ReloadAllThread()
	{
		for (size_t i = 0; i < pMem_->iThreadNum_; i++)
		{
			if (pMem_->pThreads_[i] == nullptr)
			{
				continue;
			}

			vFinishFovNum_[i].SetValue(0);
		}
		return;
	}

	unsigned int CalcProcessThread::GetThreadNum()
	{
		return pMem_->iThreadNum_;
	}

	void CalcProcessThread::StartWork(bool work)
	{
		pMem_->bStartWork_.store(work, std::memory_order_release);
	}

	// 新增：提交计算任务并唤醒线程
	bool CalcProcessThread::SubmitTask(HANDLE hPipe, const Json::Value& input, Json::Value& output) {
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}

		// 1. 查找空闲线程
		int idle_thread = -1;
		for (int i = 0; i < pMem_->iThreadNum_; ++i) {
			if (pMem_->ThdStats_[i].GetValue() == static_cast<int>(Pimple::ThreadStatus::DORMANT)) {
				idle_thread = i;
				break;
			}
		}

		if (idle_thread == -1) {
			return false; // 无空闲线程
		}

		// 2. 创建任务参数
		auto task_param = std::make_shared<CalcTaskParam>();
		task_param->hPipe = hPipe;
		task_param->input = input;
		task_param->task_finished = false;

		// 3. 将任务加入队列
		{
			std::lock_guard<std::mutex> lk(g_task_mutex);
			g_task_queue.push_back(task_param);
		}

		// 4. 唤醒指定空闲线程
		if (WakeUpAThread(idle_thread)) {
			// 等待任务完成
			std::unique_lock<std::mutex> lk(g_task_mutex);
			g_task_cv.wait(lk, [&]() {
				return task_param->task_finished.load(std::memory_order_acquire);
				});
			output = task_param->trajectory_result;
			return true;
		}

		return false;
	}

	bool CalcProcessThread::WakeUpAThread(int noThread)
	{
		if (!pMem_->bStartWork_.load(std::memory_order_acquire))
		{
			return false;
		}

		int threadState = pMem_->ThdStats_[noThread].GetValue();
		if (threadState != static_cast<int>(Pimple::ThreadStatus::DORMANT))
		{
			return false;
		}

		int try_times = 0;
		do
		{
			try_times++;
			if (try_times > 10)
			{
				return false;
			}
			timeBeginPeriod(1);
			std::this_thread::sleep_for(std::chrono::microseconds(1));
			timeEndPeriod(1);
		} while ((!pMem_->ThdStats_[noThread].CAS(threadState, static_cast<int>(Pimple::ThreadStatus::READY))) &&
			(threadState == static_cast<int>(Pimple::ThreadStatus::DORMANT)));

		std::unique_lock<std::mutex> lk(m_mut);
		m_con_var.notify_all();

		return true;
	}

	bool CalcProcessThread::Interrupted(int noThread)
	{
		int threadState = pMem_->ThdStats_[noThread].GetValue();
		if (threadState != static_cast<int>(Pimple::ThreadStatus::BUSY))
		{
			return false;
		}

		pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::INTERRUPTED));

		return true;
	}

	bool CalcProcessThread::WaitForSingleThreadFinish(unsigned int _noThread, unsigned int _numFovs, int _dwMilliseconds)
	{
		try
		{
			int Millis = 0;
			while (true)
			{
				Millis++;
				if (pMem_->ThdStats_[_noThread].GetValue() == static_cast<int>(Pimple::ThreadStatus::DORMANT)
					&& vFinishFovNum_[_noThread].GetValue() >= _numFovs)
				{
					break;
				}
				std::this_thread::sleep_for(std::chrono::microseconds(1));

				// 增加超时判断
				if (Millis >= _dwMilliseconds && _dwMilliseconds > 0) {
					return false;
				}
			}
		}
		catch (std::exception& e)
		{
			return false;
		}

		return true;
	}

	void CalcProcessThread::ThreadFunc(int noThread)
	{
		// 初始化线程状态为休眠
		pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));

		while (true)
		{
			std::unique_lock<std::mutex> lk(m_mut);
			// 等待唤醒条件：线程退出 或 线程就绪
			m_con_var.wait(lk, [&]()
				{
					int status = pMem_->ThdStats_[noThread].GetValue();
					// 如果不是退出状态，重置为休眠状态
					if (status != static_cast<int>(Pimple::ThreadStatus::QUIT)) {
						if (status != static_cast<int>(Pimple::ThreadStatus::READY)) {
							pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
						}
					}
					// 唤醒条件：退出 或 就绪
					return status == static_cast<int>(Pimple::ThreadStatus::QUIT) ||
						status == static_cast<int>(Pimple::ThreadStatus::READY);
				});

			// 判断是否是退出线程的命令
			int current_status = pMem_->ThdStats_[noThread].GetValue();
			if (current_status == static_cast<int>(Pimple::ThreadStatus::QUIT))
			{
				break;
			}

			// 线程进入忙碌状态
			pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::BUSY));
			lk.unlock(); // 释放锁，避免阻塞其他操作

			try
			{
				auto start = std::chrono::high_resolution_clock::now();

				// ========== 核心工作区：执行Barrage_Test_1计算任务 ==========
				std::shared_ptr<CalcTaskParam> task_param;
				{
					std::lock_guard<std::mutex> task_lk(g_task_mutex);
					if (!g_task_queue.empty()) {
						task_param = g_task_queue.front();
						g_task_queue.erase(g_task_queue.begin());
					}
				}

				if (task_param) {
					// 执行核心计算
					Barrage_Test_1(task_param->input, task_param->trajectory_result);

					// 标记任务完成
					task_param->task_finished = true;
					g_task_cv.notify_all();
				}

				// 并行计算示例（保留原框架）
				int num = 3;
				concurrency::parallel_for(0, num, [&](size_t index)
					{
						// 可添加并行辅助计算逻辑
					});

				auto stop = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

				std::string str = "CalcProcessThread::ThreadFunc(), Barrage_Test_1 Time Cost: [" +
					std::to_string(duration) + "]ms";

				// ========== 工作完成处理 ==========
				// 完成的Fov数量+1
				vFinishFovNum_[noThread].SetValue(vFinishFovNum_[noThread].GetValue() + 1);

				stop = std::chrono::high_resolution_clock::now();
				duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

				str = "CalcProcessThread::ThreadFunc() finish. Fov ID: " + std::to_string(noThread) +
					", Total Time Cost: [" + std::to_string(duration) + "]ms";
			}
			catch (const std::exception& e)
			{
				// 异常处理
				std::string err_str = "CalcProcessThread::ThreadFunc() exception: " + std::string(e.what());
			}

			// 重置线程状态为休眠
			if (pMem_->ThdStats_[noThread].GetValue() != static_cast<int>(Pimple::ThreadStatus::QUIT) &&
				pMem_->ThdStats_[noThread].GetValue() != static_cast<int>(Pimple::ThreadStatus::INTERRUPTED)) {
				pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
			}
		}
	}
}
