#include "process/CalcThread.hpp"
#include "barrage/barrage.hpp"
#include <deception/deception.hpp>
#include <transformation/transformation.hpp>
#include <process/process.hpp>
#include "process/CalcParamRes.hpp"

// ===================== 跨平台头文件 =====================
#ifdef _WIN32
    #include <windows.h>
    #include <processthreadsapi.h>
    #include <timeapi.h>
    #include <winnt.h>
    #pragma comment(lib, "winmm.lib")
#else
    #include <pthread.h>
#endif

namespace seven
{
	// 静态成员变量初始化
	std::shared_ptr<CalcProcessThread> CalcProcessThread::instance_ = nullptr;
	std::once_flag CalcProcessThread::flag_;

	static std::vector<std::shared_ptr<CalcTaskParam>> g_task_queue;
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
			DORMANT = 0,
			READY = 1,
			BUSY = 2,
			INTERRUPTED = 3,
			QUIT = 4,
		};

		std::atomic<int> iThreadNum_;
		std::vector<std::thread*> pThreads_;
		AtomicIntArray ThdStats_;
		std::atomic<bool> bStartWork_;
		std::vector<bool> vecInterruptRequest_;

		Pimple()
			: iThreadNum_(GetNumThread())
			, pThreads_(iThreadNum_, nullptr)
			, ThdStats_(iThreadNum_)
			, bStartWork_(false)
			, vecInterruptRequest_(iThreadNum_, false)
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
	{
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
		for (size_t i = 0; i < pMem_->iThreadNum_; ++i)
		{
			if (pMem_->pThreads_[i] != nullptr)
			{
				continue;
			}
			pMem_->pThreads_[i] = new std::thread(std::bind(&CalcProcessThread::ThreadFunc, this, i));

			// ===================== 跨平台线程命名 =====================
#ifdef _WIN32
			SetThreadDescription(pMem_->pThreads_[i]->native_handle(), L"ImageProcessThread_" + i);
#else
			// Linux 设置线程名（最多 15 个字符）
			std::string name = "CalProcessThread_" + std::to_string(i);
			pthread_setname_np(pMem_->pThreads_[i]->native_handle(), name.c_str());
#endif
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
		for (size_t i = 0; i < pMem_->iThreadNum_; ++i)
		{
			if (pMem_->pThreads_[i] == nullptr)
			{
				continue;
			}
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

	bool CalcProcessThread::SerSwitchTaskParam(int formation_id, int custom_id, Formation_Type input) {
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		bool flag = SwitchFormation(formation_id, custom_id, input);
		return flag;
	}

	bool CalcProcessThread::SerTurnTaskParam(int formation_id, double input)
	{
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		TurnFormation(formation_id, input);
		return true;
	}

	bool CalcProcessThread::SetAddNodeTaskParam(int formation_id, vector<UUVNode>& input)
	{
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		AddNode(formation_id, input);
		return true;
	}

	bool CalcProcessThread::SetRemoveNodeTaskParam(int formation_id, int num)
	{
		std::lock_guard<std::mutex> lk(g_task_mutex);
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}
		RemoveLastNode(formation_id, num);
		return true;
	}

	bool CalcProcessThread::SubmitTask(HANDLE hPipe, const Json::Value& input, Json::Value& output) {
		if (!pMem_->bStartWork_.load(std::memory_order_acquire)) {
			return false;
		}

		int idle_thread = -1;
		for (int i = 0; i < pMem_->iThreadNum_; ++i) {
			if (pMem_->ThdStats_[i].GetValue() == static_cast<int>(Pimple::ThreadStatus::DORMANT)) {
				idle_thread = i;
				break;
			}
		}

		if (idle_thread == -1) {
			return false;
		}

		auto task_param = std::make_shared<CalcTaskParam>();
		task_param->hPipe = hPipe;
		task_param->max_frames = CalcParamManager::Ins().GetCalcParam().sim_time_;
		task_param->run_frames = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
		task_param->return_frames = CalcParamManager::Ins().GetCalcParam().return_frames;
		task_param->serveral_plat = CalcParamManager::Ins().GetPlatform();
		task_param->input = input;
		task_param->task_finished = false;

		{
			std::lock_guard<std::mutex> lk(g_task_mutex);
			g_task_queue.push_back(task_param);
		}

		if (WakeUpAThread(idle_thread)) {
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
#ifdef _WIN32
			// Windows: 提高定时器精度
			timeBeginPeriod(1);
			std::this_thread::sleep_for(std::chrono::microseconds(1));
			timeEndPeriod(1);
#else
			// Linux: 直接睡眠
			std::this_thread::sleep_for(std::chrono::microseconds(1));
#endif
		} while ((!pMem_->ThdStats_[noThread].CAS(threadState, static_cast<int>(Pimple::ThreadStatus::READY))) &&
			(threadState == static_cast<int>(Pimple::ThreadStatus::DORMANT)));

		std::unique_lock<std::mutex> lk(m_mut);
		m_con_var.notify_all();

		return true;
	}

	bool CalcProcessThread::Interrupted()
	{
		for (int noThread = 0; noThread < pMem_->pThreads_.size(); ++noThread)
		{
			if (pMem_->pThreads_[noThread] == nullptr)
			{
				continue;
			}

			int threadState = pMem_->ThdStats_[noThread].GetValue();
			if (threadState != static_cast<int>(Pimple::ThreadStatus::BUSY))
			{
				std::cerr << "Interrupted: 线程" << noThread << "非BUSY状态，当前状态：" << threadState << std::endl;
				return false;
			}

			pMem_->vecInterruptRequest_[noThread] = true;
			pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::INTERRUPTED));

			std::cout << "已发起线程" << noThread << "中断请求" << std::endl;
		}

		return true;
	}

	void CalcProcessThread::ResetInterruptFlag(int noThread)
	{
		if (noThread >= 0 && noThread < pMem_->iThreadNum_) {
			pMem_->vecInterruptRequest_[noThread] = false;
		}
	}

	bool CalcProcessThread::IsInterrupted(int noThread)
	{
		if (noThread < 0 || noThread >= pMem_->iThreadNum_) {
			return false;
		}
		return (pMem_->ThdStats_[noThread].GetValue() == static_cast<int>(Pimple::ThreadStatus::INTERRUPTED)) || pMem_->vecInterruptRequest_[noThread];
	}

	void CalcProcessThread::ThreadFunc(int noThread)
	{
		pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));

		while (true)
		{
			std::unique_lock<std::mutex> lk(m_mut);
			m_con_var.wait(lk, [&]()
				{
					int status = pMem_->ThdStats_[noThread].GetValue();
					if (status != static_cast<int>(Pimple::ThreadStatus::QUIT)) {
						if (status != static_cast<int>(Pimple::ThreadStatus::READY)) {
							pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
						}
					}
					return status == static_cast<int>(Pimple::ThreadStatus::QUIT) ||
						status == static_cast<int>(Pimple::ThreadStatus::READY);
				});

			int current_status = pMem_->ThdStats_[noThread].GetValue();
			if (current_status == static_cast<int>(Pimple::ThreadStatus::QUIT))
			{
				break;
			}

			pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::BUSY));
			lk.unlock();

				auto start = std::chrono::high_resolution_clock::now();

				std::shared_ptr<CalcTaskParam> task_param;
				{
					std::lock_guard<std::mutex> task_lk(g_task_mutex);
					if (!g_task_queue.empty()) {
						task_param = g_task_queue.front();
						g_task_queue.erase(g_task_queue.begin());
					}
				}

				CalcTempParam TempParam;
				TempParam.input = task_param->input;
				TempParam.max_frames.store(task_param->max_frames);
				TempParam.run_frames.store(task_param->run_frames);
				TempParam.return_frames.store(task_param->return_frames);
				TempParam.serveral_plat = task_param->serveral_plat;

				if (task_param) {
					CalcParamManager::Ins().GetCalcParam();
					int cmd_int = task_param->input.get("cmd", 4).asInt();
					Cmd_Type type = static_cast<Cmd_Type>(cmd_int);
					if (type == Cmd_Type::Barrage)
					{
						SimConfig barrage_config = ContextManager::Ins().GetBarrageParams();
						while (true) {
							UINT run_frames_cnt_ = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
							if (run_frames_cnt_ >= (task_param->max_frames - 1)) {
								break;
							}

							bool is_interrupted = IsInterrupted(noThread);
							if (is_interrupted) {
								ResetInterruptFlag(noThread);
								pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
								break;
							}
							Barrage_Test_1(TempParam, barrage_config);
							CalcParamManager::Ins().SetRunFramesCnt(TempParam.run_frames);
							sendResultData(task_param->hPipe, TempParam.trajectory_result);
						}
					}
					else if (type == Cmd_Type::Deception)
					{
						SimParams deception_config = ContextManager::Ins().GetDeceptionParams();
						while (true) {
							UINT run_frames_cnt_ = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;
							if (run_frames_cnt_ >= (task_param->max_frames - 1)) {
								break;
							}

							bool is_interrupted = IsInterrupted(noThread);
							if (is_interrupted) {
								ResetInterruptFlag(noThread);
								pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
								break;
							}

							Deception_Use(TempParam, deception_config);
							CalcParamManager::Ins().SetRunFramesCnt(TempParam.run_frames);
							sendResultData(task_param->hPipe, TempParam.trajectory_result);
						}
					}
					else if (type == Cmd_Type::Transformation)
					{
						while (true) {
							UINT run_frames_cnt_ = CalcParamManager::Ins().GetCalcParam().run_frames_cnt;

							bool is_interrupted = IsInterrupted(noThread);
							if (is_interrupted) {
								ResetInterruptFlag(noThread);
								pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
								break;
							}
							Transformation_Use(TempParam);
							CalcParamManager::Ins().SetRunFramesCnt(TempParam.run_frames);
							sendResultData(task_param->hPipe, TempParam.trajectory_result);
						}
					}

					task_param->task_finished = true;
					g_task_cv.notify_all();
				}

				auto stop = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

				std::string str = "CalcProcessThread::ThreadFunc(), Barrage_Test_1 Time Cost: [" +
					std::to_string(duration) + "]ms";

			if (pMem_->ThdStats_[noThread].GetValue() != static_cast<int>(Pimple::ThreadStatus::QUIT) &&
				pMem_->ThdStats_[noThread].GetValue() != static_cast<int>(Pimple::ThreadStatus::INTERRUPTED)) {
				pMem_->ThdStats_[noThread].SetValue(static_cast<int>(Pimple::ThreadStatus::DORMANT));
			}
		}
	}
}
