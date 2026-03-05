#ifndef PROCESS_HPP
#define PROCESS_HPP

#include "core/CommonCore.hpp"
//#include "process/SimManager.hpp"
//#include "json/json.h"
#include <windows.h>

namespace seven {

	// 新增：线程计算任务的参数结构体
	struct CalcTaskParam {
		HANDLE hPipe = nullptr;                  // 管道句柄
		Json::Value input;                       // 输入JSON数据
		Json::Value trajectory_result;           // 输出结果
		std::atomic<int> max_frames;			 // 运行最大帧数
		std::atomic<int> run_frames;			 // 每次运行帧数
		std::atomic<int> return_frames;			 // 每次运行帧数
		std::atomic<bool> task_finished{ false };  // 任务是否完成
		vector<InputPlatParam> serveral_plat;
	};

	string formatDouble(double value, int precision);

	double formatDouble2(double value, int precision);

	string jsonToString(const Json::Value& root);

	bool stringToJson(const std::string& data, Json::Value& root);

	void handle_client(HANDLE hPipe);
	void handle_client_communication(HANDLE hPipe, HANDLE hPipe_s2c);

	bool SEVEN_EXPORTS sendResultData(HANDLE hPipe, const Json::Value& result);

	void SEVEN_EXPORTS start_pipe_server();
	void SEVEN_EXPORTS start_pipe_server_test();
	void SEVEN_EXPORTS start_double_pipe_test();

}

#endif
