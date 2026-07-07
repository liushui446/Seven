#ifndef PROCESS_HPP
#define PROCESS_HPP

#include "core/CommonCore.hpp"
#include <windows.h>

namespace seven {

	struct CalcTaskParam {
		HANDLE hPipe = nullptr;
		Json::Value input;
		Json::Value trajectory_result;
		std::atomic<int> max_frames;
		std::atomic<int> run_frames;
		std::atomic<int> return_frames;
		std::atomic<bool> task_finished{ false };
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
