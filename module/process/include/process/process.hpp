#ifndef PROCESS_HPP
#define PROCESS_HPP

#include "core/CommonCore.hpp"
//#include "process/SimManager.hpp"
//#include "json/json.h"
#include <windows.h>

namespace seven {

	string formatDouble(double value, int precision);

	double formatDouble2(double value, int precision);

	string jsonToString(const Json::Value& root);

	bool stringToJson(const std::string& data, Json::Value& root);

	void handle_client(HANDLE hPipe);

	void SEVEN_EXPORTS start_pipe_server();

}

#endif
