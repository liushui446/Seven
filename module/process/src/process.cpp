#include "process/process.hpp"
#include "transformation/transformation.hpp"
#include "barrage/barrage.hpp"
#include "deception/deception.hpp"
#include "process/SimManager.hpp"

#include <thread>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cerrno>

// ===================== 跨平台头文件 =====================
#ifdef _WIN32
    // Windows 命名管道头文件
    #include <windows.h>
    #include <winnt.h>
#else
    // Linux/麒麟 Unix Domain Socket 头文件
    #include <sys/socket.h>
    #include <sys/un.h>
    #include <csignal>
    #include <unistd.h>   // close, unlink
#endif

using namespace Json;

namespace seven {

    // ===================== 公共常量与变量（双平台共享）=====================
    const int BUF_SIZE = 4096 * 10000;
    const std::string JSON_DELIMITER = "\n###END###\n";
    volatile bool g_is_running = true;

#ifdef _WIN32
    // ===================== Windows 平台实现 =====================
    // （INVALID_HANDLE_VALUE, HANDLE, DWORD, BOOL 等由 <windows.h> 提供）

    // 控制DOUBLE类型精度,并转成string
    string formatDouble(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    double formatDouble2(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return std::stod(stream.str());
    }

    std::string jsonToString(const Json::Value& root) {
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::string json_str = Json::writeString(builder, root);
        return json_str + JSON_DELIMITER;
    }

    bool stringToJson(const std::string& data, Json::Value& root) {
        size_t delimiter_pos = data.find(JSON_DELIMITER);
        if (delimiter_pos == std::string::npos) {
            std::cerr << "未找到 JSON 数据分隔符，可能粘包或数据不完整" << std::endl;
            return false;
        }
        std::string json_str = data.substr(0, delimiter_pos);

        Json::CharReaderBuilder builder;
        JSONCPP_STRING err;
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        if (!reader->parse(json_str.c_str(), json_str.c_str() + json_str.size(), &root, &err)) {
            std::cerr << "JSON 解析失败: " << err << std::endl;
            return false;
        }
        return true;
    }

    void parser_cmd(HANDLE hPipe, Json::Value param, Json::Value& result)
    {
        if (param.get("cmd", 4).asInt() == 4) {
            result["status"] = "error";
            result["message"] = std::string("parser command fail：");
            result["data"] = Json::nullValue;
            return;
        }
        Sim_Type sim_state = static_cast<Sim_Type>(param.get("sim_type", 4).asInt());
        if (sim_state == Sim_Type::STOPPED)
        {
            g_sim_manager.sim_stop(result);
        }
        else if (sim_state == Sim_Type::STARTTED)
        {
            g_sim_manager.sim_start(param, result);
        }
        else if (sim_state == Sim_Type::RUNNING)
        {
            g_sim_manager.sim_calc(hPipe, param, result);
        }
        else if (sim_state == Sim_Type::ENDDING)
        {
            g_sim_manager.sim_end(result);
        }
    }

    // 供外部调用的管道数据发送接口
    bool sendResultData(HANDLE hPipe, const Json::Value& result) {
        if (hPipe == INVALID_HANDLE_VALUE || hPipe == NULL) {
            std::cerr << "sendResultData: 无效的s2c管道句柄" << std::endl;
            return false;
        }

        std::string result_str = jsonToString(result);
        if (result_str.empty()) {
            std::cerr << "sendResultData: 发送数据为空" << std::endl;
            return false;
        }

        std::cerr << "仿真结果数据发送中..." << std::endl;

        DWORD bytesWritten = 0;
        BOOL ret = WriteFile(
            hPipe,
            result_str.c_str(),
            static_cast<DWORD>(result_str.length()),
            &bytesWritten,
            NULL
        );

        std::cerr << "当前仿真结果数据发送完成！" << std::endl;

        if (!ret) {
            std::cerr << "sendResultData: 写s2c管道失败，错误码：" << GetLastError() << std::endl;
            return false;
        }

        if (bytesWritten != result_str.length()) {
            std::cerr << "sendResultData: 数据发送不完整，预期发送" << result_str.length()
                << "字节，实际发送" << bytesWritten << "字节" << std::endl;
            return false;
        }

        return true;
    }

    // 管道服务端：处理单个客户端连接
    void handle_client(HANDLE hPipe) {
        char buffer[4096] = { 0 };
        DWORD bytesRead = 0;

        while (true) {
            BOOL ret = ReadFile(
                hPipe,
                buffer,
                sizeof(buffer) - 1,
                &bytesRead,
                NULL
            );

            if (!ret || bytesRead == 0) {
                std::cout << "客户端断开连接" << std::endl;
                break;
            }

            buffer[bytesRead] = '\0';
            std::string cmd_str(buffer);
            std::cout << "收到命令：" << cmd_str << std::endl;

            try {
                Json::Value cmd_mes;
                if (!stringToJson(cmd_str, cmd_mes))
                {
                    continue;
                }

                Json::Value result;
                parser_cmd(hPipe, cmd_mes, result);
                std::string result_str = jsonToString(result);

                DWORD bytesWritten = 0;
                WriteFile(
                    hPipe,
                    result_str.c_str(),
                    result_str.length(),
                    &bytesWritten,
                    NULL
                );
            }
            catch (const std::exception& e)
            {
                Json::Value error_result;
                error_result["status"] = "error";
                error_result["message"] = std::string("process commande fail：") + e.what();
                error_result["data"] = Json::nullValue;

                std::string result_str = jsonToString(error_result);
                sendResultData(hPipe, error_result);
            }

            memset(buffer, 0, sizeof(buffer));
        }

        CloseHandle(hPipe);
    }

    // 启动管道服务端，持续监听连接
    void start_pipe_server() {
        const std::wstring pipe_name = L"\\\\.\\pipe\\SimCalculatorPipe";

        while (true) {
            HANDLE hPipe = CreateNamedPipeW(
                pipe_name.c_str(),
                PIPE_ACCESS_DUPLEX,
                PIPE_TYPE_MESSAGE |
                PIPE_READMODE_MESSAGE |
                PIPE_WAIT,
                PIPE_UNLIMITED_INSTANCES,
                4096,
                4096,
                0,
                NULL
            );

            if (hPipe == INVALID_HANDLE_VALUE) {
                std::cerr << "创建管道失败，错误码：" << GetLastError() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            std::cout << "等待仿真平台连接..." << std::endl;
            BOOL connected = ConnectNamedPipe(hPipe, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);

            if (connected) {
                std::cout << "仿真平台已连接" << std::endl;
                std::thread client_thread(handle_client, hPipe);
                client_thread.detach();
            }
            else {
                CloseHandle(hPipe);
            }
        }
    }

    // 处理单个客户端通信（复用管道句柄）
    void handle_client_communication(HANDLE hPipe, HANDLE hPipe_s2c) {
        char buffer[4096] = { 0 };
        DWORD bytesRead = 0;

        while (g_is_running) {
            BOOL ret = ReadFile(
                hPipe,
                buffer,
                sizeof(buffer) - 1,
                &bytesRead,
                NULL
            );

            if (!ret || bytesRead == 0) {
                DWORD err = GetLastError();
                if (err == ERROR_BROKEN_PIPE) {
                    std::cout << "客户端断开连接，等待新客户端连接..." << std::endl;
                }
                else {
                    std::cerr << "读取数据失败，错误码：" << err << std::endl;
                }
                break;
            }

            buffer[bytesRead] = '\0';
            std::string cmd_str(buffer);
            std::cout << "收到命令：" << cmd_str << std::endl;

            try {
                Json::Value cmd_mes;
                if (!stringToJson(cmd_str, cmd_mes)) {
                    memset(buffer, 0, sizeof(buffer));
                    continue;
                }

                Json::Value result;
                parser_cmd(hPipe_s2c, cmd_mes, result);
                std::string result_str = jsonToString(result);

                DWORD bytesWritten = 0;
                BOOL write_ok = WriteFile(
                    hPipe,
                    result_str.c_str(),
                    result_str.length(),
                    &bytesWritten,
                    NULL
                );

                if (write_ok) {
                    FlushFileBuffers(hPipe);
                    std::cout << "回复客户端成功，写入字节数：" << bytesWritten << std::endl;
                }
                else {
                    std::cerr << "回复客户端失败，错误码：" << GetLastError() << std::endl;
                }
            }
            catch (const std::exception& e) {
                Json::Value error_result;
                error_result["status"] = "error";
                error_result["message"] = std::string("process commande fail：") + e.what();
                error_result["data"] = Json::nullValue;

                std::string result_str = jsonToString(error_result);
                sendResultData(hPipe, error_result);
            }

            memset(buffer, 0, sizeof(buffer));
        }
    }

    // 启动管道服务端（单句柄版本）
    void start_pipe_server_test() {
        const std::wstring pipe_name = L"\\\\.\\pipe\\SimCalculatorPipe";
        const std::wstring pipe_name_client_to_server = L"\\\\.\\pipe\\ClientToServerPipe";
        const std::wstring pipe_name_server_to_client = L"\\\\.\\pipe\\ServerToClientPipe";

        HANDLE hPipe = CreateNamedPipeW(
            pipe_name.c_str(),
            PIPE_ACCESS_DUPLEX,
            PIPE_TYPE_MESSAGE |
            PIPE_READMODE_MESSAGE |
            PIPE_WAIT,
            1,
            4096,
            4096,
            0,
            NULL
        );

        if (hPipe == INVALID_HANDLE_VALUE) {
            std::cerr << "创建管道失败，错误码：" << GetLastError() << std::endl;
            return;
        }
        std::cout << "管道句柄创建成功，等待客户端连接..." << std::endl;

        while (g_is_running) {
            BOOL connected = ConnectNamedPipe(hPipe, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);

            if (connected) {
                std::cout << "仿真平台已连接" << std::endl;
                DisconnectNamedPipe(hPipe);
            }
            else {
                DWORD err = GetLastError();
                if (err != ERROR_NO_DATA && err != ERROR_PIPE_CONNECTED) {
                    std::cerr << "等待客户端连接失败，错误码：" << err << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        if (hPipe != INVALID_HANDLE_VALUE) {
            CloseHandle(hPipe);
            std::cout << "管道句柄已关闭" << std::endl;
        }
    }

    void start_double_pipe_test() {
        const std::wstring pipe_name_client_to_server = L"\\\\.\\pipe\\ClientToServerPipe";
        const std::wstring pipe_name_server_to_client = L"\\\\.\\pipe\\ServerToClientPipe";

        HANDLE g_hPipe_s2c = INVALID_HANDLE_VALUE;

        // ========== 1. 创建c2s管道 ==========
        HANDLE hPipe_c2s = CreateNamedPipeW(
            pipe_name_client_to_server.c_str(),
            PIPE_ACCESS_DUPLEX,
            PIPE_TYPE_MESSAGE |
            PIPE_READMODE_MESSAGE |
            PIPE_WAIT,
            PIPE_UNLIMITED_INSTANCES,
            4096,
            4096,
            0,
            NULL
        );

        if (hPipe_c2s == INVALID_HANDLE_VALUE) {
            std::cerr << "创建c2s管道失败，错误码：" << GetLastError() << std::endl;
            return;
        }
        std::cout << "c2s管道句柄创建成功，等待客户端连接..." << std::endl;

        // ========== 2. 创建s2c管道 ==========
        g_hPipe_s2c = CreateNamedPipeW(
            pipe_name_server_to_client.c_str(),
            PIPE_ACCESS_DUPLEX,
            PIPE_TYPE_MESSAGE |
            PIPE_READMODE_MESSAGE |
            PIPE_WAIT,
            PIPE_UNLIMITED_INSTANCES,
            4096,
            4096,
            0,
            NULL
        );

        if (g_hPipe_s2c == INVALID_HANDLE_VALUE) {
            std::cerr << "创建s2c管道失败，错误码：" << GetLastError() << std::endl;
            CloseHandle(hPipe_c2s);
            return;
        }
        std::cout << "s2c管道句柄创建成功，等待客户端连接..." << std::endl;

        // ========== 主循环 ==========
        while (g_is_running) {

            BOOL c2s_connected = ConnectNamedPipe(hPipe_c2s, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);
            if (c2s_connected) {
                std::cout << "c2s管道：客户端已连接" << std::endl;
            }
            else {
                DWORD err = GetLastError();
                if (err != ERROR_NO_DATA && err != ERROR_PIPE_CONNECTED) {
                    std::cerr << "c2s管道连接失败，错误码：" << err << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }

            if (g_hPipe_s2c == INVALID_HANDLE_VALUE) {
                g_hPipe_s2c = CreateNamedPipeW(
                    pipe_name_server_to_client.c_str(),
                    PIPE_ACCESS_DUPLEX,
                    PIPE_TYPE_MESSAGE |
                    PIPE_READMODE_MESSAGE |
                    PIPE_WAIT,
                    PIPE_UNLIMITED_INSTANCES,
                    4096,
                    4096,
                    0,
                    NULL
                );
                if (g_hPipe_s2c == INVALID_HANDLE_VALUE) {
                    std::cerr << "重新创建s2c管道失败，错误码：" << GetLastError() << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }

            BOOL s2c_connected = ConnectNamedPipe(g_hPipe_s2c, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);
            if (s2c_connected) {
                std::cout << "s2c管道：客户端已连接（持续推送模式）" << std::endl;
            }
            else {
                DWORD err = GetLastError();
                if (err != ERROR_NO_DATA && err != ERROR_PIPE_CONNECTED) {
                    std::cerr << "s2c管道连接失败，错误码：" << err << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    DisconnectNamedPipe(g_hPipe_s2c);
                    CloseHandle(g_hPipe_s2c);
                    g_hPipe_s2c = INVALID_HANDLE_VALUE;
                }
            }

            if (c2s_connected && s2c_connected) {
                handle_client_communication(hPipe_c2s, g_hPipe_s2c);
            }
        }

        // ========== 清理资源 ==========
        if (hPipe_c2s != INVALID_HANDLE_VALUE) {
            DisconnectNamedPipe(hPipe_c2s);
            CloseHandle(hPipe_c2s);
            std::cout << "hPipe_c2s管道句柄已关闭" << std::endl;
        }

        if (g_hPipe_s2c != INVALID_HANDLE_VALUE) {
            DisconnectNamedPipe(g_hPipe_s2c);
            CloseHandle(g_hPipe_s2c);
            std::cout << "hPipe_s2c管道句柄已关闭" << std::endl;
        }

        g_is_running = false;
        std::cout << "管道服务已退出" << std::endl;
    }

#else
    // ===================== Linux / 麒麟V10 平台实现 =====================
    // （使用 Unix Domain Socket 替代 Windows 命名管道）

    #define TRUE 1
    #define FALSE 0

    inline DWORD GetLastError() { return errno; }

    // 双 Socket 路径（对应客户端管道名称）
    const char* SOCKET_CMD_PATH  = "/tmp/ClientToServerPipe";
    const char* SOCKET_DATA_PATH = "/tmp/ServerToClientPipe";

    string formatDouble(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    double formatDouble2(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return std::stod(stream.str());
    }

    std::string jsonToString(const Json::Value& root) {
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::string json_str = Json::writeString(builder, root);
        return json_str + JSON_DELIMITER;
    }

    bool stringToJson(const std::string& data, Json::Value& root) {
        size_t delimiter_pos = data.find(JSON_DELIMITER);
        if (delimiter_pos == std::string::npos) {
            std::cerr << "未找到 JSON 数据分隔符" << std::endl;
            return false;
        }
        std::string json_str = data.substr(0, delimiter_pos);

        Json::CharReaderBuilder builder;
        JSONCPP_STRING err;
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        if (!reader->parse(json_str.c_str(), json_str.c_str() + json_str.size(), &root, &err)) {
            std::cerr << "JSON 解析失败: " << err << std::endl;
            return false;
        }
        return true;
    }

    void parser_cmd(HANDLE hPipe, Json::Value param, Json::Value& result)
    {
        if (param.get("cmd", 4).asInt() == 4) {
            result["status"] = "error";
            result["message"] = "parser command fail";
            result["data"] = Json::nullValue;
            return;
        }
        Sim_Type sim_state = static_cast<Sim_Type>(param.get("sim_type", 4).asInt());
        if (sim_state == Sim_Type::STOPPED) {
            g_sim_manager.sim_stop(result);
        } else if (sim_state == Sim_Type::STARTTED) {
            g_sim_manager.sim_start(param, result);
        } else if (sim_state == Sim_Type::RUNNING) {
            g_sim_manager.sim_calc(hPipe, param, result);
        } else if (sim_state == Sim_Type::ENDDING) {
            g_sim_manager.sim_end(result);
        }
    }

    // 发送数据（通过 Socket）
    bool sendResultData(HANDLE fd, const Json::Value& result) {
        if (fd == INVALID_HANDLE_VALUE) {
            std::cerr << "sendResultData: 无效句柄" << std::endl;
            return false;
        }

        std::string result_str = jsonToString(result);
        if (result_str.empty()) {
            std::cerr << "sendResultData: 发送数据为空" << std::endl;
            return false;
        }

        ssize_t ret = send(fd, result_str.c_str(), result_str.size(), MSG_NOSIGNAL);
        if (ret <= 0) {
            std::cerr << "sendResultData: 发送失败" << std::endl;
            return false;
        }
        return true;
    }

    // 创建 UNIX Socket 服务端
    int create_unix_socket_server(const char* path) {
        unlink(path);

        int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (server_fd < 0) {
            std::cerr << "socket 创建失败: " << path << std::endl;
            return -1;
        }

        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        strcpy(addr.sun_path, path);

        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "bind 失败: " << path << std::endl;
            close(server_fd);
            return -1;
        }

        if (listen(server_fd, 5) < 0) {
            std::cerr << "listen 失败: " << path << std::endl;
            close(server_fd);
            return -1;
        }

        std::cout << "Socket 服务已启动: " << path << std::endl;
        return server_fd;
    }

    // 等待客户端连接
    HANDLE accept_client(HANDLE server_fd) {
        int client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd >= 0) {
            std::cout << "客户端已连接" << std::endl;
        }
        return client_fd;
    }

    void handle_client(HANDLE hPipe) {
        char buffer[4096] = {0};
        while (g_is_running) {
            ssize_t bytesRead = recv(hPipe, buffer, sizeof(buffer)-1, MSG_NOSIGNAL);
            if (bytesRead <= 0) {
                std::cout << "客户端断开" << std::endl;
                break;
            }

            buffer[bytesRead] = '\0';
            std::string cmd_str(buffer);
            std::cout << "收到命令：" << cmd_str << std::endl;

            try {
                Json::Value cmd_mes;
                if (!stringToJson(cmd_str, cmd_mes)) continue;

                Json::Value result;
                parser_cmd(hPipe, cmd_mes, result);
                std::string result_str = jsonToString(result);
                send(hPipe, result_str.c_str(), result_str.size(), MSG_NOSIGNAL);
            } catch (const std::exception& e) {
                Json::Value error_result;
                error_result["status"] = "error";
                error_result["message"] = std::string("process commande fail：") + e.what();
                error_result["data"] = Json::nullValue;
                sendResultData(hPipe, error_result);
            }
            memset(buffer, 0, sizeof(buffer));
        }
        close(hPipe);
    }

    // 处理命令通道
    void handle_client_communication(HANDLE cmd_fd, HANDLE data_fd) {
        char buffer[4096] = {0};

        while (g_is_running) {
            ssize_t n = recv(cmd_fd, buffer, sizeof(buffer)-1, MSG_NOSIGNAL);
            if (n <= 0) {
                std::cout << "命令通道断开" << std::endl;
                break;
            }

            buffer[n] = '\0';
            std::string cmd_str(buffer);
            std::cout << "收到命令: " << cmd_str << std::endl;

            try {
                Json::Value cmd, res;
                if (!stringToJson(cmd_str, cmd)) continue;

                parser_cmd(data_fd, cmd, res);
                std::string resp = jsonToString(res);
                send(cmd_fd, resp.c_str(), resp.size(), MSG_NOSIGNAL);
            }
            catch (const std::exception& e) {
                Json::Value err;
                err["status"] = "error";
                err["message"] = "异常: " + std::string(e.what());
                err["data"] = Json::nullValue;
                sendResultData(cmd_fd, err);
            }
            memset(buffer, 0, sizeof(buffer));
        }
    }

    void start_pipe_server() {
        int server_fd = create_unix_socket_server("/tmp/SimCalculatorPipe");
        if (server_fd < 0) return;

        while (g_is_running) {
            std::cout << "等待仿真平台连接..." << std::endl;
            HANDLE hPipe = accept_client(server_fd);
            if (hPipe >= 0) {
                std::cout << "仿真平台已连接" << std::endl;
                std::thread client_thread(handle_client, hPipe);
                client_thread.detach();
            }
        }
        close(server_fd);
        unlink("/tmp/SimCalculatorPipe");
    }

    void start_pipe_server_test() {
        start_pipe_server();
    }

    // 双 UNIX Socket 服务端
    void start_double_pipe_test() {
        int server_cmd  = create_unix_socket_server(SOCKET_CMD_PATH);
        int server_data = create_unix_socket_server(SOCKET_DATA_PATH);

        if (server_cmd < 0 || server_data < 0) {
            std::cerr << "Socket 启动失败" << std::endl;
            return;
        }

        int cmd_fd  = INVALID_HANDLE_VALUE;
        int data_fd = INVALID_HANDLE_VALUE;

        while (g_is_running) {
            if (cmd_fd <= 0) {
                std::cout << "等待命令客户端连接... " << SOCKET_CMD_PATH << std::endl;
                cmd_fd = accept_client(server_cmd);
            }

            if (data_fd <= 0) {
                std::cout << "等待数据客户端连接... " << SOCKET_DATA_PATH << std::endl;
                data_fd = accept_client(server_data);
            }

            if (cmd_fd > 0 && data_fd > 0) {
                handle_client_communication(cmd_fd, data_fd);

                close(cmd_fd);
                close(data_fd);
                cmd_fd = INVALID_HANDLE_VALUE;
                data_fd = INVALID_HANDLE_VALUE;
                std::cout << "等待重连..." << std::endl;
            }
        }

        close(server_cmd);
        close(server_data);
        unlink(SOCKET_CMD_PATH);
        unlink(SOCKET_DATA_PATH);

        std::cout << "Socket 服务已退出" << std::endl;
    }

#endif

}
