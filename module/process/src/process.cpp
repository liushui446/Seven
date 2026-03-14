#include "process/process.hpp"
#include "transformation/transformation.hpp"
#include "barrage/barrage.hpp"
#include "deception/deception.hpp"
#include "process/SimManager.hpp"

#include <thread>
#include <chrono>
// Windows 管道头文件
#include <windows.h>
#include <cstring>
#include <stdexcept>
#include <winnt.h>
//#include <signal.h>


using namespace Json;

namespace seven {

    // 管道读写缓冲区大小
    const int BUF_SIZE = 4096 * 10000;
    // JSON 数据分隔符（解决管道粘包，需确保分隔符不在 JSON 内容中）
    const std::string JSON_DELIMITER = "\n###END###\n";

    // 全局退出标志：用于控制主循环退出
    volatile bool g_is_running = true;
    

    //// 信号处理函数：捕获Ctrl+C，设置退出标志
    //void signal_handler(int sig) {
    //    if (sig == SIGINT) {
    //        std::cout << "\n接收到退出信号，准备关闭管道服务端..." << std::endl;
    //        g_is_running = false;
    //    }
    //}

    //控制DOUBLE类型精度,并转成string
    string formatDouble(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    double formatDouble2(double value, int precision) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return std::stod(stream.str()); // Convert formatted string back to double
    }

    /**
     * @brief 将 Json::Value 序列化为带分隔符的字符串（适配管道传输）
     * @param root JSON 根节点
     * @return 序列化后的字符串（含分隔符）
     */
    std::string jsonToString(const Json::Value& root) {
        Json::StreamWriterBuilder builder;
        // 美化输出（可选，传输时也可使用 FastWriter 减小体积）
        builder["indentation"] = "  ";
        std::string json_str = Json::writeString(builder, root);
        // 追加分隔符，用于识别单次传输的结束
        return json_str + JSON_DELIMITER;
    }

    /**
     * @brief 从带分隔符的字符串中解析出 Json::Value
     * @param data 管道读取的原始数据
     * @param root 输出的 JSON 根节点
     * @return 解析成功返回 true，失败返回 false
     */
    bool stringToJson(const std::string& data, Json::Value& root) {
        // 截取分隔符前的有效 JSON 数据
        size_t delimiter_pos = data.find(JSON_DELIMITER);
        if (delimiter_pos == std::string::npos) {
            std::cerr << "未找到 JSON 数据分隔符，可能粘包或数据不完整" << std::endl;
            return false;
        }
        std::string json_str = data.substr(0, delimiter_pos);

        // 解析 JSON 字符串
        Json::CharReaderBuilder builder;
        JSONCPP_STRING err;
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        if (!reader->parse(json_str.c_str(), json_str.c_str() + json_str.size(), &root, &err)) {
            std::cerr << "JSON 解析失败: " << err << std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief 从字符串中解析出 命令类型
     * @param data 读取的 JSON
     * 压制：1; 欺骗：2 ;编队：3;
     * @param root 输出的 JSON 根节点
     */
    void parser_cmd(HANDLE hPipe, Json::Value param, Json::Value& result)
    {
        //Cmd_Type type = static_cast<Cmd_Type>(param["cmd"].asInt());
        //if (type == Cmd_Type::Barrage)
        //{
        //    Barrage_Test(param, result);
        //}
        //else if (type == Cmd_Type::Deception)
        //{
        //    Deception_Test(param, result);
        //}
        //else if (type == Cmd_Type::Transformation)
        //{
        //    Transformation_Test(param, result);
        //}
        //else
        //{
        //    result["status"] = "error";  // 字符串值
        //    result["message"] = std::string("parser command fail：");  // 拼接字符串
        //    result["data"] = Json::nullValue;  // 空值（替代 nullptr，JsonCpp 专用）
        //}

        // ============== 2.28新增 仿真开始 停止 =============//
        if (param.get("cmd", 4).asInt() == 4) {
            result["status"] = "error";  // 字符串值
            result["message"] = std::string("parser command fail：");  // 拼接字符串
            result["data"] = Json::nullValue;  // 空值（替代 nullptr，JsonCpp 专用）
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
        // 校验参数合法性
        if (hPipe == INVALID_HANDLE_VALUE || hPipe == NULL) {
            std::cerr << "sendResultData: 无效的s2c管道句柄" << std::endl;
            return false;
        }

        std::string result_str = jsonToString(result); // 转为 JSON 字符串
        if (result_str.empty()) {
            std::cerr << "sendResultData: 发送数据为空" << std::endl;
            return false;
        }

        std::cerr << "仿真结果数据发送中..." << std::endl;

        DWORD bytesWritten = 0;
        // 核心写管道逻辑
        BOOL ret = WriteFile(
            hPipe,
            result_str.c_str(),
            static_cast<DWORD>(result_str.length()),
            &bytesWritten,
            NULL
        );

        std::cerr << "当前仿真结果数据发送完成！" << std::endl;

        // 错误处理和日志
        if (!ret) {
            std::cerr << "sendResultData: 写s2c管道失败，错误码：" << GetLastError() << std::endl;
            return false;
        }

        if (bytesWritten != result_str.length()) {
            std::cerr << "sendResultData: 数据发送不完整，预期发送" << result_str.length()
                << "字节，实际发送" << bytesWritten << "字节" << std::endl;
            return false;
        }

        // 可选：打印发送成功日志（调试用）
        // std::cout << "sendResultData: 成功发送 " << bytesWritten << " 字节数据" << std::endl;
        return true;
    }

    // 管道服务端：处理单个客户端连接
    void handle_client(HANDLE hPipe) {
        char buffer[4096] = { 0 };
        DWORD bytesRead = 0;

        // 持续读取客户端（仿真平台）发送的命令
        while (true) {
            // 读取管道中的命令（阻塞，直到有数据）
            BOOL ret = ReadFile(
                hPipe,
                buffer,
                sizeof(buffer) - 1,
                &bytesRead,
                NULL
            );

            if (!ret || bytesRead == 0) {
                // 客户端断开连接，退出当前处理线程
                std::cout << "客户端断开连接" << std::endl;
                break;
            }

            // 解析接收到的 JSON 命令
            buffer[bytesRead] = '\0'; // 确保字符串结束
            std::string cmd_str(buffer);
            std::cout << "收到命令：" << cmd_str << std::endl;

            try {
                Json::Value cmd_mes;
                if (!stringToJson(cmd_str, cmd_mes))
                {
                    continue;
                }

                Json::Value result;
                // 执行计算
                parser_cmd(hPipe, cmd_mes, result);
                std::string result_str = jsonToString(result); // 转为 JSON 字符串

                // 将结果写回管道（返回给仿真平台）
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
                // 构造错误结果返回给客户端
                Json::Value error_result;
                error_result["status"] = "error";  // 字符串值
                error_result["message"] = std::string("process commande fail：") + e.what();  // 拼接字符串
                error_result["data"] = Json::nullValue;  // 空值（替代 nullptr，JsonCpp 专用）

                std::string result_str = jsonToString(error_result); // 转为 JSON 字符串

                // 将结果写回管道（返回给仿真平台）
                /*DWORD bytesWritten = 0;
                WriteFile(
                    hPipe,
                    result_str.c_str(),
                    result_str.length(),
                    &bytesWritten,
                    NULL
                );*/

                sendResultData(hPipe, result_str);
            }

            // 清空缓冲区
            memset(buffer, 0, sizeof(buffer));
        }

        // 关闭管道句柄
        CloseHandle(hPipe);
    }

    // 启动管道服务端，持续监听连接
    void start_pipe_server() {
        // 管道名称（Windows 格式：\\.\pipe\管道名，需唯一）
        const std::wstring pipe_name = L"\\\\.\\pipe\\SimCalculatorPipe";

        while (true) {
            // 创建命名管道
            HANDLE hPipe = CreateNamedPipeW(
                pipe_name.c_str(),
                PIPE_ACCESS_DUPLEX,       // 双向管道（可读可写）
                PIPE_TYPE_MESSAGE |       // 消息模式（按消息边界读取）
                PIPE_READMODE_MESSAGE |
                PIPE_WAIT,                // 阻塞模式
                PIPE_UNLIMITED_INSTANCES, // 允许多个客户端连接（按需）
                4096,                  // 输出缓冲区大小
                4096,                  // 输入缓冲区大小
                0,                        // 默认超时
                NULL                      // 安全属性
            );

            if (hPipe == INVALID_HANDLE_VALUE) {
                std::cerr << "创建管道失败，错误码：" << GetLastError() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            // 等待客户端（仿真平台）连接（阻塞）
            std::cout << "等待仿真平台连接..." << std::endl;
            BOOL connected = ConnectNamedPipe(hPipe, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);

            if (connected) {
                std::cout << "仿真平台已连接" << std::endl;
                // 启动线程处理当前客户端（支持并发连接，按需）
                std::thread client_thread(handle_client, hPipe);
                client_thread.detach(); // 分离线程，无需等待
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

        // 持续读取客户端命令（直到客户端断开/程序退出）
        while (g_is_running) {
            // 读取管道中的命令（阻塞，直到有数据/出错）
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
                break; // 退出通信循环，回到等待连接阶段
            }

            // 解析接收到的 JSON 命令
            buffer[bytesRead] = '\0'; // 确保字符串结束
            std::string cmd_str(buffer);
            std::cout << "收到命令：" << cmd_str << std::endl;

            try {
                Json::Value cmd_mes;
                if (!stringToJson(cmd_str, cmd_mes)) {
                    memset(buffer, 0, sizeof(buffer));
                    continue;
                }

                Json::Value result;
                // 执行计算
                parser_cmd(hPipe_s2c, cmd_mes, result);
                std::string result_str = jsonToString(result); // 转为 JSON 字符串

                // 将结果写回管道（返回给仿真平台）
                DWORD bytesWritten = 0;
                BOOL write_ok = WriteFile(
                    hPipe,
                    result_str.c_str(),
                    result_str.length(),
                    &bytesWritten,
                    NULL
                );

                if (write_ok) {
                    FlushFileBuffers(hPipe); // 强制刷新，确保数据立即发送
                    std::cout << "回复客户端成功，写入字节数：" << bytesWritten << std::endl;
                }
                else {
                    std::cerr << "回复客户端失败，错误码：" << GetLastError() << std::endl;
                }
            }
            catch (const std::exception& e) {
                // 构造错误结果返回给客户端
                Json::Value error_result;
                error_result["status"] = "error";
                error_result["message"] = std::string("process commande fail：") + e.what();
                error_result["data"] = Json::nullValue;

                std::string result_str = jsonToString(error_result);
                sendResultData(hPipe, result_str);
            }

            // 清空缓冲区
            memset(buffer, 0, sizeof(buffer));
        }
    }

    // 启动管道服务端（单句柄版本）
    void start_pipe_server_test() {
        // 管道名称（Windows 格式：\\.\pipe\管道名）
        const std::wstring pipe_name = L"\\\\.\\pipe\\SimCalculatorPipe";
        // 定义两个命名管道的名称
        const std::wstring pipe_name_client_to_server = L"\\\\.\\pipe\\ClientToServerPipe";
        const std::wstring pipe_name_server_to_client = L"\\\\.\\pipe\\ServerToClientPipe";

        // ========== 核心修改：只创建一次管道句柄 ==========
        HANDLE hPipe = CreateNamedPipeW(
            pipe_name.c_str(),
            PIPE_ACCESS_DUPLEX,       // 双向管道（可读可写）
            PIPE_TYPE_MESSAGE |       // 消息模式（按消息边界读取）
            PIPE_READMODE_MESSAGE |
            PIPE_WAIT,                // 阻塞模式
            1,                        // 单实例（匹配单句柄复用）
            4096,                     // 输出缓冲区大小
            4096,                     // 输入缓冲区大小
            0,                        // 默认超时
            NULL                      // 安全属性
        );

        if (hPipe == INVALID_HANDLE_VALUE) {
            std::cerr << "创建管道失败，错误码：" << GetLastError() << std::endl;
            return; // 管道创建失败，直接退出
        }
        std::cout << "管道句柄创建成功，等待客户端连接..." << std::endl;

        // 主循环：复用管道句柄，持续等待客户端连接
        while (g_is_running) {
            // 等待客户端（仿真平台）连接（阻塞）
            BOOL connected = ConnectNamedPipe(hPipe, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);

            if (connected) {
                std::cout << "仿真平台已连接" << std::endl;
                // 处理客户端通信（复用当前管道句柄）
                //handle_client_communication(hPipe);

                // 客户端断开后，重置管道状态，等待下一次连接
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

        // ========== 程序退出时，关闭唯一的管道句柄 ==========
        if (hPipe != INVALID_HANDLE_VALUE) {
            CloseHandle(hPipe);
            std::cout << "管道句柄已关闭" << std::endl;
        }
    }

    void start_double_pipe_test() {
        // 定义两个命名管道的名称
        const std::wstring pipe_name_client_to_server = L"\\\\.\\pipe\\ClientToServerPipe";
        const std::wstring pipe_name_server_to_client = L"\\\\.\\pipe\\ServerToClientPipe";

        // s2c管道句柄（供其他线程访问）
        HANDLE g_hPipe_s2c = INVALID_HANDLE_VALUE;

        // ========== 1. 创建c2s管道（双向：读指令+即时回复） ==========
        HANDLE hPipe_c2s = CreateNamedPipeW(
            pipe_name_client_to_server.c_str(),
            PIPE_ACCESS_DUPLEX,               // 双向管道（服务端可读可写）
            PIPE_TYPE_MESSAGE |               // 消息模式
            PIPE_READMODE_MESSAGE |
            PIPE_WAIT,                        // 阻塞模式
            PIPE_UNLIMITED_INSTANCES, // 允许多个客户端连接（按需）
            4096,                             // 输出缓冲区
            4096,                             // 输入缓冲区
            0,                                // 默认超时
            NULL                              // 安全属性
        );

        if (hPipe_c2s == INVALID_HANDLE_VALUE) {
            std::cerr << "创建c2s管道失败，错误码：" << GetLastError() << std::endl;
            return;
        }
        std::cout << "c2s管道句柄创建成功，等待客户端连接..." << std::endl;

        // ========== 2. 创建s2c管道（服务端只写：主动推送结果） ==========
        //g_hPipe_s2c = CreateNamedPipeW(
        //    pipe_name_server_to_client.c_str(),
        //    PIPE_ACCESS_OUTBOUND,             // 服务端只写（关键：仅用于推送数据）
        //    PIPE_TYPE_MESSAGE |               // 消息模式
        //    PIPE_WAIT,                        // 阻塞模式
        //    1,                                // 单实例
        //    4096,                             // 输出缓冲区（仅这个生效）
        //    0,                                // 输入缓冲区（无需，设为0）
        //    0,                                // 默认超时
        //    NULL                              // 安全属性
        //);

        g_hPipe_s2c = CreateNamedPipeW(
            pipe_name_server_to_client.c_str(),
            PIPE_ACCESS_DUPLEX,               // 双向管道（服务端可读可写）
            PIPE_TYPE_MESSAGE |               // 消息模式
            PIPE_READMODE_MESSAGE |
            PIPE_WAIT,                        // 阻塞模式
            PIPE_UNLIMITED_INSTANCES, // 允许多个客户端连接（按需）
            4096,                             // 输出缓冲区
            4096,                             // 输入缓冲区
            0,                                // 默认超时
            NULL                              // 安全属性
        );

        if (g_hPipe_s2c == INVALID_HANDLE_VALUE) {
            std::cerr << "创建s2c管道失败，错误码：" << GetLastError() << std::endl;
            CloseHandle(hPipe_c2s);
            return;
        }
        std::cout << "s2c管道句柄创建成功，等待客户端连接..." << std::endl;

        // ========== 主循环：处理管道连接 ==========
        while (g_is_running) {
            
            // ---------- 处理c2s管道（接收指令+即时回复） ----------
            BOOL c2s_connected = ConnectNamedPipe(hPipe_c2s, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);
            if (c2s_connected) {
                std::cout << "c2s管道：客户端已连接" << std::endl;
                //handle_client_communication(hPipe_c2s, g_hPipe_s2c); // 处理指令+即时回复
                //DisconnectNamedPipe(hPipe_c2s);        // 断开，等待下一次指令
                //std::cout << "c2s管道：已断开客户端连接，等待新连接..." << std::endl;
            }
            else {
                DWORD err = GetLastError();
                if (err != ERROR_NO_DATA && err != ERROR_PIPE_CONNECTED) {
                    std::cerr << "c2s管道连接失败，错误码：" << err << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }

            // ---------- 处理s2c管道（常驻连接，供推送数据） ----------
            if (g_hPipe_s2c == INVALID_HANDLE_VALUE) {
                // 重新创建s2c管道（如果之前断开）
                g_hPipe_s2c = CreateNamedPipeW(
                    pipe_name_server_to_client.c_str(),
                    PIPE_ACCESS_DUPLEX,               // 双向管道（服务端可读可写）
                    PIPE_TYPE_MESSAGE |               // 消息模式
                    PIPE_READMODE_MESSAGE |
                    PIPE_WAIT,                        // 阻塞模式
                    PIPE_UNLIMITED_INSTANCES, // 允许多个客户端连接（按需）
                    4096,                             // 输出缓冲区
                    4096,                             // 输入缓冲区
                    0,                                // 默认超时
                    NULL                              // 安全属性
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
                // s2c管道连接后不立即断开，保持连接供其他线程推送数据
                // 断开逻辑由send_result_to_client失败时触发
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
                handle_client_communication(hPipe_c2s, g_hPipe_s2c); // 处理指令+即时回复
            }
        }

        // ========== 程序退出：清理资源 ==========
        // 关闭c2s管道
        if (hPipe_c2s != INVALID_HANDLE_VALUE) {
            DisconnectNamedPipe(hPipe_c2s);
            CloseHandle(hPipe_c2s);
            std::cout << "hPipe_c2s管道句柄已关闭" << std::endl;
        }

        // 关闭s2c管道
        //std::lock_guard<std::mutex> lock(g_s2c_pipe_mutex);
        if (g_hPipe_s2c != INVALID_HANDLE_VALUE) {
            DisconnectNamedPipe(g_hPipe_s2c);
            CloseHandle(g_hPipe_s2c);
            std::cout << "hPipe_s2c管道句柄已关闭" << std::endl;
        }

        g_is_running = false;
        std::cout << "管道服务已退出" << std::endl;
    }
}