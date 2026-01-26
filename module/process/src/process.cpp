#include "process/process.hpp"
#include "transformation/transformation.hpp"

#include <thread>
#include <chrono>
// Windows 管道头文件
#include <windows.h>
#include <cstring>
#include <stdexcept>

using namespace Json;

namespace seven {

    // 管道读写缓冲区大小
    const int BUF_SIZE = 4096;
    // JSON 数据分隔符（解决管道粘包，需确保分隔符不在 JSON 内容中）
    const std::string JSON_DELIMITER = "\n###END###\n";

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

    void parser_cmd(Json::Value param)
    {
        Cmd_Type type = static_cast<Cmd_Type>(param["num_uavs"].asInt());
        if (type == Cmd_Type::Barrage)
        {

        }
        else if (type == Cmd_Type::Deception)
        {


        }
        else if (type == Cmd_Type::Transformation)
        {

        }
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

                //json result = calculate(cmd); // 执行计算
                Transformation_Test(cmd_mes);

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
                error_result["message"] = std::string("处理命令失败：") + e.what();  // 拼接字符串
                error_result["data"] = Json::nullValue;  // 空值（替代 nullptr，JsonCpp 专用）

                std::string result_str = jsonToString(error_result); // 转为 JSON 字符串

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
                4096,                     // 输出缓冲区大小
                4096,                     // 输入缓冲区大小
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
}