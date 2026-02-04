#ifndef COMMONCORE_HPP
#define COMMONCORE_HPP

#include "json.h"

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <set>
#include <deque>
#include <algorithm>
#include <functional>
#include <filesystem>
#include <mutex>
#include <fstream>
#include <unordered_map>

# if (defined Windows)
#   define SEVEN_EXPORTS __declspec(dllexport)
# elif defined Linux
#   define SEVEN_EXPORTS __attribute__ ((visibility ("default")))
# endif

using std::cout;
using std::endl;
using std::map;
using std::set;
using std::pair;
using std::deque;
using std::vector;
using std::string;
using std::to_string;
using std::shared_ptr;
using std::make_shared;
using std::unique_ptr;
using std::make_unique;
using std::for_each;
using std::mutex;
using std::function;
using std::condition_variable;
using std::ifstream;
using std::ofstream;
using std::thread;
using std::dynamic_pointer_cast;
using std::tuple;

typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;
typedef double				DOUBLE;
typedef int                 INT;
typedef unsigned int        UINT;

typedef unsigned char uchar;
typedef unsigned short ushort;

#define	MARK_SUM (40)		// Mark点数

#define M_PI 3.141592653589793238462643383

namespace seven
{
    enum class Cmd_Type
    {
        Barrage = 1,             // 压制
        Deception = 2,           // 欺骗
        Transformation = 3       // 编队
    };

    // 坐标结构体定义
    struct LLA {
        double lon_deg;  // 经度(°)
        double lat_deg;  // 纬度(°)
        double h_m;     // 高度(m)

        // 【加法运算符】LLA + LLA，对应分量逐元素相加
        friend LLA operator+(const LLA& lhs, const LLA& rhs) {
            return { lhs.lon_deg + rhs.lon_deg,
                    lhs.lat_deg + rhs.lat_deg,
                    lhs.h_m + rhs.h_m };
        }

        // 【减法运算符】LLA - LLA，对应分量逐元素相减
        friend LLA operator-(const LLA& lhs, const LLA& rhs) {
            return { lhs.lon_deg - rhs.lon_deg,
                    lhs.lat_deg - rhs.lat_deg,
                    lhs.h_m - rhs.h_m };
        }

        // 【点乘运算符】LLA · LLA，返回标量值（分量相乘后求和）
        // 点乘是向量运算核心，结果为double类型
        friend LLA operator*(const LLA& lhs, const double& other) {
            double lon_deg = lhs.lon_deg * other;
            double lat_deg = lhs.lat_deg * other;
            double h_m = lhs.h_m * other;
            return { lon_deg, lat_deg, h_m };
        }

        // 拓展：自增/自减运算符（可选，工程中常用，贴合使用习惯）
        LLA& operator+=(const LLA& other) {
            *this = *this + other;
            return *this;
        }
        LLA& operator-=(const LLA& other) {
            *this = *this - other;
            return *this;
        }
        LLA& operator=(const LLA& other) {
            this->lon_deg = other.lon_deg;
            this->lat_deg = other.lat_deg;
            this->h_m = other.h_m;
            return *this;
        }
    };

    // ECEF坐标结构体
    struct ECEF {
        double X;        // X轴(m)
        double Y;        // Y轴(m)
        double Z;        // Z轴(m)
    };
}

#endif