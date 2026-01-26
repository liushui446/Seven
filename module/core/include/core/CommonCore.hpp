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
}

#endif