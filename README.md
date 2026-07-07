# seven_merge — 跨平台统一工程

本项目由 `seven_win`（Windows 工程）和 `seven_kylin`（麒麟 V10 工程）合并而来，**一套代码同时支持 Windows 和 Linux/麒麟 V10 平台**，编译时自动识别当前系统环境。

## 目录结构

```
seven_merge/
├── 3rdparty/                  # 第三方库（Windows 用：glfw, imgui, jsoncpp, Eigen）
├── app/
│   ├── CMakeLists.txt
│   └── main.cpp               # 跨平台入口（#ifdef _WIN32 区分）
├── module/
│   ├── barrage/               # 压制干扰模块
│   ├── core/                  # 核心公共模块（CommonCore.hpp 含跨平台类型定义）
│   ├── deception/             # 欺骗干扰模块
│   ├── process/               # 仿真进程模块（命名管道/Unix Socket 通信）
│   └── transformation/        # 编队变换模块
├── scripts/
│   ├── build_win.bat          # Windows 一键编译脚本
│   └── build_kylin_v10.sh     # 麒麟 V10 一键编译脚本
├── data/                      # 数据文件
├── install/                   # Linux 安装输出目录
└── CMakeLists.txt             # 顶层跨平台 CMake 配置
```

## 跨平台设计说明

### 平台识别机制

CMake 通过 `WIN32` 变量自动识别当前系统：

| 特性 | Windows | Linux / 麒麟 V10 |
|------|---------|-------------------|
| C++ 标准 | C++17 | C++17 |
| JSON 库 | 3rdparty/jsoncpp（本地） | 系统安装的 jsoncpp（pkg-config） |
| Eigen 库 | 3rdparty/Eigen（本地） | 系统安装的 libeigen3-dev |
| GUI 库 | glfw + imgui（3rdparty） | 不需要 |
| IPC 通信 | Windows 命名管道（Named Pipe） | Unix Domain Socket |
| 并行计算 | MSVC PPL（concurrency::parallel_for） | OpenMP（#pragma omp parallel for） |
| 线程命名 | SetThreadDescription | pthread_setname_np |
| 定时器精度 | timeBeginPeriod/timeEndPeriod | 直接 sleep |
| 导出宏 | __declspec(dllexport) | __attribute__((visibility("default"))) |
| 句柄类型 | HANDLE（来自 windows.h） | typedef int HANDLE |
| 安装方式 | 直接输出到 build/bin | make install 到 install/ |

### 关键跨平台文件

1. **CommonCore.hpp** — 核心头文件，定义了所有跨平台类型：
   - `#ifdef _WIN32` 选择 JSON 头文件路径
   - Linux 下 `typedef int HANDLE` 模拟 Windows 句柄
   - Linux 下 `#define INVALID_HANDLE_VALUE (-1)`

2. **process.cpp** — IPC 通信核心：
   - `#ifdef _WIN32` 分支：使用 `CreateNamedPipeW` / `ConnectNamedPipe` / `ReadFile` / `WriteFile`
   - `#else` 分支：使用 `socket` / `bind` / `listen` / `accept` / `recv` / `send`（Unix Domain Socket）

3. **transformation.cpp** — 并行计算：
   - `#ifdef _WIN32` 分支：使用 `concurrency::parallel_for`（需要 `<ppl.h>`）
   - `#else` 分支：使用 `#pragma omp parallel for`（需要 OpenMP）

4. **CalcThread.cpp** — 线程管理：
   - 线程命名：`SetThreadDescription` vs `pthread_setname_np`
   - 定时器：`timeBeginPeriod(1)` vs 直接 `sleep_for`

## 编译方法

### Windows 编译

**前置条件：**
- Visual Studio 2022（含 C++ 开发工具）
- CMake 3.16+

**方式一：一键脚本**
```bat
cd seven_merge
scripts\build_win.bat
```

**方式二：手动编译**
```bat
cd seven_merge
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

输出：`build\bin\Release\app.exe`

### 麒麟 V10 / Linux 编译

**前置条件：**
```bash
sudo apt install -y gcc g++ cmake make libeigen3-dev libjsoncpp-dev pkg-config
```

**方式一：一键脚本**
```bash
cd seven_merge
chmod +x scripts/build_kylin_v10.sh
sudo ./scripts/build_kylin_v10.sh
```

**方式二：手动编译**
```bash
cd seven_merge
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fopenmp" -DCMAKE_EXE_LINKER_FLAGS="-fopenmp"
make -j$(nproc)
make install
```

输出：`install/seven-cli`

## 运行说明

### Windows
- 可执行文件和依赖 DLL 都在 `build\bin\Release\` 目录下
- 直接双击 `app.exe` 运行

### 麒麟 V10 / Linux
- 可执行文件在 `install/` 目录下
- 运行：`cd install && ./seven-cli`
- 程序会在 `/tmp/` 下创建两个 Socket 文件：
  - `/tmp/ClientToServerPipe` — 命令通道
  - `/tmp/ServerToClientPipe` — 数据通道

## 架构适配

- **x86_64**：自动检测，使用 `-m64 -O2 -fPIC` 编译选项
- **ARM64（aarch64）**：自动检测，使用 `-march=armv8-a -O2 -fPIC` 编译选项
- **Windows 32/64 位**：通过 `CMAKE_SIZEOF_VOID_P` 自动检测
