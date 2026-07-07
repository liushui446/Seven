cmake_minimum_required(VERSION 3.16.0)

# ===================== 跨平台统一工程 =====================
# seven_merge - 同时支持 Windows 和 麒麟V10(Kylin) Linux
# 编译时自动识别当前系统环境，选择对应的依赖库和编译选项
# ==========================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ── Windows 强制 x64 ──（必须放在 project() 之前）
if(WIN32)
    set(CMAKE_GENERATOR_PLATFORM x64 CACHE STRING "Force x64 architecture" FORCE)
    message(STATUS "强制架构: CMAKE_GENERATOR_PLATFORM = ${CMAKE_GENERATOR_PLATFORM}")
endif()

project(seven CXX C)

# ===================== 平台识别与宏定义 =====================
if(WIN32)
    message(STATUS "=========================================")
    message(STATUS "  检测到 Windows 平台")
    message(STATUS "=========================================")
    add_definitions(-DWindows)

    # Windows 架构检测（优先用 CMAKE_GENERATOR_PLATFORM，更可靠）
    if(CMAKE_GENERATOR_PLATFORM MATCHES "^(x64|amd64|ARM64)$" OR CMAKE_SIZEOF_VOID_P EQUAL 8)
        add_compile_definitions(_M_X64)
        message(STATUS "Windows 64位架构: _M_X64")
    else()
        add_compile_definitions(_M_IX86)
        message(STATUS "Windows 32位架构: _M_IX86")
    endif()

    # Windows 版本宏（兼容 Windows 10/11）
    add_compile_definitions(
        _WIN32_WINNT=0x0A00
        NTDDI_VERSION=0x0A000006
        WIN32_LEAN_AND_MEAN
    )

    # ---- Windows 第三方库路径 ----
    set(GLFW_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/glfw)
    set(3DRPARTY_DIR "${CMAKE_SOURCE_DIR}/3rdparty")

    # 收集 imgui 源文件
    file(GLOB IMGUI_SRC
        ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/*.cpp
        ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/backends/imgui_impl_glfw.cpp
        ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/backends/imgui_impl_opengl3.cpp
    )

    # Windows 头文件目录
    include_directories(
        ${3DRPARTY_DIR}/jsoncpp/include
        ${3DRPARTY_DIR}/imgui-1.92.8
        ${3DRPARTY_DIR}/imgui-1.92.8/backends
        ${GLFW_ROOT}/include
    )

    # Windows 查找 OpenGL 和 GLFW
    find_package(OpenGL REQUIRED)
    find_library(GLFW_LIB
        NAMES glfw3 glfw3dll glfw3_mt
        PATHS ${GLFW_ROOT}/lib-vc2022
        REQUIRED
    )
    set(GLFW_EXTRA_LIBS opengl32 gdi32 user32 shell32)

    # 复制三方库 DLL 文件到输出目录
    file(GLOB JSON_DEBUG_FILES ${3DRPARTY_DIR}/jsoncpp/Debug/*)
    file(GLOB JSON_RELEASE_FILES ${3DRPARTY_DIR}/jsoncpp/Release/*)
    file(COPY ${JSON_DEBUG_FILES} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
    file(COPY ${JSON_RELEASE_FILES} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
    file(GLOB GLFW3_FILES ${GLFW_ROOT}/lib-vc2022/glfw3.dll)
    file(COPY ${GLFW3_FILES} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
    file(COPY ${GLFW3_FILES} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)

    # Windows 输出目录
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

else()
    message(STATUS "=========================================")
    message(STATUS "  检测到 Linux 平台")
    message(STATUS "=========================================")
    add_definitions(-DLinux)

    # ---- 麒麟V10 系统识别 ----
    execute_process(COMMAND cat /etc/os-release OUTPUT_VARIABLE OS_RELEASE_INFO ERROR_QUIET)
    string(FIND "${OS_RELEASE_INFO}" "Kylin" KYLIN_FLAG)
    if(${KYLIN_FLAG} GREATER -1)
        message(STATUS "  -> 检测到银河麒麟V10桌面版")
        add_definitions(-DKYLIN_V10_OS)
    else()
        message(STATUS "  -> 通用 Linux 发行版")
    endif()

    # ---- 架构适配 ----
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        message(STATUS "  -> 架构：ARM64")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a -O2 -fPIC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -O2 -fPIC")
    else()
        message(STATUS "  -> 架构：x86_64")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64 -O2 -fPIC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -O2 -fPIC")
    endif()

    # ---- Linux 编译选项 ----
    set(CMAKE_C_STANDARD 99)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    add_compile_options(-Wall -Wno-unused-variable -Wno-unused-function -fPIC)

    # ---- Linux RPATH（程序从自身目录找库）----
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
    set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

    # ---- Linux 安装目录 ----
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Install path" FORCE)
    endif()

    # ---- Linux 全局头文件 ----
    include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/module
        ${PROJECT_SOURCE_DIR}/3rdparty
    )
endif()

message(STATUS "=========================================")

# ===================== 子模块 =====================
add_subdirectory(module/core)
add_subdirectory(module/barrage)
add_subdirectory(module/deception)
add_subdirectory(module/transformation)
add_subdirectory(module/process)
add_subdirectory(app)

# ===================== 安装完成提示（Linux）=====================
if(NOT WIN32)
    install(CODE "MESSAGE(STATUS \"编译完成！安装目录：${CMAKE_INSTALL_PREFIX}\")")
endif()
