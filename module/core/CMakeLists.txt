project (core)

#find_package(OpenCV REQUIRED)
#find_package(pylon REQUIRED)

# 3. 收集源文件（建议用 GLOB_RECURSE 或显式列举，GLOB 可能漏文件）
file(GLOB_RECURSE SRC_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
file(GLOB_RECURSE INC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core/*.hpp
)

set(Eigen3_DIR "${CMAKE_SOURCE_DIR}/3rdparty/Eigen/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)

# 4. 定义库目标：使用项目名作为目标名，创建别名
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
add_library(sub::core ALIAS ${PROJECT_NAME})

set(JSON_LIB "${3DRPARTY_DIR}/jsoncpp/lib/*")

# 5. 设置头文件路径：
target_include_directories(${PROJECT_NAME}
    PUBLIC 
		${PROJECT_SOURCE_DIR}/include  # 自身头文件（对外暴露）
		#${OpenCV_INCLUDE_DIRS}        # OpenCV 头文件（仅内部使用）
		#${pylon_DIR}
)
#message(INC: ${Eigen3::Eigen})

# ===================== 链接 Basler 库和其他依赖 =====================
#("${CMAKE_SOURCE_DIR}/3rdparty/Basler/lib/x64")
#link_directories("${CMAKE_SOURCE_DIR}/3rdparty/Basler/Runtime/x64")

# 7. 链接依赖库：增加目标存在性检查（可选但更健壮）
target_link_libraries(${PROJECT_NAME}
    PUBLIC
		sub::core
		Eigen3::Eigen
		${JSON_LIB}
)
