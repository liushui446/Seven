project (process)

#find_package(OpenCV REQUIRED)
#find_package(pylon REQUIRED)

# 3. 收集源文件（建议用 GLOB_RECURSE 或显式列举，GLOB 可能漏文件）
file(GLOB_RECURSE SRC_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
file(GLOB_RECURSE INC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/process/*.hpp
)

# 4. 定义库目标：使用项目名作为目标名，创建别名
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
add_library(sub::process ALIAS ${PROJECT_NAME})


# 5. 设置头文件路径：
target_include_directories(${PROJECT_NAME}
    PUBLIC 
		${PROJECT_SOURCE_DIR}/include  # 自身头文件（对外暴露）
		#${OpenCV_INCLUDE_DIRS}            # OpenCV 头文件（仅内部使用）
		#${pylon_DIR}
)

#message(INC: ${JSON_DEBUG_FILES})
#message(bin: ${CMAKE_BINARY_DIR})
#file(COPY ${3DRPARTY_DIR}/JSON/Debug DESTINATION ${PROJECT_BINARY_DIR}/bin/Release)
#set(json_LIB "${3DRPARTY_DIR}/JSON/lib/*")

# ===================== 链接 Basler 库和其他依赖 =====================
#("${CMAKE_SOURCE_DIR}/3rdparty/Basler/lib/x64")
#link_directories("${CMAKE_SOURCE_DIR}/3rdparty/Basler/Runtime/x64")

# 7. 链接依赖库：增加目标存在性检查（可选但更健壮）
target_link_libraries(${PROJECT_NAME}
    PUBLIC
		sub::core
		sub::transformation
		#pylon::pylon  # 链接 Basler 库
		#${OpenCV_LIBS}
)
