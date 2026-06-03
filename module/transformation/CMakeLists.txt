project (transformation)

#find_package(OpenCV REQUIRED)
#find_package(pylon REQUIRED)

# 3. 收集源文件（排除 imGui_visualizer，它编译进 app）
file(GLOB SRC_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/src/transformation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory_serializer.cpp
)
file(GLOB_RECURSE INC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/transformation/*.hpp
)

# 4. 定义库目标：使用项目名作为目标名，创建别名
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
add_library(sub::transformation ALIAS ${PROJECT_NAME})


# 5. 设置头文件路径：
target_include_directories(${PROJECT_NAME}
    PUBLIC 
		${PROJECT_SOURCE_DIR}/include  # 自身头文件（对外暴露）
		#${OpenCV_INCLUDE_DIRS}            # OpenCV 头文件（仅内部使用）
		#${pylon_DIR}
)

# 添加编译定义以导出符号
target_compile_definitions(${PROJECT_NAME} PRIVATE TRANSFORMATION_EXPORTS_DLL)
#message(INC: ${pylon_DIR})

# ===================== 链接 Basler 库和其他依赖 =====================
#("${CMAKE_SOURCE_DIR}/3rdparty/Basler/lib/x64")
#link_directories("${CMAKE_SOURCE_DIR}/3rdparty/Basler/Runtime/x64")
#set(json_LIB "${3DRPARTY_DIR}/JSON/lib/jsoncpp.lib")

# 5. 链接 JsonCpp 库（静态库，无需处理 dll）
#target_link_libraries(${PROJECT_NAME} PRIVATE jsoncpp_static)

# 7. 链接依赖库：增加目标存在性检查（可选但更健壮）
target_link_libraries(${PROJECT_NAME}
    PUBLIC
		sub::core
		#pylon::pylon  # 链接 Basler 库
		#${OpenCV_LIBS}
)

# ===================== 测试目标 =====================
option(BUILD_TRANSFORMATION_TESTS "Build transformation module tests" ON)

if(BUILD_TRANSFORMATION_TESTS)
	# 收集测试源文件
	file(GLOB TEST_SRC_FILES
		${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp
	)

	# 测试可执行文件 - 直接编译源文件以避免DLL导出问题
	add_executable(transformation_test
		${TEST_SRC_FILES}
		${CMAKE_CURRENT_SOURCE_DIR}/src/transformation.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory_serializer.cpp
	)

	# 头文件路径
	target_include_directories(transformation_test
		PRIVATE
			${PROJECT_SOURCE_DIR}/include
			${CMAKE_SOURCE_DIR}/module/core/include
			${CMAKE_CURRENT_SOURCE_DIR}/test
	)

	# 定义导出宏（直接编译源文件，需要dllexport而非dllimport）
	target_compile_definitions(transformation_test PRIVATE TRANSFORMATION_EXPORTS_DLL)

	# UTF-8 编码支持 (MSVC)
	if(MSVC)
		target_compile_options(transformation_test PRIVATE /utf-8)
	endif()

	# 链接依赖
	target_link_libraries(transformation_test
		PRIVATE
			sub::core
	)

	# 输出到测试目录
	set_target_properties(transformation_test PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/test"
	)

	# 复制依赖DLL到测试输出目录
	add_custom_command(TARGET transformation_test POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			"${CMAKE_BINARY_DIR}/bin/$<CONFIG>/jsoncpp.dll"
			"${CMAKE_BINARY_DIR}/bin/test/$<CONFIG>/"
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			"${CMAKE_BINARY_DIR}/bin/$<CONFIG>/core.dll"
			"${CMAKE_BINARY_DIR}/bin/test/$<CONFIG>/"
	)

	message(STATUS "Transformation tests: ENABLED")
endif()
