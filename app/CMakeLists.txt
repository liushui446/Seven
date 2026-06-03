project(app)
# Create the executable with ImGui + visualizer built-in
add_executable(${PROJECT_NAME}
    main.cpp
    # ImGui core
    ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/imgui.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/imgui_draw.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/imgui_tables.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/imgui_widgets.cpp
    # ImGui backends
    ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/backends/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/3rdparty/imgui-1.92.8/backends/imgui_impl_opengl3.cpp
    # Visualizer (compiled into app, uses transformation DLL)
    ${CMAKE_SOURCE_DIR}/module/transformation/src/imGui_visualizer.cpp
)

# Link the static library from subproject1 using it's alias sub::lib1
# Link the header only library from subproject2 using it's alias sub::lib2
# This will cause the include directories for that target to be added to this project

# target_include_directories(${PROJECT_NAME}
    # PUBLIC 
		# ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/Jsonc
# )
target_link_libraries(${PROJECT_NAME}
    sub::core
	sub::transformation
	sub::process
	${OPENGL_LIBRARIES}
    ${GLFW_LIB}
    ${GLFW_EXTRA_LIBS}
)

