project(app)

# ===================== 收集源文件 =====================
file(GLOB APP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
)

if(WIN32)
    # ===================== Windows =====================
    add_executable(${PROJECT_NAME} ${APP_SRC})

    target_link_libraries(${PROJECT_NAME}
        sub::core
        sub::transformation
        sub::process
        ${OPENGL_LIBRARIES}
        ${GLFW_LIB}
        ${GLFW_EXTRA_LIBS}
    )
else()
    # ===================== Linux / 麒麟V10 =====================
    add_executable(seven-cli ${APP_SRC})

    target_link_libraries(seven-cli
        sub::core
        sub::process
        pthread
        dl
        m
        stdc++
    )

    install(TARGETS seven-cli
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
    )
endif()
