project(process)

# ===================== 收集源文件 =====================
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE INC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/process/*.hpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
add_library(sub::process ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

if(WIN32)
    # ===================== Windows =====================
    # Windows 使用命名管道 (Named Pipe)
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            sub::core
            sub::transformation
            sub::barrage
            sub::deception
    )
else()
    # ===================== Linux / 麒麟V10 =====================
    # Linux 使用 Unix Domain Socket
    target_link_libraries(${PROJECT_NAME}
        sub::core
        sub::barrage
        sub::deception
        sub::transformation
        pthread
        dl
    )

    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()
