project(transformation)

# ===================== 收集源文件 =====================
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE INC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/transformation/*.hpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
add_library(sub::transformation ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

if(WIN32)
    # ===================== Windows =====================
    # Windows 使用 MSVC 并行库 (ppl.h)
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            sub::core
    )
else()
    # ===================== Linux / 麒麟V10 =====================
    # Linux 使用 OpenMP 替代 ppl.h
    find_package(OpenMP REQUIRED)

    target_link_libraries(${PROJECT_NAME}
        sub::core
        OpenMP::OpenMP_CXX
        pthread
        dl
    )

    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()
