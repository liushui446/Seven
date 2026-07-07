project(core)

# ===================== 收集源文件 =====================
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE INC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/core/*.hpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
add_library(sub::core ALIAS ${PROJECT_NAME})

if(WIN32)
    # ===================== Windows =====================
    # Eigen3 路径（3rdparty 本地）
    set(Eigen3_DIR "${CMAKE_SOURCE_DIR}/3rdparty/Eigen/share/eigen3/cmake")
    find_package(Eigen3 REQUIRED)

    target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${PROJECT_SOURCE_DIR}/include
    )

    set(JSON_LIB "${3DRPARTY_DIR}/jsoncpp/lib/*")

    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            Eigen3::Eigen
            ${JSON_LIB}
    )
else()
    # ===================== Linux / 麒麟V10 =====================
    find_package(Eigen3 REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP REQUIRED jsoncpp)

    target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${JSONCPP_INCLUDE_DIRS}
        /usr/include/jsoncpp
    )

    target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        ${JSONCPP_LIBRARIES}
        pthread
        dl
        m
    )

    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()
