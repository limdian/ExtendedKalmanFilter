cmake_minimum_required(VERSION 3.10)

project(ExtendedKalmanFilter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

set(SOURCES
    src/ExtendedKalmanFilter.cpp
    src/FuncBase.cpp
    src/Measure.cpp
    src/StateTrans.cpp
)

add_executable(test test.cpp ${SOURCES})

target_link_libraries(test
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${CERES_LIBRARIES}
)

# 安装可执行文件
install(TARGETS test
    DESTINATION bin
)
