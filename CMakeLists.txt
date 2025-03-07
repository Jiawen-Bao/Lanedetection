cmake_minimum_required(VERSION 3.10)
project(LaneDetection)

set(CMAKE_CXX_STANDARD 14)

# vcpkg 集成
set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")


# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 源码文件
file(GLOB SOURCES "src/*.cpp")

add_executable(LaneDetection ${SOURCES})

# 链接 OpenCV
target_link_libraries(LaneDetection ${OpenCV_LIBS})
