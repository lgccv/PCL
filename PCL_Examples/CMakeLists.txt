cmake_minimum_required(VERSION 3.14)
project(PCL_Example)

# --------------------------------------------------------------------------- set
set(CMAKE_CXX_STANDARD 14)

# 设置输出根目录为build/Debug ${CMAKE_BUILD_TYPE}
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build)
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)

# --------------------------------------------------------------------------- find

find_package(PCL REQUIRED)
# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
# 设置依赖库链接目录
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})

# --------------------------------------------------------------------------- add
add_subdirectory(src/day01)
add_subdirectory(src/day02)
add_subdirectory(src/day03)
add_subdirectory(src/day04)
add_subdirectory(src/day05)
add_subdirectory(src/day06)
add_executable(PCL_Example main.cpp)


