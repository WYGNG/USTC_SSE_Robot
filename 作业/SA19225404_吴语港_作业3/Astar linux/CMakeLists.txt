# CMake 最低版本号要求
cmake_minimum_required(VERSION 2.8)
# 项目名称
project(Astar)
# 查找指定目录下的所有.cpp与.h文件 并存放到指定变量名SC_FILES中
FILE(GLOB SC_FILES "*.cpp" "*.h")

# 指定可执行文件Astar
add_executable(Astar ${SC_FILES})



