#cmake verson，指定cmake版本 
cmake_minimum_required(VERSION 3.2)
 
#project name，指定项目的名称，一般和项目的文件夹名称对应
PROJECT(HELLO)
 
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)

SET(CMAKE_BUILD_TYPE "Release")

#INSTALL(TARGETS libhello LIBRARY DESTINATION /usr/local/lib)
#INSTALL(./libhello/hello.h /usr/local/include)
