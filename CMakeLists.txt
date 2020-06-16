cmake_minimum_required(VERSION 3.16)
project(libcppbase
	VERSION 0.0.1
	DESCRIPTION "libcppbase"
	HOMEPAGE_URL https://zizztux.github.io)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_INSTALL_PREFIX "/usr/local/lib")

add_subdirectory(src/pipeline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

set(CMAKE_VERBOSE_MAKEFILE true)

message(STATUS "Project: " ${CMAKE_PROJECT_NAME} " v" ${CMAKE_PROJECT_VERSION})
