cmake_minimum_required (VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)

project(lqr_test)

find_package(cmake_modules REQUIRED)


add_executable(my_lqr_test lqr_test.cpp)