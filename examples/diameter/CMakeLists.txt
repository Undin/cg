cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(diameter)

add_executable(diameter diameter.cpp)
target_link_libraries(diameter visualization ${GMP_LIBRARIES})
