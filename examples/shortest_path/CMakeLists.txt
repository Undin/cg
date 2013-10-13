cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(shortest_path)

add_executable(visibility_graph visibility_graph.cpp)
target_link_libraries(visibility_graph visualization ${GMP_LIBRARIES})

add_executable(shortest_path shortest_path.cpp)
target_link_libraries(shortest_path visualization ${GMP_LIBRARIES})

add_executable(minkowski_sum minkowski_sum.cpp)
target_link_libraries(minkowski_sum visualization ${GMP_LIBRARIES})
