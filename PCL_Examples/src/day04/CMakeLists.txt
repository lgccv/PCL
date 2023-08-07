
add_executable(01-iterative_closest_point 01-iterative_closest_point.cpp)
target_link_libraries( 01-iterative_closest_point ${PCL_LIBRARIES})
add_executable(02-interactive_icp 02-interactive_icp.cpp)
target_link_libraries( 02-interactive_icp ${PCL_LIBRARIES})