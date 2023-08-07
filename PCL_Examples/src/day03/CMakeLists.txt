
add_executable( 01_normal_estimation 01_normal_estimation.cpp)
target_link_libraries( 01_normal_estimation ${PCL_LIBRARIES})
add_executable( 02_normal_estimation 02_normal_estimation.cpp)
target_link_libraries( 02_normal_estimation ${PCL_LIBRARIES})
add_executable( 03_normal_estimation 03_normal_estimation.cpp)
target_link_libraries( 03_normal_estimation ${PCL_LIBRARIES})

add_executable(04-integral_image_normal 04-integral_image_normal.cpp)
target_link_libraries( 04-integral_image_normal ${PCL_LIBRARIES})
add_executable(05-pfh_estimation 05-pfh_estimation.cpp)
target_link_libraries( 05-pfh_estimation ${PCL_LIBRARIES})