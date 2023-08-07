add_executable(01-pass_through 01-pass_through.cpp)
target_link_libraries(01-pass_through ${PCL_LIBRARIES})

add_executable(02-downsample_voxel_grid 02-downsample_voxel_grid.cpp)
target_link_libraries(02-downsample_voxel_grid ${PCL_LIBRARIES})

add_executable(03-statistical_removal 03-statistical_removal.cpp)
target_link_libraries(03-statistical_removal ${PCL_LIBRARIES})

add_executable(04-remove_outliers 04-remove_outliers.cpp)
target_link_libraries(04-remove_outliers ${PCL_LIBRARIES})

add_executable(05-range_image 05-range_image.cpp)
target_link_libraries(05-range_image ${PCL_LIBRARIES})
add_executable(06-range_image_border 06-range_image_border.cpp)
target_link_libraries(06-range_image_border ${PCL_LIBRARIES})

add_executable(07-narf_keypoint_extraction 07-narf_keypoint_extraction.cpp)
target_link_libraries(07-narf_keypoint_extraction ${PCL_LIBRARIES})
add_executable(08-random_sample_consensus 08-random_sample_consensus.cpp)
target_link_libraries(08-random_sample_consensus ${PCL_LIBRARIES})