add_executable(01-incrementtally_register 01-incrementtally_register.cpp)
target_link_libraries(01-incrementtally_register ${PCL_LIBRARIES})

add_executable(02-alignment_prerejective 02-alignment_prerejective.cpp)
target_link_libraries(02-alignment_prerejective ${PCL_LIBRARIES})

add_executable(03-planar_segmentation 03-planar_segmentation.cpp)
target_link_libraries(03-planar_segmentation ${PCL_LIBRARIES})

add_executable(04-cylinder_segmentation 04-cylinder_segmentation.cpp)
target_link_libraries(04-cylinder_segmentation ${PCL_LIBRARIES})

add_executable(05-cluster_extraction 05-cluster_extraction.cpp)
target_link_libraries(05-cluster_extraction ${PCL_LIBRARIES})

add_executable(06-resampling 06-resampling.cpp)
target_link_libraries(06-resampling ${PCL_LIBRARIES})

add_executable(07-moment_of_inertia 07-moment_of_inertia.cpp)
target_link_libraries(07-moment_of_inertia ${PCL_LIBRARIES})
