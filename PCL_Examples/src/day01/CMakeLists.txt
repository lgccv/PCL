add_executable(01-pcl_demo 01-pcl_demo.cpp)
target_link_libraries(01-pcl_demo ${PCL_LIBRARIES})

add_executable(02-cloud_viewer 02-cloud_viewer.cpp)
target_link_libraries(02-cloud_viewer ${PCL_LIBRARIES})

add_executable(03-pcl_visualizer 03-pcl_visualizer.cpp)
target_link_libraries(03-pcl_visualizer ${PCL_LIBRARIES})

add_executable(04-pcl_transform 04-pcl_transform.cpp)
target_link_libraries(04-pcl_transform ${PCL_LIBRARIES})

add_executable(05-load_pcd 05-load_pcd.cpp)
target_link_libraries(05-load_pcd ${PCL_LIBRARIES})

add_executable(06-save_pcd 06-save_pcd.cpp)
target_link_libraries(06-save_pcd ${PCL_LIBRARIES})

add_executable(07-kd_tree 07-kd_tree.cpp)
target_link_libraries(07-kd_tree ${PCL_LIBRARIES})
add_executable(08-oc_tree 08-oc_tree.cpp)
target_link_libraries(08-oc_tree ${PCL_LIBRARIES})