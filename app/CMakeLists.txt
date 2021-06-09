add_executable(rrt test_rrt.cc)
target_link_libraries(rrt env utils)

add_executable(rrt_spreading test_rrt_spreading.cc)
target_link_libraries(rrt_spreading env utils)

add_executable(rcs test_rcs.cc)
target_link_libraries(rcs env utils)

add_executable(rcs_spreading test_rcs_spreading.cc)
target_link_libraries(rcs_spreading env utils)
