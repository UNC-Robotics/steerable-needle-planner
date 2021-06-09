add_library(utils utils.cc)
add_library(env image_environment.cc)
target_link_libraries(env utils)
