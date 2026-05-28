cmake_minimum_required(VERSION 3.10)
project(fenrir_sim_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Harmonic component versions (as vendored with ROS 2 Jazzy).
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

add_library(LedMaterialColor SHARED LedMaterialColor.cc)

target_link_libraries(LedMaterialColor
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::register)

# Install next to the package so GZ_SIM_SYSTEM_PLUGIN_PATH can find it.
install(TARGETS LedMaterialColor
  LIBRARY DESTINATION lib)
