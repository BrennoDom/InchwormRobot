cmake_minimum_required(VERSION 3.8)
project(magnet_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

add_library(MagnetPlugin SHARED src/MagnetPlugin.cpp)
target_compile_features(MagnetPlugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# set_property(TARGET ForcePlugin PROPERTY CXX_STANDARD 17)

target_include_directories(MagnetPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(MagnetPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6::ignition-gazebo6)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(MagnetPlugin PRIVATE "MAGNET_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY worlds launch models config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS MagnetPlugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  MagnetPlugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
