
cmake_minimum_required(VERSION 3.8)
project(escalador-gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(ROS_PKGS
    rclcpp
    std_msgs
    sensor_msgs
  )
set(LIBRARIES
    Qt5Widgets
    Qt5PrintSupport
  )
foreach(DEPEND ${ROS_PKGS} ${LIBRARIES})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PACKAGE_NAME})  ## this package name is the name of the directory this cmake file is in
add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# find dependencies

################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
)

include_directories(
  include
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)  
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
)


qt5_wrap_ui (UI_HDRS  ${UI})

add_executable(gui src/gui_escalador.cpp ${HDRS} ${SRCS} ${UI})
ament_target_dependencies(gui 
rclcpp
std_msgs
sensor_msgs)
target_link_libraries(gui Qt5::Widgets Qt5::PrintSupport)
target_include_directories(gui PRIVATE "include/${PROJECT_NAME}/")

install(TARGETS
  gui
  DESTINATION lib/${PROJECT_NAME}
)

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
