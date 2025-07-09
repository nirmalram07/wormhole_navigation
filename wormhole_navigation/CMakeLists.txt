cmake_minimum_required(VERSION 3.8)
project(wormhole_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(wormhole_nav_msg REQUIRED)
find_package(nlohmann_json REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  wormhole_nav_msg
  ament_index_cpp
  nlohmann_json
  nav2_msgs
)

set(executable_name wormhole_navigator)
set(library_name ${executable_name}_core)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${library_name} SHARED
  src/data_fetcher.cpp
  src/nav_control.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name} sqlite3)

# Main executable
add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} 
  ${library_name}
  sqlite3  
)

# Install the executable
install(TARGETS
  ${executable_name}
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
