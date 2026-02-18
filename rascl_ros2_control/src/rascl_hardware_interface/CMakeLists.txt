cmake_minimum_required(VERSION 3.8)
project(rascl_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(soem REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  
#add_executable(rascl_hardware_interface src/rascl_hardware_interface.cpp)  

target_link_libraries(rascl_hardware_interface soem)

ament_target_dependencies(rascl_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  # soem
  # ament_index_cpp
  # geometry_msgs
  # sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface rascl_hardware_interface.xml)
  
target_compile_features(rascl_hardware_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)


# enable testing functionality
enable_testing()


if(BUILD_TESTING)
  #find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  

  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  #target_link_libraries(${PROJECT_NAME} ament_cmake_gmock)
  
  #ament_add_gmock(test_load_rascl_hardware_interface test/test_load_rascl_hardware_interface.cpp)
endif()



ament_package()
