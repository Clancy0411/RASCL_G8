cmake_minimum_required(VERSION 3.19)
project(rascl_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rascl_hardware_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface rascl_hardware_interface.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rascl_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# The main branch lost the executable bit that exists on auto_homing. Generate
# a build-tree copy with explicit POSIX permissions so --symlink-install also
# resolves to an executable target, independent of the source checkout mode.
set(BRIDGE_BUILD_SCRIPT
  "${CMAKE_CURRENT_BINARY_DIR}/scripts/rascl_faulhaber_bridge.py"
)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/scripts")
configure_file(
  scripts/rascl_faulhaber_bridge.py
  "${BRIDGE_BUILD_SCRIPT}"
  COPYONLY
)
file(CHMOD "${BRIDGE_BUILD_SCRIPT}"
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(PROGRAMS
  "${BRIDGE_BUILD_SCRIPT}"
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  ament_add_gtest(test_generic_system test/test_generic_system.cpp)
  if(TARGET test_generic_system)
    target_link_libraries(test_generic_system ${PROJECT_NAME})
    ament_target_dependencies(test_generic_system
      hardware_interface
      rclcpp
      rclcpp_lifecycle
    )
  endif()

  ament_add_pytest_test(test_faulhaber_bridge test/test_faulhaber_bridge.py)

  ament_clang_format()
  ament_cppcheck()
  ament_cpplint()
endif()

ament_package()
