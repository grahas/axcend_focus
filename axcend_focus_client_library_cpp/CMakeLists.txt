cmake_minimum_required(VERSION 3.8)
project(axcend_focus_client_library_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(axcend_focus_client_exe src/main.cpp)
set_target_properties(axcend_focus_client_exe PROPERTIES OUTPUT_NAME "axcend_focus_client_cli_debug_tool")
target_link_libraries(axcend_focus_client_exe axcend_focus_client_library)
ament_target_dependencies(axcend_focus_client_exe rclcpp std_msgs)
target_include_directories(axcend_focus_client_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(axcend_focus_client_exe PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_library(axcend_focus_client_library SHARED src/axcend_focus_client.cpp)
set_target_properties(axcend_focus_client_library PROPERTIES OUTPUT_NAME "axcend_focus_client")
target_compile_definitions(axcend_focus_client_library PRIVATE "AXCEND_FOCUS_CLIENT_LIBRARY_CPP_BUILDING_DLL")
ament_target_dependencies(axcend_focus_client_library rclcpp std_msgs)
target_include_directories(axcend_focus_client_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(axcend_focus_client_library PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS axcend_focus_client_exe
  DESTINATION bin/${PROJECT_NAME})

install(TARGETS axcend_focus_client_library
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/axcend_focus_client_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test axcend_focus_client_library)
endif()

ament_package()
