cmake_minimum_required(VERSION 3.8)
project(axcend_focus_client_library_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(axcend_focus_client SHARED src/axcend_focus_client.cpp)
target_compile_definitions(axcend_focus_client PRIVATE "AXCEND_FOCUS_CLIENT_LIBRARY_CPP_BUILDING_DLL")
ament_target_dependencies(axcend_focus_client rclcpp std_msgs)
target_include_directories(axcend_focus_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(axcend_focus_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS axcend_focus_client
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
  ament_add_gtest(${PROJECT_NAME}_axcend_focus_client_test test/axcend_focus_client_test.cpp)
  target_include_directories(${PROJECT_NAME}_axcend_focus_client_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_axcend_focus_client_test axcend_focus_client)
endif()

ament_package()
