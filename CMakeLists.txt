cmake_minimum_required(VERSION 3.8)
project(bxi_revo2_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(communication REQUIRED)

#revo2相关文件
add_compile_definitions(STARK_USE_BXIPCI)
file(GLOB_RECURSE revo2_source ${CMAKE_SOURCE_DIR}/common/*.cpp)

add_executable(bxi_revo2_example src/bxi_revo2_example.cpp ${revo2_source})
target_include_directories(bxi_revo2_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bxi_revo2_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  bxi_revo2_example
  "rclcpp"
  "ament_cmake"
  "std_msgs"
  "communication"
)
#revo2相关文件
target_include_directories(bxi_revo2_example PRIVATE common dist/include)
target_link_libraries(bxi_revo2_example ${CMAKE_SOURCE_DIR}/dist/shared/linux/libbc_stark_sdk.so)
set_target_properties(bxi_revo2_example PROPERTIES
    INSTALL_RPATH ${CMAKE_SOURCE_DIR}/dist/shared/linux
    BUILD_RPATH ${CMAKE_SOURCE_DIR}/dist/shared/linux
)

install(TARGETS bxi_revo2_example
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
