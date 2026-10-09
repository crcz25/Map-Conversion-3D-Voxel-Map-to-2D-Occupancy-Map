cmake_minimum_required(VERSION 3.5)
project(mapconversion)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mapconversion_msgs REQUIRED)

# Find OctoMap dependencies
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS} 
)

add_executable(path_converter_node src/path_converter_node.cpp)
ament_target_dependencies(path_converter_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  mapconversion_msgs
)

# Create library from MapConverter class
add_library(map_conversion_class src/MapConverter.cpp)
ament_target_dependencies(map_conversion_class 
  Eigen3
)

# Link the library to the executable properly
add_executable(map_conversion_oct_node src/map_conversion_oct_node.cpp)
target_link_libraries(map_conversion_oct_node 
map_conversion_class
${OCTOMAP_LIBRARIES}
)  
ament_target_dependencies(map_conversion_oct_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  mapconversion_msgs
  octomap_ros
  octomap
  octomap_msgs
)

# Install executables
install(TARGETS
  path_converter_node
  map_conversion_oct_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Map2D is header-only; MapConverter is the map_conversion_class library.
  ament_add_gtest(test_map2d test/test_map2d.cpp)
  ament_target_dependencies(test_map2d Eigen3)
  ament_add_gtest(test_map_converter test/test_map_converter.cpp)
  target_link_libraries(test_map_converter map_conversion_class)
  ament_target_dependencies(test_map_converter Eigen3)
  # PathConverter: test/test_path_converter_node.cpp compiles
  # src/path_converter_node.cpp with main() renamed. Creates a node, so it
  # gets its own ROS_DOMAIN_ID.
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(test_path_converter_node
    test/test_path_converter_node.cpp TIMEOUT 120)
  ament_target_dependencies(test_path_converter_node
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    mapconversion_msgs
  )

  # Static analysis, opt-in with -DENABLE_LINT=ON (tools/ci/run_ci.sh): the
  # default colcon test runs no lint test.
  option(ENABLE_LINT "Add the cppcheck and clang-tidy tests" OFF)
  if(ENABLE_LINT)
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(include src test
      LANGUAGE c++ INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
    # ament_cppcheck otherwise skips cppcheck 1.88 and 2.x as too slow.
    set_tests_properties(cppcheck PROPERTIES
      ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
    find_package(ament_cmake_clang_tidy QUIET)
    if(ament_cmake_clang_tidy_FOUND)
      # The workspace .clang-tidy, in this directory or the nearest parent.
      set(_clang_tidy_dir "${CMAKE_CURRENT_SOURCE_DIR}")
      while(NOT EXISTS "${_clang_tidy_dir}/.clang-tidy" AND
            NOT _clang_tidy_dir STREQUAL "/")
        get_filename_component(_clang_tidy_dir "${_clang_tidy_dir}" DIRECTORY)
      endwhile()
      # ament_clang_tidy passes its own --header-filter (include/<package>/),
      # which overrides HeaderFilterRegex, and runs one job under a 300 s
      # timeout: report this package's own headers, on every core.
      cmake_host_system_information(RESULT _clang_tidy_jobs
        QUERY NUMBER_OF_LOGICAL_CORES)
      ament_clang_tidy(${CMAKE_CURRENT_BINARY_DIR}
        --jobs ${_clang_tidy_jobs}
        --header-filter "${CMAKE_CURRENT_SOURCE_DIR}/(include|src|test)/.*"
        CONFIG_FILE "${_clang_tidy_dir}/.clang-tidy"
        TIMEOUT 1800)
    else()
      message(WARNING "ament_cmake_clang_tidy not found: no clang_tidy test")
    endif()
  endif()
endif()

ament_package()
