cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# below sets up code coverage and uncrustify config file location
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
  set(ARG_CONFIG_FILE "${PROJECT_SOURCE_DIR}/uncrustify_google_cpp_allman.cfg"
    CACHE STRING "uncrustify config file")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the following lines skip flake8 check
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/basic_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs)
endif()


# build my dummy library
add_library(my_dummy_lib
  src/my_dummy_lib_funct1.cpp
  src/my_dummy_lib_funct2.cpp)
target_include_directories(my_dummy_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# define the executables
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# link the executables with my dummy library
target_link_directories(talker PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(talker my_dummy_lib)
target_link_directories(listener PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(listener my_dummy_lib)

# install executables
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

## Install some arbitrary executables
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})


ament_package()
