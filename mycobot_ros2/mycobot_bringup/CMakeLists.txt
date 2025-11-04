cmake_minimum_required(VERSION 3.8)
project(mycobot_bringup)

# Compiler warnings for GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Uncomment the following section to manually add further dependencies
# find_package(<dependency> REQUIRED)

# Enable testing and linting if requested
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # Skip copyright linter until copyrights/licenses are added
    set(ament_cmake_copyright_FOUND TRUE)

    # Skip cpplint (only works in a git repo)
    set(ament_cmake_cpplint_FOUND TRUE)

    ament_lint_auto_find_test_dependencies()
endif()

# Install Python launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Finalize package
ament_package()

