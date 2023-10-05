cmake_minimum_required(VERSION 3.5)
project(dtc_test)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options(-Wall -Wextra -Wno-unused-variable -Wno-unused-parameter)
    add_compile_options(-g) # Allows geting backtraces
endif()

# Find dependency packages individually
find_package(ament_cmake REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

set(interface_dependencies std_msgs moveit_task_constructor_msgs)

set(dependencies
    ${interface_dependencies}
    moveit_msgs
    model_msgs
    moveit_task_constructor_core
    rclcpp
    rosparam_shortcuts
    tf2_eigen
    tf2_ros
    yaml_cpp_vendor
    yaml-cpp
    rviz_visual_tools
)

foreach(dep_pkg ${dependencies})
    find_package(${dep_pkg} REQUIRED)
endforeach()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN3_DEFINITIONS})

# Set target names
set(idl_target ${PROJECT_NAME})
set(lib_target ${PROJECT_NAME}_lib)
set(cpp_target ${PROJECT_NAME}_cpp)

# ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

# Export dependencies to downstream packages
ament_export_dependencies(rosidl_default_runtime)

#
# Install the library
#
add_library(
    ${lib_target} SHARED
    src/generate_offset_pose.cpp
    src/compute_ik.cpp
    src/dynamic_task_planner.cpp
)

target_include_directories(
    ${lib_target} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${lib_target} ${dependencies} eigen3_cmake_module Eigen3)

# Include interfaces generated in this package
# rosidl_target_interfaces(${lib_target} ${idl_target} "rosidl_typesupport_cpp")

# Export targets to downstream packages
ament_export_targets(${lib_target}Targets HAS_LIBRARY_TARGET)

# Export dependencies to downstream packages
ament_export_dependencies(${dependencies})
ament_export_dependencies(eigen3_cmake_module) # export before Eigen3
ament_export_dependencies(Eigen3)

# Install the header files
install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.h"
)

# Install the library
install(
    TARGETS ${lib_target}
    EXPORT ${lib_target}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include
)

#
# Add executables
#
add_executable(${cpp_target} src/main.cpp)

ament_target_dependencies(${cpp_target} ${dependencies})

set_target_properties(${cpp_target} PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

install(
    TARGETS ${cpp_target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${cpp_target} ${lib_target})

# Install directories
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

#
# Add gtests
#
# if(BUILD_TESTING) find_package(ament_cmake_gtest REQUIRED)
#
# endif()

#
# Generate the package
#
ament_package() # call this last
