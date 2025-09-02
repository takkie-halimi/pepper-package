cmake_minimum_required(VERSION 3.0.2)
project(pepper_vhw_interface)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rospy
)

# Declare a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pepper_virtual_hw_interface
  CATKIN_DEPENDS controller_manager hardware_interface roscpp rospy
#  DEPENDS system_lib
)

# Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(virtual_hw_interface src/virtual_hw_interface.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(virtual_hw_interface ${catkin_LIBRARIES})

# Add install targets
install(TARGETS virtual_hw_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

