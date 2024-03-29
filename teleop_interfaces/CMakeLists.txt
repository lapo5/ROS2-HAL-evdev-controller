cmake_minimum_required(VERSION 3.5)
project(teleop_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

########################################
# Declaration of messages and services #
########################################

set(msg_files
	"msgs/AxisCmd.msg"
	"msgs/ButtonCmd.msg"
 )
   
set(srv_files
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${srv_files}
	DEPENDENCIES std_msgs
)
	
ament_export_dependencies(rosidl_default_runtime)
ament_package()
