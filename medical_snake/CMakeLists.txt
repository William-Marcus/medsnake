cmake_minimum_required(VERSION 3.5)

# set the project name
project(medical_snake)

set (CMAKE_CXX_STANDARD 14)

include_directories(
  include
  deps/DynamixelSDK/c++/include/dynamixel_sdk
)

################################################################################
# Build YAML-cpp
################################################################################
include_directories(
	deps/yaml-cpp/include
)
add_subdirectory("deps/yaml-cpp/")

################################################################################
# Build Dynamixel SDK
################################################################################

if(APPLE)
	set(PORT_HANDLER "mac")
elseif(WIN32)
	set(PORT_HANDLER "windows")
else()
	set(PORT_HANDLER "linux")
endif()

add_library(dynamixel_sdk SHARED
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/packet_handler.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/protocol1_packet_handler.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/protocol2_packet_handler.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/group_sync_read.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/group_sync_write.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/group_bulk_read.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/group_bulk_write.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/port_handler.cpp"
	"deps/DynamixelSDK/c++/src/dynamixel_sdk/port_handler_${PORT_HANDLER}.cpp"
)

################################################################################
# Build
################################################################################

add_library(medical_snake
	"src/dynamixel_controller.cpp"
	"src/medical_snake.cpp"
	"src/cmd_joint_state_publisher.cpp"
	"src/medsnake_control.cpp"
)

target_link_libraries(medical_snake PUBLIC dynamixel_sdk yaml-cpp)

################################################################################
# ROS 2 SPECIFIC BUILD STEPS
################################################################################
# find_package(ament_cmake)
# if(ament_cmake_FOUND)
# 	# target_include_directories(medical_snake
# 	# 	PUBLIC
# 	# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	# 	$<INSTALL_INTERFACE:include>)
# 	# ament_export_targets(medical_snake HAS_LIBRARY_TARGET)
# 	# ament_export_dependencies(yaml-cpp)
# 	# ament_export_dependencies(dynamixel_sdk)
		
# 	# install(
# 	# 	DIRECTORY include/
# 	# 	DESTINATION include
# 	# )
	
# 	# install(
# 	# 	TARGETS medical_snake
# 	# 	EXPORT my_libraryTargets
# 	# 	LIBRARY DESTINATION lib
# 	# 	ARCHIVE DESTINATION lib
# 	# 	RUNTIME DESTINATION bin
# 	# 	INCLUDES DESTINATION include
# 	# )
# 	ament_export_include_directories(include)
# 	ament_export_libraries(medical_snake)
# 	ament_package()
################################################################################
# ROS 1 SPECIFIC BUILD STEPS
################################################################################
find_package(catkin COMPONENTS roscpp message_generation)
if(catkin_FOUND)
	add_message_files(
		FILES
		Tension_readings.msg
		Motor_positions.msg
		)
	generate_messages(
		DEPENDENCIES
		std_msgs  # Or other packages containing msgs
		)
	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES ${PROJECT_NAME}
		CATKIN_DEPENDS roscpp message_runtime
		DEPENDS)
	
	include_directories(${catkin_INCLUDE_DIRS})
	# add the executable
	add_executable(medical_snake_node src/medical_snake_ros.cpp)
	message("Building ROS 1 library")
	target_link_libraries(medical_snake_node PUBLIC medical_snake ${catkin_LIBRARIES})
	add_executable(command_publisher src/cmd_key_node.cpp)
	target_link_libraries(command_publisher ${catkin_LIBRARIES})
	# add_dependencies(command_publisher roscpp)

else()
	include(FetchContent)
	FetchContent_Declare(
	  googletest
	  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
	)
	# For Windows: Prevent overriding the parent project's compiler/linker settings
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_MakeAvailable(googletest)

	enable_testing()

	add_executable(
	  test_medsnake
	  test/test.cpp
	)
	target_link_libraries(
	  test_medsnake
	  gtest_main
	  medical_snake
	  dynamixel_sdk
	  yaml-cpp
	)

	include(GoogleTest)
	gtest_discover_tests(test_medsnake)
endif()