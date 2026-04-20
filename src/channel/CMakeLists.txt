include(GTestHelper)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "BUILD_WITH_CHANNEL_AUTO: ${BUILD_WITH_CHANNEL_AUTO}")
message(STATUS "ROS_VERSION: $ENV{ROS_VERSION}")
if(BUILD_WITH_CHANNEL_AUTO)
    if(DEFINED ENV{ROS_VERSION})
        if("$ENV{ROS_VERSION}" STREQUAL "1")
            set(BUILD_WITH_CHANNEL_ROS1 ON)
            message("Auto-detected ROS1 environment")
        elseif("$ENV{ROS_VERSION}" STREQUAL "2")
            set(BUILD_WITH_CHANNEL_ROS2 ON)
            message("Auto-detected ROS2 environment")
        endif()
    endif()
endif()

if(BUILD_WITH_CHANNEL_ROS1)
    message("Building ROS1 channel")
    add_subdirectory(ros1)
endif()

if(BUILD_WITH_CHANNEL_ROS2)
    message("Building ROS2 channel")
    add_subdirectory(ros2)
endif()

if(BUILD_WITH_CHANNEL_ROSBRIDGE)
    message("Building ROSBridge channel")
    add_subdirectory(rosbridge)
endif()

#manager
find_package(Boost REQUIRED COMPONENTS filesystem)


add_library(channel STATIC
  channel_manager.cc
  channel_manager.h
  virtual_channel_node.h
)

if(WIN32)
  target_link_libraries(channel PUBLIC Boost::filesystem common framework)
else()
  target_link_libraries(channel PUBLIC Boost::filesystem pthread dl common framework)
endif()

target_include_directories(channel PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}
manager
)

set_target_properties(channel PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

#######test
if(WIN32)
  set(TEST_LINK_LIBRARIES channel)
else()
  find_library(DL_LIBRARY dl)
  set(TEST_LINK_LIBRARIES channel ${DL_LIBRARY})
endif()
AddTest()