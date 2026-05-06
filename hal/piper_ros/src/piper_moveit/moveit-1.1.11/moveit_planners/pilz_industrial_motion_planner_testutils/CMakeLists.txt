cmake_minimum_required(VERSION 3.1.3)
project(pilz_industrial_motion_planner_testutils)

find_package(catkin REQUIRED COMPONENTS
  tf2_eigen
  moveit_core
  moveit_msgs
)

find_package(Boost REQUIRED COMPONENTS)

################
## Clang tidy ##
################
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_core moveit_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/cartesianconfiguration.cpp
  src/jointconfiguration.cpp
  src/robotconfiguration.cpp
  src/sequence.cpp
  src/xml_testdata_loader.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
