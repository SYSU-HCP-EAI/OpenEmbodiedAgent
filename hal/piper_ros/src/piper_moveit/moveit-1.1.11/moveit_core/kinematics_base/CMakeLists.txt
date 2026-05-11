set(MOVEIT_LIB_NAME moveit_kinematics_base)

add_library(${MOVEIT_LIB_NAME} src/kinematics_base.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # Avoid warnings about deprecated members of KinematicsBase when building KinematicsBase itself.
  # TODO: remove when deperecated variables (tip_frame_, search_discretization_) are finally removed.
  target_compile_options(${MOVEIT_LIB_NAME} PRIVATE -Wno-deprecated-declarations)
endif()

include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME} EXPORT_FILE_NAME ${CATKIN_DEVEL_PREFIX}/include/moveit/${MOVEIT_LIB_NAME}_export.h)
target_include_directories(${MOVEIT_LIB_NAME} PRIVATE $<BUILD_INTERFACE:${CATKIN_DEVEL_PREFIX}/include>)  # for this library
list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)  # for use by other libraries in moveit_core

# This line ensures that messages are built before the library is built
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES ${CATKIN_DEVEL_PREFIX}/include/moveit/${MOVEIT_LIB_NAME}_export.h
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/moveit)
