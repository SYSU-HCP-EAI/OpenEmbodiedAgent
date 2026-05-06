set(MOVEIT_LIB_NAME moveit_utils)

add_library(${MOVEIT_LIB_NAME}
  src/lexical_casts.cpp
  src/message_checks.cpp
  src/moveit_error_code.cpp
  src/xmlrpc_casts.cpp
)
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")


set(MOVEIT_TEST_LIB_NAME moveit_test_utils)
add_library(${MOVEIT_TEST_LIB_NAME}
  src/robot_model_test_utils.cpp
)
add_dependencies(${MOVEIT_TEST_LIB_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MOVEIT_TEST_LIB_NAME} moveit_robot_model ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_TEST_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
    ${MOVEIT_TEST_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
