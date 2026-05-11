set(MOVEIT_LIB_NAME moveit_robot_interface)

add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_robot_interface.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  moveit_common_planning_interface_objects moveit_py_bindings_tools)
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_robot_interface PREFIX "")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")
if(WIN32)
  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif(WIN32)

install(TARGETS ${MOVEIT_LIB_NAME}_python
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
