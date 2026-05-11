set(MOVEIT_LIB_NAME moveit_move_group_interface)

add_library(${MOVEIT_LIB_NAME} src/move_group_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} moveit_common_planning_interface_objects moveit_planning_scene_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_move_group.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} eigenpy::eigenpy ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_py_bindings_tools)
add_dependencies(${MOVEIT_LIB_NAME}_python ${catkin_EXPORTED_TARGETS})
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_move_group_interface PREFIX "")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")
if(WIN32)
  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif(WIN32)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME}_python
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
