set(MOVEIT_LIB_NAME moveit_robot_model)

add_library(${MOVEIT_LIB_NAME}
  src/aabb.cpp
  src/fixed_joint_model.cpp
  src/floating_joint_model.cpp
  src/joint_model.cpp
  src/joint_model_group.cpp
  src/link_model.cpp
  src/planar_joint_model.cpp
  src/prismatic_joint_model.cpp
  src/revolute_joint_model.cpp
  src/robot_model.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_profiler moveit_exceptions moveit_kinematics_base ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_robot_model test/test.cpp)
  target_link_libraries(test_robot_model moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
