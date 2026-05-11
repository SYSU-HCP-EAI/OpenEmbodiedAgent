set(MOVEIT_LIB_NAME moveit_constraint_samplers)

add_library(${MOVEIT_LIB_NAME}
  src/constraint_sampler.cpp
  src/constraint_sampler_manager.cpp
  src/constraint_sampler_tools.cpp
  src/default_constraint_samplers.cpp
  src/union_constraint_sampler.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_state
  moveit_kinematic_constraints
  moveit_kinematics_base
  moveit_planning_scene
  ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(orocos_kdl REQUIRED)
  find_package(angles REQUIRED)
  find_package(tf2_kdl REQUIRED)
  find_package(rostest REQUIRED)
  include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS} ${angles_INCLUDE_DIRS} ${tf2_kdl_INCLUDE_DIRS})

  add_rostest_gtest(test_constraint_samplers
    test/constraint_samplers.test
    test/test_constraint_samplers.cpp
    test/pr2_arm_kinematics_plugin.cpp
    test/pr2_arm_ik.cpp
  )

  target_link_libraries(test_constraint_samplers
    moveit_test_utils
    gmock_main
    ${MOVEIT_LIB_NAME}
    ${catkin_LIBRARIES}
    ${angles_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
  )
endif()
