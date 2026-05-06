set(MOVEIT_LIB_NAME moveit_pick_place_planner)

add_library(${MOVEIT_LIB_NAME}
  src/pick_place_params.cpp
  src/manipulation_pipeline.cpp
  src/reachable_valid_pose_filter.cpp
  src/approach_and_translate_stage.cpp
  src/plan_stage.cpp
  src/pick_place.cpp
  src/pick.cpp
  src/place.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}) # don't build until necessary msgs are available

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
