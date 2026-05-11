set(MOVEIT_LIB_NAME moveit_move_group_pick_place_capability)

include_directories(src)

add_library(${MOVEIT_LIB_NAME}
  src/pick_place_action_capability.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_pick_place_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
