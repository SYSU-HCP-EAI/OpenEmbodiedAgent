set(MOVEIT_LIB_NAME moveit_default_planning_request_adapter_plugins)

set(SOURCE_FILES
  src/empty.cpp
  src/fix_start_state_bounds.cpp
  src/fix_start_state_collision.cpp
  src/fix_start_state_path_constraints.cpp
  src/fix_workspace_bounds.cpp
  src/add_time_parameterization.cpp
  src/add_ruckig_traj_smoothing.cpp
  src/add_iterative_spline_parameterization.cpp
  src/add_time_optimal_parameterization.cpp
  src/resolve_constraint_frames.cpp
  src/limit_max_cartesian_link_speed.cpp
  )

add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
#ruckig requires C++17
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES CXX_STANDARD 17)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_list_request_adapter_plugins src/list.cpp)
target_link_libraries(moveit_list_request_adapter_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS moveit_list_request_adapter_plugins
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
