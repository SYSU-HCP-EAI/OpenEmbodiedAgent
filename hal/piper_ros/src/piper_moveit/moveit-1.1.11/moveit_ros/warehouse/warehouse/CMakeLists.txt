set(MOVEIT_LIB_NAME moveit_warehouse)

add_library(${MOVEIT_LIB_NAME}
  src/moveit_message_storage.cpp
  src/planning_scene_storage.cpp
  src/planning_scene_world_storage.cpp
  src/constraints_storage.cpp
  src/trajectory_constraints_storage.cpp
  src/state_storage.cpp
  src/warehouse_connector.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_warehouse_broadcast src/broadcast.cpp)
target_link_libraries(moveit_warehouse_broadcast ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES} )

add_executable(moveit_save_to_warehouse src/save_to_warehouse.cpp)
target_link_libraries(moveit_save_to_warehouse ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_executable(moveit_warehouse_import_from_text src/import_from_text.cpp)
target_link_libraries(moveit_warehouse_import_from_text ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_executable(moveit_warehouse_save_as_text src/save_as_text.cpp)
target_link_libraries(moveit_warehouse_save_as_text ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_executable(moveit_init_demo_warehouse src/initialize_demo_db.cpp)
target_link_libraries(moveit_init_demo_warehouse ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_executable(moveit_warehouse_services src/warehouse_services.cpp)
target_link_libraries(moveit_warehouse_services ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS
    moveit_save_to_warehouse
    moveit_warehouse_broadcast
    moveit_warehouse_import_from_text
    moveit_warehouse_save_as_text
    moveit_init_demo_warehouse
    moveit_warehouse_services
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
