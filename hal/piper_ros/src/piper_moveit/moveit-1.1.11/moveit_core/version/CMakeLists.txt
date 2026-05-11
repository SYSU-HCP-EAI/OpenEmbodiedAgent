# Generate and install version.h

# https://stackoverflow.com/questions/13920072/how-to-always-run-command-when-building-regardless-of-any-dependency
add_custom_command(
	OUTPUT ${VERSION_FILE_PATH}/moveit/version.h always_rebuild
	COMMAND ${CMAKE_COMMAND}
	-DVERSION_FILE_PATH="${VERSION_FILE_PATH}"
	-DMOVEIT_VERSION="${${PROJECT_NAME}_VERSION}"
	-P ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(version_h
	DEPENDS always_rebuild
	COMMENT "Generating version.h"
)

add_executable(moveit_version version.cpp)
add_dependencies(moveit_version version_h)

# catkin_lint: ignore_once external_file
install(FILES "${VERSION_FILE_PATH}/moveit/version.h" DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/moveit)
install(TARGETS moveit_version RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
