# Extract version number components
string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" MOVEIT_VERSION_MAJOR "${MOVEIT_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_MINOR "${MOVEIT_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_PATCH "${MOVEIT_VERSION}")

# Retrieve (active) branch name
execute_process(
	COMMAND git rev-parse --abbrev-ref HEAD
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	OUTPUT_VARIABLE MOVEIT_GIT_NAME
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
)

if("${MOVEIT_GIT_NAME}" STREQUAL "HEAD")
	# Retrieve any associated name (tag or branch)
	execute_process(
		COMMAND git describe --contains --all HEAD
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE MOVEIT_GIT_NAME
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)
endif()

# Retrieve (short) commit hash
execute_process(
	COMMAND git rev-parse --short HEAD
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	OUTPUT_VARIABLE MOVEIT_GIT_COMMIT_HASH
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
)

# Retrieve all tags
execute_process(
	COMMAND git tag --points-at HEAD
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	OUTPUT_VARIABLE GIT_TAGS
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
)

# split GIT_TAGS into list
string(REPLACE "\n" ";" GIT_TAGS "${GIT_TAGS}")
list(FIND GIT_TAGS "${MOVEIT_VERSION}" _index)

if(MOVEIT_GIT_COMMIT_HASH AND _index LESS 0) # MOVEIT_VERSION is not a tag at HEAD
	# increase patch number
	math(EXPR MOVEIT_VERSION_PATCH "${MOVEIT_VERSION_PATCH}+1")
	set(MOVEIT_VERSION_EXTRA "-devel")
endif()

set(MOVEIT_VERSION "${MOVEIT_VERSION_MAJOR}.${MOVEIT_VERSION_MINOR}.${MOVEIT_VERSION_PATCH}${MOVEIT_VERSION_EXTRA}")
message(STATUS " *** Building MoveIt ${MOVEIT_VERSION} ***")

configure_file("version.h.in" "${VERSION_FILE_PATH}/moveit/version.h")
