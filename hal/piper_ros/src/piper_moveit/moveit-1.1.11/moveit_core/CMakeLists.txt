cmake_minimum_required(VERSION 3.1.3)
project(moveit_core)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # Silent these warnings. They are too often false-positives.
  message("TODO: Analyse and fix gcc's maybe-uninitialized warnings")
  add_compile_options(-Wno-maybe-uninitialized)
endif()
include(cmake/moveit.cmake)
moveit_build_options()

find_package(Boost REQUIRED system filesystem date_time thread iostreams regex ${EXTRA_BOOST_COMPONENTS})
find_package(Eigen3 REQUIRED)

# TODO: Move collision detection into separate packages
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Bullet 2.87)

# TODO(j-petit): Version check can be dropped when Xenial reaches end-of-life
if(BULLET_FOUND)
  set(BULLET_ENABLE "BULLET")
  set(BULLET_LIB "moveit_collision_detection_bullet")
  set(BULLET_INC "collision_detection_bullet/include")
  message(STATUS "Compiling with Bullet")
else()
  message(STATUS "Version of Bullet too old or not available: disabling Bullet collision detection plugin. Try using Ubuntu 18.04 or later.")
endif()

find_package(fcl QUIET)
if (fcl_FOUND)
  # convert cmake target to variables for catkin_package DEPENDS
  if(TARGET ${FCL_LIBRARIES})
    get_target_property(LIBFCL_LOCATION ${FCL_LIBRARIES} LOCATION)
    get_target_property(LIBFCL_INTERFACE_LINK_LIBRARIES ${FCL_LIBRARIES} INTERFACE_LINK_LIBRARIES)
    get_target_property(LIBFCL_INCLUDE_DIRS ${FCL_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
    set(LIBFCL_LIBRARIES ${LIBFCL_LOCATION} ${LIBFCL_INTERFACE_LINK_LIBRARIES})
  else()
    set(LIBFCL_LIBRARIES ${FCL_LIBRARIES})
    set(LIBFCL_INCLUDE_DIRS ${FCL_INCLUDE_DIRS})
  endif()
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBFCL_PC REQUIRED fcl)
  set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
  # find *absolute* paths to LIBFCL_LIBRARIES
  set(LIBFCL_LIBRARIES)
  foreach(_lib ${LIBFCL_PC_LIBRARIES})
    find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
    list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
  endforeach()
endif()

find_package(octomap REQUIRED)
find_package(ruckig REQUIRED)
set(ruckig_LIBRARIES "ruckig::ruckig")
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(catkin REQUIRED
COMPONENTS
  tf2_eigen
  tf2_geometry_msgs
  eigen_stl_containers
  geometric_shapes
  geometry_msgs
  kdl_parser
  moveit_msgs
  octomap_msgs
  random_numbers
  roslib
  rostime
  rosconsole
  sensor_msgs
  shape_msgs
  srdfdom
  std_msgs
  trajectory_msgs
  urdf
  visualization_msgs
  xmlrpcpp
  pybind11_catkin
  pluginlib
)

catkin_python_setup()

set(VERSION_FILE_PATH "${CATKIN_DEVEL_PREFIX}/include")
# Pass the folder of the generated version.h to catkin_package() for export in devel-space
# This is how gencpp adds the folder of generated message code to the include dirs, see:
#   https://github.com/ros/gencpp/blob/e5acaf6/cmake/gencpp-extras.cmake.em#L51-L54
list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${VERSION_FILE_PATH})
file(MAKE_DIRECTORY "${VERSION_FILE_PATH}/moveit")

set(THIS_PACKAGE_INCLUDE_DIRS
    background_processing/include
    exceptions/include
    backtrace/include
    collision_detection/include
    collision_detection_fcl/include
    ${BULLET_INC}
    constraint_samplers/include
    controller_manager/include
    distance_field/include
    collision_distance_field/include
    dynamics_solver/include
    kinematics_base/include
    kinematics_metrics/include
    robot_model/include
    transforms/include
    robot_state/include
    robot_trajectory/include
    kinematic_constraints/include
    macros/include
    planning_interface/include
    planning_request_adapter/include
    planning_scene/include
    profiler/include
    python/tools/include
    sensor_manager/include
    trajectory_processing/include
    utils/include
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    moveit_exceptions
    moveit_background_processing
    moveit_kinematics_base
    moveit_robot_model
    moveit_transforms
    moveit_robot_state
    moveit_robot_trajectory
    moveit_planning_interface
    moveit_collision_detection
    moveit_collision_detection_fcl
    ${BULLET_LIB}
    moveit_kinematic_constraints
    moveit_planning_scene
    moveit_constraint_samplers
    moveit_planning_request_adapter
    moveit_profiler
    moveit_python_tools
    moveit_trajectory_processing
    moveit_distance_field
    moveit_collision_distance_field
    moveit_kinematics_metrics
    moveit_dynamics_solver
    moveit_utils
    moveit_test_utils
  CATKIN_DEPENDS
    eigen_stl_containers
    geometric_shapes
    geometry_msgs
    kdl_parser
    moveit_msgs
    octomap_msgs
    random_numbers
    sensor_msgs
    shape_msgs
    srdfdom
    std_msgs
    tf2_eigen
    tf2_geometry_msgs
    trajectory_msgs
    visualization_msgs
  DEPENDS
    Boost
    EIGEN3
    LIBFCL
    OCTOMAP
    ruckig
    urdfdom
    urdfdom_headers
    ${BULLET_ENABLE}
  CFG_EXTRAS
    moveit.cmake
    )

if (WIN32)
  # for msvc 2017 compatibility
  add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIRS}
                           ${EIGEN3_INCLUDE_DIRS}
                           ${urdfdom_INCLUDE_DIRS}
                           ${urdfdom_headers_INCLUDE_DIRS}
                           ${LIBFCL_INCLUDE_DIRS}
                           ${BULLET_INCLUDE_DIRS}
                           ${OCTOMAP_INCLUDE_DIRS}
                    )

#catkin_lint: ignore_once external_directory  (${VERSION_FILE_PATH})
include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${VERSION_FILE_PATH})

add_subdirectory(version)
add_subdirectory(macros)
add_subdirectory(backtrace)
add_subdirectory(exceptions)
add_subdirectory(profiler)
add_subdirectory(utils)
add_subdirectory(background_processing)
add_subdirectory(kinematics_base)
add_subdirectory(controller_manager)
add_subdirectory(sensor_manager)
add_subdirectory(robot_model)
add_subdirectory(transforms)
add_subdirectory(robot_state)
add_subdirectory(robot_trajectory)
add_subdirectory(collision_detection)
add_subdirectory(collision_detection_fcl)
add_subdirectory(kinematic_constraints)
add_subdirectory(planning_scene)
add_subdirectory(constraint_samplers)
add_subdirectory(planning_interface)
add_subdirectory(planning_request_adapter)
add_subdirectory(trajectory_processing)
add_subdirectory(distance_field)
add_subdirectory(collision_distance_field)
add_subdirectory(kinematics_metrics)
add_subdirectory(dynamics_solver)

add_subdirectory(python)
set(pymoveit_libs
  moveit_collision_detection
  moveit_kinematic_constraints
  moveit_planning_scene
  moveit_python_tools
  moveit_robot_model
  moveit_robot_state
  moveit_transforms
)

pybind_add_module(pymoveit_core
    python/pymoveit_core.cpp
    collision_detection/src/pycollision_detection.cpp
    robot_model/src/pyrobot_model.cpp
    robot_state/src/pyrobot_state.cpp
    transforms/src/pytransforms.cpp
    planning_scene/src/pyplanning_scene.cpp
    kinematic_constraints/src/pykinematic_constraint.cpp
)
target_include_directories(pymoveit_core SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(pymoveit_core PRIVATE ${pymoveit_libs} ${catkin_LIBRARIES})

#catkin_lint: ignore_once undefined_target (pymoveit_core is defined by pybind_add_module)
install(TARGETS pymoveit_core LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

if(BULLET_ENABLE)
  add_subdirectory(collision_detection_bullet)
else()
  install(FILES collision_detection_bullet/empty_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} RENAME collision_detector_bullet_description.xml)
endif()
