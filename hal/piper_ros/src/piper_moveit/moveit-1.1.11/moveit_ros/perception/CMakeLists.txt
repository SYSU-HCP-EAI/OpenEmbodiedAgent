cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_perception)

option(WITH_OPENGL "Build the parts that depend on OpenGL" ON)

find_package(Boost REQUIRED thread)

if(WITH_OPENGL)
  # Prefer newer vendor-specific OpenGL library
  if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
  endif()
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(GLUT REQUIRED)
  set(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
  set(perception_GL_INCLUDE_DIRS "mesh_filter/include" "depth_image_octomap_updater/include")
  set(SYSTEM_GL_INCLUDE_DIRS ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
endif(WITH_OPENGL)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  moveit_core
  roscpp
  rosconsole
  urdf
  message_filters
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  pluginlib
  image_transport
  object_recognition_msgs
  sensor_msgs
  moveit_msgs
  moveit_ros_occupancy_map_monitor
  moveit_ros_planning
  nodelet
)
moveit_build_options()

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV)

set(PACKAGE_LIBRARIES
  moveit_lazy_free_space_updater
  moveit_point_containment_filter
  moveit_pointcloud_octomap_updater_core
  moveit_semantic_world
)

if (WITH_OPENGL)
  list(APPEND PACKAGE_LIBRARIES
          moveit_mesh_filter
          moveit_depth_self_filter
          moveit_depth_image_octomap_updater)
endif(WITH_OPENGL)

catkin_package(
  INCLUDE_DIRS
    lazy_free_space_updater/include
    point_containment_filter/include
    pointcloud_octomap_updater/include
    semantic_world/include
    ${perception_GL_INCLUDE_DIRS}
  LIBRARIES
    ${PACKAGE_LIBRARIES}
  CATKIN_DEPENDS
    image_transport
    moveit_core
    moveit_msgs
    moveit_ros_occupancy_map_monitor
    moveit_ros_planning
    nodelet
    object_recognition_msgs
    roscpp
    sensor_msgs
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(lazy_free_space_updater/include
                    point_containment_filter/include
                    pointcloud_octomap_updater/include
                    semantic_world/include
                    ${perception_GL_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${SYSTEM_GL_INCLUDE_DIR}
                    ${X11_INCLUDE_DIR}
                    )

add_subdirectory(lazy_free_space_updater)
add_subdirectory(point_containment_filter)
add_subdirectory(pointcloud_octomap_updater)
if (WITH_OPENGL)
  add_subdirectory(mesh_filter)
  add_subdirectory(depth_image_octomap_updater)
endif(WITH_OPENGL)

add_subdirectory(semantic_world)

install(
  FILES
    pointcloud_octomap_updater_plugin_description.xml
    depth_image_octomap_updater_plugin_description.xml
    moveit_depth_self_filter.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
