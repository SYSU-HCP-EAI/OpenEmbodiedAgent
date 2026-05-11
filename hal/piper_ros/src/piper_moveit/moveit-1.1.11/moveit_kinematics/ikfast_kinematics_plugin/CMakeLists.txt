set(MOVEIT_LIB_NAME moveit_ikfast_kinematics_plugin)

#############
## Install ##
#############

install(
  PROGRAMS
    scripts/auto_create_ikfast_moveit_plugin.sh
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/create_ikfast_moveit_plugin.py
    scripts/round_collada_numbers.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    templates
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/ikfast_kinematics_plugin
)
