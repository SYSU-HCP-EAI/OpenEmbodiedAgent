#!/bin/bash

# Create an ikfast plugin package for MoveIt from a given robot URDF
# This includes:
# - conversion of URDF to Collada
# - running openrave from a docker image to create the actual ikfast solver
# - running create_ikfast_moveit_plugin.py to create the corresponding plugin package

# TODO: Would be nice to integrate this functionality directly into create_ikfast_moveit_plugin.py

set -e  # fail on error

function print_help {
   echo "$(basename $0) [--help|-h] [--quiet|-q] [--keep] [--name|-n] [--pkg|-p] [--iktype|-t] <input> <group> <base> <eef>"
   echo "  input          .urdf, .dae, or .cpp input file"
   echo "                 File type determines required processing stages."
   echo "  group          planning group"
   echo "  base           name of base link"
   echo "  eef            name of end-effector link"
   echo "--quiet          suppress output of OpenRave"
   echo "--keep           keep intermediate results, i.e. don't delete temporary folder on exit"
   echo "--name <robot>   Robot name, default extracted from urdf"
   echo "--pkg  <name>    Package name, default: <robot>_<planning group>_ikfast_plugin"
   echo "--iktype <type>  OpenRave kinematics type [must be one of Direction3D, Transform6D, Rotation3D, TranslationDirection5D, TranslationAxisAngle4D, Ray4D, TranslationXYOrientation3D, TranslationXY2D, Translation3D,  Rotation3D, LookAt3D, Direction3D, or Direction3D default: Transform6D]"
}

function parse_options {
   # set defaults (if not set in environment yet)
   IK_TYPE=${IK_TYPE:-Transform6D}
   QUIET=${QUIET:-0}
   KEEP_RESULTS=${KEEP_RESULTS:-0}
   while true ; do
      case "$1" in
         --help|-h)
            print_help
            exit 0
            ;;
         --quiet|-q)
            QUIET=1
            ;;
         --keep)
            KEEP_RESULTS=1
            ;;
         --iktype|-t)
            IK_TYPE=$2; shift
            ;;
         --name|-n)
            ROBOT_NAME=$2; shift
            ;;
         --pkg|-p)
            PKG_NAME=$2; shift
            ;;
         -*)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
         *) break ;;
      esac
      shift
   done
   if [ $# -ne 4 ] ; then
      echo "Expecting 4 positional arguments! Got $#: $*"
      print_help
      exit 1
   fi
   INPUT=$1
   PLANNING_GROUP=$2
   BASE_LINK=$3
   EEF_LINK=$4
}

function set_option_defaults {
   # ROBOT_NAME is auto-extracted from URDF or otherwise needs to be specified as an option
   if [ -z "$ROBOT_NAME" ] ; then
      echo "Undefined robot name. Please specify option --name".
      exit 1
   fi
   # Define default PKG_NAME if not yet defined
   PKG_NAME=${PKG_NAME:-${ROBOT_NAME}_${PLANNING_GROUP}_ikfast_plugin}
}

function cleanup {
   rm -rf "$TMP_DIR"
}

function run_quiet {
   # When running in quiet mode, save stdout as 3, then redirect stdout to $TMP_DIR/ikfast.log
   if [ "$QUIET" == "1" ] ; then
      local STDOUT=3
      local STDERR=4
      exec 3>&1 1>"$TMP_DIR/ikfast.log"
      exec 4>&2 2>&1
   fi

   set +e
   "$@"
   ret=$?
   set -e

   # Restore stdout + stderr
   exec 1>&${STDOUT:-1}  # restore stdout
   exec 2>&${STDERR:-2}  # restore stderr

   if [ $ret != 0 ] ; then
      echo "'$*' failed with exec code $ret:"
      test "$QUIET" == "1" && cat "$TMP_DIR/ikfast.log"
   fi

   return $ret
}

function build_docker_image {
   test "$__DOCKER_BUILT" == "1" && return

   echo "Building docker image"
   cat <<EOF > $TMP_DIR/Dockerfile
FROM personalrobotics/ros-openrave
# Update ROS keys (https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454)
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    apt-get update && \
    apt-get install -y --no-install-recommends python-pip build-essential liblapack-dev ros-indigo-collada-urdf && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
# enforce a specific version of sympy, which is known to work with OpenRave
RUN pip install git+https://github.com/sympy/sympy.git@sympy-0.7.1
EOF
   run_quiet docker build -t fixed-openrave $TMP_DIR
   echo "Successfully built docker image."
   __DOCKER_BUILT=1
}

function extract_robot_name {
   local robot_name=$(check_urdf "$1" 2> /dev/null | grep "^robot name is: ")
   robot_name=${robot_name##robot name is: }
   test -z "$robot_name" && return
   if [ -n "$ROBOT_NAME" -a "$ROBOT_NAME" != "$robot_name" ]; then
      echo "Robot name in urdf ($robot_name) differs from specified name ($ROBOT_NAME)"
   else
      ROBOT_NAME="$robot_name"
   fi
}

function create_dae_file {
   echo "Try converting urdf to Collada directly"
   if ! rosrun collada_urdf urdf_to_collada "$INPUT" "$DAE_FILE" 2> /dev/null ; then
      # When this failed, run docker
      echo "Failed. Converting urdf to Collada (in docker)"
      build_docker_image
      cp "$INPUT" "$TMP_DIR/robot.urdf"
      run_quiet docker run --rm --user $(id -u):$(id -g) -v $TMP_DIR:/input --workdir /input -e HOME=/input \
             fixed-openrave:latest rosrun collada_urdf urdf_to_collada robot.urdf robot.dae
   fi
}

function create_solver {
   # create wrapper.xml file, needed by OpenRave
   cat <<EOF > $TMP_DIR/wrapper.xml
<robot file="robot.dae">
  <Manipulator name="$ROBOT_NAME">
    <base>$BASE_LINK</base>
    <effector>$EEF_LINK</effector>
  </Manipulator>
</robot>
EOF

   build_docker_image

   # Assemble openrave command
   cmd="openrave0.9.py --database inversekinematics --robot=/input/wrapper.xml --iktype=$IK_TYPE --iktests=1000"
   echo "Running $cmd"

   # run $cmd in docker as current user, outputting files to $TMP_DIR/.openrave
   run_quiet docker run --rm --user $(id -u):$(id -g) \
      -v $TMP_DIR:/input --workdir /input -e HOME=/input \
      fixed-openrave:latest $cmd

   # update INPUT to generated .cpp
   INPUT=$(ls -1 $TMP_DIR/.openrave/*/*.cpp 2> /dev/null)
   if [ -n "$INPUT" ] ; then
      echo "Created $INPUT"
   else
      echo "Failed to create ikfast solver"
      exit 1
   fi
}

function create_plugin {
   echo
   echo "Running $(dirname $0)/create_ikfast_moveit_plugin.py \"$ROBOT_NAME\" \"$PLANNING_GROUP\" \"$PKG_NAME\" \"$BASE_LINK\" \"$EEF_LINK\" \"$INPUT\""
   $(dirname "$0")/create_ikfast_moveit_plugin.py "$ROBOT_NAME" "$PLANNING_GROUP" "$PKG_NAME" "$BASE_LINK" "$EEF_LINK" "$INPUT"
}

### main program ###

parse_options $*

# Create a temporary directory to operate in
TMP_DIR=$(mktemp -d --tmpdir ikfast.XXXXXX)
DAE_FILE="$TMP_DIR/robot.dae"
if [ "$KEEP_RESULTS" == "1" ] ; then
   echo "Storing intermediate results in: $TMP_DIR"
else
   # Register cleanup function to be called on EXIT signal
   trap cleanup EXIT
fi

# Depending on input file type, select the required processing stages
while true ; do
   filename=$(basename -- "$INPUT")
   extension=$(echo "${filename##*.}" | tr '[:upper:]' '[:lower:]')
   case "$extension" in
      xacro)
         URDF="$TMP_DIR/${filename%.*}.urdf"
         xacro "$INPUT" > "$URDF"
         INPUT="$URDF"
         ;;
      urdf)  # create .dae from .urdf
         extract_robot_name "$INPUT"
         create_dae_file
         INPUT="$DAE_FILE"
         ;;
      dae)  # create solver .cpp
         if ! [ "$INPUT" -ef "$DAE_FILE" ] ; then
            cp "$INPUT" "$DAE_FILE"
         fi
         set_option_defaults
         create_solver
         ;;
      cpp)  # create wrapper plugin
         set_option_defaults
         create_plugin
         break
         ;;
      *)
         echo "Unknown input file type '.$extension'. Expecting .urdf, .dae, or .cpp."
         exit 1
         ;;
   esac
done
