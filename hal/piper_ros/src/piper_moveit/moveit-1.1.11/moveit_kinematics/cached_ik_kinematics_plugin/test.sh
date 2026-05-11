#!/bin/bash
for robot in abb_irb2400 abb_irb6640 cob fetch hironx nao nextage pepper pr2 ur5_robotiq85 vs060; do
    for kinematics in kdl kdl_cached trac trac_cached; do
        roslaunch moveit_kinematics measure_ik_call_cost.launch robot:=${robot} num:=10000 kinematics:=true kinematics_path:=src/moveit/moveit_kinematics/cached_ik_kinematics_plugin/config/${kinematics}.yaml 2>&1 | grep Summary | cut -f 7-10 -d' ' | sed "s/^/${robot} ${kinematics} /"
    done
    if [[ ${robot} == "ur5_robotiq85" ]]; then
        for kinematics in ur5 ur5_cached; do
            roslaunch moveit_kinematics measure_ik_call_cost.launch robot:=${robot} num:=100 kinematics:=true kinematics_path:=src/moveit/moveit_kinematics/cached_ik_kinematics_plugin/config/${kinematics}.yaml 2>&1 | grep Summary | cut -f 7-10 -d' ' | sed "s/^/${robot} ${kinematics} /"
        done
    fi
done
