from typing import Any, Dict

import numpy as np

from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.rigid_body import IRigidBody


def get_rigidbody_status(rigid_body: IRigidBody) -> Dict[str, Any]:
    """Get the current status of a RigidPrim"""
    return {
        'transforms': rigid_body._rigid_prim_view._physics_view.get_transforms(),
        'velocities': rigid_body._rigid_prim_view._physics_view.get_velocities(),
        'masses': rigid_body._rigid_prim_view._physics_view.get_masses(),
        'coms': rigid_body._rigid_prim_view._physics_view.get_coms(),
        'inertias': rigid_body._rigid_prim_view._physics_view.get_inertias(),
        'disable_gravities': rigid_body._rigid_prim_view._physics_view.get_disable_gravities(),
        'disable_simulations': rigid_body._rigid_prim_view._physics_view.get_disable_simulations(),
        'material_properties': rigid_body._rigid_prim_view._physics_view.get_material_properties(),
        'contact_offsets': rigid_body._rigid_prim_view._physics_view.get_contact_offsets(),
        'rest_offsets': rigid_body._rigid_prim_view._physics_view.get_rest_offsets(),
    }


def set_rigidbody_status(rigid_body: IRigidBody, status: Dict):
    """Set the current status of a RigidPrim"""
    if not status:
        return
    rigid_body._rigid_prim_view._physics_view.set_transforms(status['transforms'], indices=np.array([0]))
    rigid_body._rigid_prim_view._physics_view.set_velocities(status['velocities'], indices=np.array([0]))
    rigid_body._rigid_prim_view._physics_view.set_masses(status['masses'], indices=np.array([0]))
    rigid_body._rigid_prim_view._physics_view.set_coms(status['coms'], indices=np.array([0]))
    rigid_body._rigid_prim_view._physics_view.set_inertias(status['inertias'], indices=np.array([0]))
    rigid_body._rigid_prim_view._physics_view.set_disable_gravities(status['disable_gravities'], indices=np.array([0]))
    rigid_body._rigid_prim_view._physics_view.set_disable_simulations(
        status['disable_simulations'], indices=np.array([0])
    )
    rigid_body._rigid_prim_view._physics_view.set_material_properties(
        status['material_properties'], indices=np.array([0])
    )
    rigid_body._rigid_prim_view._physics_view.set_contact_offsets(status['contact_offsets'], indices=np.array([0]))
    rigid_body._rigid_prim_view._physics_view.set_rest_offsets(status['rest_offsets'], indices=np.array([0]))


def get_articulation_status(articulation: IArticulation) -> Dict[str, Any]:
    """Get the current status of an Articulation"""
    return {
        'dof_limits': articulation._articulation_view._physics_view.get_dof_limits(),
        'dof_stiffnesses': articulation._articulation_view._physics_view.get_dof_stiffnesses(),
        'dof_dampings': articulation._articulation_view._physics_view.get_dof_dampings(),
        'dof_max_forces': articulation._articulation_view._physics_view.get_dof_max_forces(),
        'dof_friction_coefficients': articulation._articulation_view._physics_view.get_dof_friction_coefficients(),
        'dof_max_velocities': articulation._articulation_view._physics_view.get_dof_max_velocities(),
        'dof_armatures': articulation._articulation_view._physics_view.get_dof_armatures(),
        'root_transforms': articulation._articulation_view._physics_view.get_root_transforms(),
        'root_velocities': articulation._articulation_view._physics_view.get_root_velocities(),
        'dof_positions': articulation._articulation_view._physics_view.get_dof_positions(),
        'dof_velocities': articulation._articulation_view._physics_view.get_dof_velocities(),
        'dof_actuation_forces': articulation._articulation_view._physics_view.get_dof_actuation_forces(),
        'dof_position_targets': articulation._articulation_view._physics_view.get_dof_position_targets(),
        'dof_velocity_targets': articulation._articulation_view._physics_view.get_dof_velocity_targets(),
        'masses': articulation._articulation_view._physics_view.get_masses(),
        'coms': articulation._articulation_view._physics_view.get_coms(),
        'inertias': articulation._articulation_view._physics_view.get_inertias(),
        'disable_gravities': articulation._articulation_view._physics_view.get_disable_gravities(),
        'material_properties': articulation._articulation_view._physics_view.get_material_properties(),
        'contact_offsets': articulation._articulation_view._physics_view.get_contact_offsets(),
        'rest_offsets': articulation._articulation_view._physics_view.get_rest_offsets(),
    }


def set_articulation_status(articulation: IArticulation, status: Dict):
    """Set the current status of an Articulation"""
    articulation._articulation_view._physics_view.set_dof_limits(data=status['dof_limits'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_stiffnesses(data=status['dof_stiffnesses'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_dampings(data=status['dof_dampings'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_max_forces(data=status['dof_max_forces'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_friction_coefficients(
        data=status['dof_friction_coefficients'], indices=[0]
    )
    articulation._articulation_view._physics_view.set_dof_max_velocities(data=status['dof_max_velocities'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_armatures(data=status['dof_armatures'], indices=[0])
    articulation._articulation_view._physics_view.set_root_transforms(data=status['root_transforms'], indices=[0])
    articulation._articulation_view._physics_view.set_root_velocities(data=status['root_velocities'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_positions(data=status['dof_positions'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_velocities(data=status['dof_velocities'], indices=[0])
    articulation._articulation_view._physics_view.set_dof_actuation_forces(
        data=status['dof_actuation_forces'], indices=[0]
    )
    articulation._articulation_view._physics_view.set_dof_position_targets(
        data=status['dof_position_targets'], indices=[0]
    )
    articulation._articulation_view._physics_view.set_dof_velocity_targets(
        data=status['dof_velocity_targets'], indices=[0]
    )
    articulation._articulation_view._physics_view.set_masses(data=status['masses'], indices=[0])
    articulation._articulation_view._physics_view.set_coms(data=status['coms'], indices=[0])
    articulation._articulation_view._physics_view.set_inertias(data=status['inertias'], indices=[0])
    articulation._articulation_view._physics_view.set_disable_gravities(data=status['disable_gravities'], indices=[0])
    articulation._articulation_view._physics_view.set_material_properties(
        data=status['material_properties'], indices=[0]
    )
    articulation._articulation_view._physics_view.set_contact_offsets(data=status['contact_offsets'], indices=[0])
    articulation._articulation_view._physics_view.set_rest_offsets(data=status['rest_offsets'], indices=[0])
