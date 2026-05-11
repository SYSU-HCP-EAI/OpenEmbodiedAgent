def create_joint(
    prim_path,
    joint_type,
    body0=None,
    body1=None,
    enabled=True,
    joint_frame_in_parent_frame_pos=None,
    joint_frame_in_parent_frame_quat=None,
    joint_frame_in_child_frame_pos=None,
    joint_frame_in_child_frame_quat=None,
    break_force=None,
    break_torque=None,
):
    """
    Creates a joint between @body0 and @body1 of specified type @joint_type

    Args:
        prim_path (str): absolute path to where the joint will be created
        joint_type (str or JointType): type of joint to create. Valid options are:
            "FixedJoint", "Joint", "PrismaticJoint", "RevoluteJoint", "SphericalJoint"
                        (equivalently, one of JointType)
        body0 (str or None): absolute path to the first body's prim. At least @body0 or @body1 must be specified.
        body1 (str or None): absolute path to the second body's prim. At least @body0 or @body1 must be specified.
        enabled (bool): whether to enable this joint or not.
        joint_frame_in_parent_frame_pos (np.ndarray or None): relative position of the joint frame to the parent frame (body0).
        joint_frame_in_parent_frame_quat (np.ndarray or None): relative orientation of the joint frame to the parent frame (body0).
        joint_frame_in_child_frame_pos (np.ndarray or None): relative position of the joint frame to the child frame (body1).
        joint_frame_in_child_frame_quat (np.ndarray or None): relative orientation of the joint frame to the child frame (body1).
        break_force (float or None): break force for linear dofs, unit is Newton.
        break_torque (float or None): break torque for angular dofs, unit is Newton-meter.

    Returns:
        Usd.Prim: Created joint prim
    """
    import omni
    import pxr
    from omni.isaac.core import World

    world = World()
    stage = world.stage
    # Create the joint
    joint = getattr(pxr.UsdPhysics, joint_type).Define(stage, prim_path)

    # Possibly add body0, body1 targets
    if body0 is not None:
        assert omni.isaac.core.utils.prims.is_prim_path_valid(body0), f'Invalid body0 path specified: {body0}'
        joint.GetBody0Rel().SetTargets([pxr.Sdf.Path(body0)])
    if body1 is not None:
        assert omni.isaac.core.utils.prims.is_prim_path_valid(body1), f'Invalid body1 path specified: {body1}'
        joint.GetBody1Rel().SetTargets([pxr.Sdf.Path(body1)])

    # Get the prim pointed to at this path
    joint_prim = omni.isaac.core.utils.prims.get_prim_at_path(prim_path)

    # Apply joint API interface
    pxr.PhysxSchema.PhysxJointAPI.Apply(joint_prim)

    # We need to step rendering once to auto-fill the local pose before overwriting it.
    # Note that for some reason, if multi_gpu is used, this line will crash if create_joint is called during on_contact
    # callback, e.g. when an attachment joint is being created due to contacts.

    world.render()

    if joint_frame_in_parent_frame_pos is not None:
        joint_prim.GetAttribute('physics:localPos0').Set(pxr.Gf.Vec3f(*joint_frame_in_parent_frame_pos))
    if joint_frame_in_parent_frame_quat is not None:
        joint_prim.GetAttribute('physics:localRot0').Set(pxr.Gf.Quatf(*joint_frame_in_parent_frame_quat[[3, 0, 1, 2]]))
    if joint_frame_in_child_frame_pos is not None:
        joint_prim.GetAttribute('physics:localPos1').Set(pxr.Gf.Vec3f(*joint_frame_in_child_frame_pos))
    if joint_frame_in_child_frame_quat is not None:
        joint_prim.GetAttribute('physics:localRot1').Set(pxr.Gf.Quatf(*joint_frame_in_child_frame_quat[[3, 0, 1, 2]]))

    if break_force is not None:
        joint_prim.GetAttribute('physics:breakForce').Set(break_force)
    if break_torque is not None:
        joint_prim.GetAttribute('physics:breakTorque').Set(break_torque)

    # Possibly (un-/)enable this joint
    joint_prim.GetAttribute('physics:jointEnabled').Set(enabled)

    # We update the simulation now without stepping physics if sim is playing so we can bypass the snapping warning from PhysicsUSD
    # if world.is_playing():
    #     # with suppress_omni_log(channels=["omni.physx.plugin"]):
    #     world.pi.update_simulation(elapsedStep=0, currentTime=world.current_time)

    # Return this joint
    return joint_prim
