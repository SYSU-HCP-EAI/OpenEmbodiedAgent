def clear_stage_by_prim_path(prim_path: str = None) -> None:
    """Deletes all prims in the stage without populating the undo command buffer

    Args:
        prim_path (str, optional): path of the stage. Defaults to None.
    """
    # Note: Need to import this here to prevent circular dependencies.
    from omni.isaac.core.utils.prims import (
        get_all_matching_child_prims,
        get_prim_path,
        get_prim_type_name,
        is_prim_ancestral,
        is_prim_hidden_in_stage,
        is_prim_no_delete,
    )
    from omni.usd.commands import DeletePrimsCommand

    def default_predicate(path: str):
        # prim = get_prim_at_path(prim_path)
        # skip prims that we cannot delete
        if is_prim_no_delete(path):
            return False
        if is_prim_hidden_in_stage(path):
            return False
        if is_prim_ancestral(path):
            return False
        if path == '/':
            return False
        if get_prim_type_name(prim_path=path) == 'PhysicsScene':
            return False
        if path == '/World':
            return False
        # Don't remove any /Render prims as that can cause crashes
        if path.startswith('/Render'):
            return False
        return True

    prims = get_all_matching_child_prims(prim_path, default_predicate)
    prim_paths_to_delete = [get_prim_path(prim) for prim in prims]
    DeletePrimsCommand(prim_paths_to_delete).do()
