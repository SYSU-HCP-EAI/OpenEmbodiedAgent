# To install pxr, use `pip install usd-core`
"""
    Collider
"""


def set_collider_(mesh, approx=None, init_state=True):
    """set collider for a mesh
    Args:
    - mesh: Usd.Prim -> Mesh
    - approx: Convex approximation method, e.g. ConvenDecomposition, ConvexHull
    - init_state: init state of collider(whether activate)
    """
    from pxr import UsdPhysics

    if approx is None:
        approx = UsdPhysics.Tokens.convexDecomposition
    if mesh.GetTypeName() == 'Mesh' and UsdPhysics.CollisionAPI.CanApply(mesh):
        collider = UsdPhysics.CollisionAPI.Apply(mesh)
        meshcollider = UsdPhysics.MeshCollisionAPI.Apply(mesh)  # introduce meshcollider to set approx
        meshcollider.GetApproximationAttr().Set(approx)
        collider.GetCollisionEnabledAttr().Set(init_state)


def set_collider(item, approx=None, init_state=True):
    """set collider for all meshes of an Usd.Prim
    Args:
    - item: Usd.Prim, e.g. Xform
    - init_state: init state of collider(whether activate)
    """
    from pxr import UsdPhysics

    if approx is None:
        approx = UsdPhysics.Tokens.convexDecomposition
    if len(item.GetChildren()) == 0:
        set_collider_(item, approx=approx, init_state=init_state)
    for i in item.GetChildren():
        set_collider(i, approx=approx, init_state=init_state)


def activate_collider_(mesh):
    """enable collision property of a mesh
    Args:
    - mesh: Usd.Prim -> Mesh
    """
    if mesh.GetTypeName() == 'Mesh' and mesh.GetAttribute('physics:collisionEnabled').Get() == 0:
        mesh.GetAttribute('physics:collisionEnabled').Set(1)


def activate_collider(item):
    """enable collision property of all meshes of an Usd.Prim
    Args:
    - item: Usd.Prim, e.g. Xform
    """
    if len(item.GetChildren()) == 0:
        activate_collider_(item)
    for i in item.GetChildren():
        activate_collider(i)


def deactivate_collider_(mesh):
    """disable collision property of a mesh
    Args:
    - mesh: Usd.Prim -> Mesh
    """
    if mesh.GetTypeName() == 'Mesh' and mesh.GetAttribute('physics:collisionEnabled').Get() == 1:
        mesh.GetAttribute('physics:collisionEnabled').Set(0)


def deactivate_collider(item):
    """disable collision property of a mesh
    Args:
    - mesh: Usd.Prim -> Mesh
    """
    if len(item.GetChildren()) == 0:
        deactivate_collider_(item)
    for i in item.GetChildren():
        deactivate_collider(i)


def remove_collider_(mesh):
    from pxr import UsdPhysics

    if mesh.GetTypeName() == 'Mesh':
        mesh.RemoveAPI(UsdPhysics.CollisionAPI)
        mesh.RemoveAPI(UsdPhysics.MeshCollisionAPI)


def remove_collider(item):
    if len(item.GetChildren()) == 0:
        remove_collider_(item)
    for i in item.GetChildren():
        remove_collider(i)
