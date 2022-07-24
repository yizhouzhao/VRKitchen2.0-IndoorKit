import omni
import pxr
import carb
from pxr import UsdPhysics, UsdShade, Gf
from omni.physx.scripts import physicsUtils
from omni.physx.scripts.utils import setCollider, setRigidBody, setStaticCollider, removeCollider

from ..param import IS_IN_ISAAC_SIM

if IS_IN_ISAAC_SIM:
    from omni.isaac.core.utils.semantics import add_update_semantics

def modify_game_obj_prim(prim):
    """
    modify game object attributes:
    if Bottle, add rigibody, physical material, and mass
    """
    print("modifyiing: " + prim.GetPath().pathString)
    if "Bottle" in prim.GetPath().pathString or "standalone" in prim.GetPath().pathString:
        """
        Set bottle rigidbox and physical material
        """
        setRigidBody(prim, "convexDecomposition", False)
        #prim.GetAttribute("physics:rigidBodyEnabled").Set(False)
        setup_physics_material(prim)
        add_mass_to_prim(prim)

        # stage = omni.usd.get_context().get_stage()
        # physicsUtils.add_ground_plane(stage, "/groundPlane", "Y", 750.0, Gf.Vec3f(0.0, -10.0, 0), Gf.Vec3f(0.5))
    # if 'Faucet' in  prim.GetPath().pathString:
    #     setup_physics_material(prim)
    #     add_mass_to_prim(prim)
        if IS_IN_ISAAC_SIM and "Bottle" in prim.GetPath().pathString :
            add_update_semantics(prim, "Bottle")

    if "StorageFurniture" in prim.GetPath().pathString:
        """
        Set up physical material for handles
        """
        # setup_physics_material(prim)
        # add_physical_material_to("coll")
        fix_handle('StorageFurniture')
        # remove_collider_to("visuals")
        # if IS_IN_ISAAC_SIM:
        #     add_update_semantics(prim, "StorageFurniture")
            # add_semantics("handle")
    
    if "Basin" in prim.GetPath().pathString:
        approximationShape = "convexDecomposition"

        # convex decomp basin
        stage = omni.usd.get_context().get_stage()
        collision_api = UsdPhysics.MeshCollisionAPI.Get(stage, prim.GetPath())
        if not collision_api:
            collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        
        collision_api.CreateApproximationAttr().Set(approximationShape)
       
        # set up physical metarial
        # add_physical_material_to("Basin")

        if IS_IN_ISAAC_SIM:
            add_update_semantics(prim, "Basin")
    
    elif "Faucet" in prim.GetPath().pathString:
        from .fluid.cup_data import FAUCET_INFO
        faucet_id = prim.GetPath().pathString.split("_")[-1]
        inflow_position = FAUCET_INFO[faucet_id]["inflow_pos"]

        omni.kit.commands.execute(
            "CreatePrim",
            prim_path="/World/game/inflow",
            prim_type="Xform",
            select_new_prim=False,
        )

        inflow_xform = pxr.Gf.Matrix4d().SetTranslate(inflow_position)
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path="/World/game/inflow",
            new_transform_matrix=inflow_xform,
        )
        stage = omni.usd.get_context().get_stage()
        import re
        link_pattern = re.compile('.*'+'link_[0-9]+$')
        links = list(filter( lambda x : link_pattern.findall(x.GetPath().pathString) , list(stage.TraverseAll()) ))
        for link in links:
            add_mass_to_prim(link, 0.1)
        if IS_IN_ISAAC_SIM:
            add_update_semantics(prim, "Faucet")


    
def add_mass_to_prim(prim, mass:float=0.02, density:float=1):
    stage = omni.usd.get_context().get_stage()
    
    mass_api = UsdPhysics.MassAPI.Get(stage, prim.GetPath())
    if not mass_api:
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(mass)
        # mass_api.CreateDensityAttr().Set(density)
    else:
        mass_api.GetMassAttr().Set(mass)
        # mass_api.GetDensityAttr().Set(density)

def setup_physics_material(prim):
    """
    Set up physic material for prim at Path
    """
    # def _setup_physics_material(self, path: Sdf.Path):
    stage = omni.usd.get_context().get_stage()
    _material_static_friction = 100.0
    _material_dynamic_friction = 100.0
    _material_restitution = 0.0
    _physicsMaterialPath = None

    if _physicsMaterialPath is None:
        # _physicsMaterialPath = stage.GetDefaultPrim().GetPath().AppendChild("physicsMaterial")
        _physicsMaterialPath = prim.GetPath().AppendChild("physicsMaterial")
        # print("physics_material_path: ", _physicsMaterialPath)
        
        UsdShade.Material.Define(stage, _physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(_physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(_material_static_friction)
        material.CreateDynamicFrictionAttr().Set(_material_dynamic_friction)
        material.CreateRestitutionAttr().Set(_material_restitution)

    collisionAPI = UsdPhysics.CollisionAPI.Get(stage, prim.GetPath())
    # prim = stage.GetPrimAtPath(path)
    if not collisionAPI:
        collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
    # apply material
    physicsUtils.add_physics_material_to_prim(stage, prim, _physicsMaterialPath)
    print("physics material: path: ", _physicsMaterialPath)


def add_ground_plane(prim_path = "/World/game"):
    stage = omni.usd.get_context().get_stage()
    if True: #IS_IN_ISAAC_SIM:
            purposes = [pxr.UsdGeom.Tokens.default_]
            bboxcache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), purposes)
            prim = stage.GetPrimAtPath(prim_path)
            bboxes = bboxcache.ComputeWorldBound(prim)
            # print("bboxes", bboxes)
            y = bboxes.ComputeAlignedRange().GetMin()[1]
            physicsUtils.add_ground_plane(stage, "/groundPlane", "Y", 750.0, pxr.Gf.Vec3f(0.0, y, 0), pxr.Gf.Vec3f(0.2))
            prim_list = list(stage.TraverseAll())
            prim_list = [ item for item in prim_list if 'groundPlane' in item.GetPath().pathString and item.GetTypeName() == 'Mesh' ]
            for prim in prim_list:
                prim.GetAttribute('visibility').Set('invisible')
    else:
        # prim_path = stage.GetDefaultPrim().GetPath().pathString
        usd_context = omni.usd.get_context()
        bboxes = usd_context.compute_path_world_bounding_box(prim_path)

        physicsUtils.add_ground_plane(stage, "/groundPlane", "Y", 750.0, pxr.Gf.Vec3f(0.0, bboxes[0][1], 0), pxr.Gf.Vec3f(0.2))

def add_physical_material_to(keyword:str):
    """
    Set up physical material
    """
    stage = omni.usd.get_context().get_stage()
    prim_list = list(stage.TraverseAll())
    prim_list = [ item for item in prim_list if keyword in item.GetPath().pathString and 'visuals' not in item.GetPath().pathString ]
    for prim in prim_list:
        setup_physics_material(prim)
        print("add physics material to handle")
        setStaticCollider(prim, approximationShape = "convexDecomposition")

def fix_handle(keyword):
    """
    Set up physical material
    and change collision type ot covex decomposition
    
    """
    stage = omni.usd.get_context().get_stage()
    prim_list = list(stage.TraverseAll())
    #=========================
    prim_list = [ item for item in prim_list if keyword in item.GetPath().pathString and \
         'handle' in item.GetPath().pathString and item.GetTypeName() == 'Mesh'  ]
    # print("prim_list: ", prim_list)
    for prim in prim_list:
        setStaticCollider(prim, approximationShape = "convexDecomposition")
        setup_physics_material(prim)
    # table = {}
    # for prim_path in prim_list:
    #     prefix, suffix = "/".join(prim_path.split('/')[:-1]), prim_path.split('/')[-1]
    #     if prefix not in table:
    #         table[prefix] = []
        
    #     table[prefix].append(suffix)
    
    # for prefix, value in table.items():
    #     handle = value[-1]
    #     import os
    #     from omni.isaac.core.utils.prims import get_prim_at_path
    #     handle_path  =str(os.path.join(prefix, handle))
    #     handle_prim = get_prim_at_path(handle_path)
        
    #     setup_physics_material(handle_prim)
    #     setStaticCollider(handle_prim, approximationShape = "convexDecomposition")
    #=================================

    # prim_list = list(stage.TraverseAll())
    # prim_list = [ item for item in prim_list if keyword in item.GetPath().pathString and \
    #      'visuals' in item.GetPath().pathString and item.GetTypeName() == 'Mesh'  ]
    # print(prim_list)
    # for prim in prim_list:
    #     setup_physics_material(prim)
    #     setStaticCollider(prim, approximationShape = "convexDecomposition")

def remove_collider_to(keyword:str):
    """
    Set up physical material
    """
    stage = omni.usd.get_context().get_stage()
    prim_list = list(stage.TraverseAll())
    prim_list = [ item for item in prim_list if keyword in item.GetPath().pathString ]
    for prim in prim_list:
        removeCollider(prim.GetPath().pathString)



    