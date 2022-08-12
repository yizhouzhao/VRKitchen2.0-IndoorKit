import os
import json

import omni
import pxr
import carb

# phyxc
from omni.physx.scripts.utils import setCollider, setRigidBody, setStaticCollider, set_physics_scene_asyncsimrender

from ..param import SAPIEN_ASSET_PATH, HOUSE_INFO_PATH, DATA_PATH_ROOT, RIGIDBODY_OBJ_TYPES, GAME_OBJ_NAMES
from .utils import rename_prim, rotationXYZ_to_quaternion

# from omni.isaac.core.utils.stage import (
#     get_current_stage,
# )
from pxr import UsdGeom, UsdLux, Gf, Vt, UsdPhysics, PhysxSchema, Usd, UsdShade, Sdf

class House():
    def __init__(self, 
        data_path:str = DATA_PATH_ROOT, 
        sapien_asset_path:str = SAPIEN_ASSET_PATH,  
        house_info_path:str = HOUSE_INFO_PATH):

        self.data_path = data_path
        self.sapien_asset_path = sapien_asset_path
        self.house_info_path = house_info_path

        self.layout = {
            "id":0,
            "params":{
                # "SCENE_ASSET_PATH":self.data_path,
                "SAPIEN_ASSET_PATH":self.sapien_asset_path,
                "HOUSE_INFO_PATH":self.house_info_path,
            },
            "asset":{
                "room_name":"",
                "sapien":[],
            },
            "layout_offsets":[]
        }

    def set_id(self, example_id):
        """
        Set up example id
        """
        self.example_id = example_id
        self.layout["id"] = example_id
    
    def set_task(self, task_type, task_id = None):
        """
        Set up task type
        """
        self.layout["task"] = task_type

    def get_furniture_info(self):
        """
        Get furniture information especially for collision from current scene
        """
        self.stage = omni.usd.get_context().get_stage()

        # furniture parent
        furni_parent = self.stage.GetPrimAtPath("/World/layout/furniture")
        
        additional_collisions = []
        for prim in furni_parent.GetChildren():
            
            if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.HasAPI(pxr.UsdPhysics.CollisionAPI):
                #  prim.GetAttribute("physics:rigidBodyEnabled").Set(False)
                print("collision prim name", prim.GetPath(), prim.GetAttribute("physics:rigidBodyEnabled").Get())
                # robot_prim.GetAttribute("xformOp:orient").Get()
                additional_collisions.append(prim.GetPath().pathString)
        
        self.layout["asset"]["furniture_collisions"] = additional_collisions

    
    def get_robot_info(self, robot_prim_path = "/World/game/franka"):
        """
        Get robot information at robot_prim_path
        """
        self.stage = omni.usd.get_context().get_stage()
        robot_prim = self.stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim or not pxr.UsdGeom.Xform.Get(self.stage, robot_prim_path):
            raise Exception(f"Must have a robot with XForm at path {robot_prim_path}")
        
        quad = robot_prim.GetAttribute("xformOp:orient").Get()
        if not quad:
            rotateXYZ = robot_prim.GetAttribute("xformOp:rotateXYZ").Get()
            quad = rotationXYZ_to_quaternion(rotateXYZ)
        translate = robot_prim.GetAttribute("xformOp:translate").Get()
        scale = robot_prim.GetAttribute("xformOp:scale").Get()

        quad = eval(str(quad))
        # print(quad)

        robot_info = {
            "position": [round(translate[0], 3), round(translate[1],3), round(translate[2], 3)],
            "rotation": [round(quad[0], 3), round(quad[1], 3), round(quad[2], 3), round(quad[3], 3)],
        }
        
        return robot_info

    def add_asset_info(self):
        """
        Add other asset infomation
        """
        # move to randomizer
        pass

    
    def get_asset_info(self, append = False):
        """
        Get mobility, and furniture information from current scene
        :param::
            append: append room information if True else delete json
        """
        self.stage = omni.usd.get_context().get_stage()
        room_layout_json = os.path.join(self.data_path, "house", str(self.example_id) + ".json")

        # if layout json already exists, record game/parent offset as obj randomization
        if os.path.exists(room_layout_json):
            carb.log_warn(f"room info already exists at {room_layout_json}")

            # append other information into json
            if append:
                self.layout = json.load(open(room_layout_json))
                self.add_asset_info()
                return 
            else:
                # delete json and start another
                os.remove(room_layout_json)
                
        # Get room name
        room_path = self.stage.GetRootLayer().realPath
        # print("room_path: ", room_path)
        if room_path:
            relative_path = omni.client.make_relative_url(self.house_info_path, room_path)
            print("room_name: ", relative_path)
            self.layout["asset"]["room_name"] = relative_path
        else:
            self.layer = self.stage.GetRootLayer()
            # print("layer: ", )
            for ref in self.layer.GetExternalReferences():
                if "layout" in str(ref):
                    #PathUtils.compute_relative_path(self.house_info_path,str(ref))
                    relative_path = omni.client.make_relative_url(self.house_info_path, str(ref))
                    relative_path.replace("\\\\", "/")
                    self.layout["asset"]["room_name"] = relative_path
                    break

        # Get sapien asset name
        prims = [self.stage.GetDefaultPrim()]
        game_prim = self.stage.GetPrimAtPath("/World/game")
        if game_prim:
            prims.append(game_prim)
        for game_prim in prims:
            for prim in game_prim.GetChildren():
                # if prim is game obj, record information
                is_game_obj = False
                for game_name in GAME_OBJ_NAMES:
                    if game_name in prim.GetPath().pathString:
                        is_game_obj = True
                        break

                if is_game_obj:
                    reference, _ = omni.usd.get_composed_references_from_prim(prim)[0]
                    print("mobility reference: ", reference.assetPath)

                    # get obj type from paths
                    path_splits = reference.assetPath.split("/")
    
                    if 'sapien_parsed' in path_splits:
                        # sapien objs
                        obj_type = reference.assetPath.split("/")[-3]
                        obj_id = int(reference.assetPath.split("/")[-2])
                        assetPath = None
                    elif 'omniverse:' in path_splits:
                        # obj from omniverse cloud
                        assetPath = reference.assetPath
                        obj_type = path_splits[-2]
                        obj_id  = 0
                    else:
                        # custom objs
                        assetPath =  "/".join(path_splits[-3:])
                        obj_type = path_splits[-3]
                        obj_id = path_splits[-2]
                    
                    obj_info = {
                        "asset_path": assetPath,
                        "obj_type": obj_type,
                        "obj_id": obj_id,
                    }

                    # for attr in prim.GetAttributes():
                    #     print(attr)
                    if prim.HasAttribute("xformOp:orient"):
                        quad = prim.GetAttribute("xformOp:orient").Get()
                    else:
                        rotateXYZ = prim.GetAttribute("xformOp:rotateXYZ").Get()
                        quad = rotationXYZ_to_quaternion(rotateXYZ)

                    translate = prim.GetAttribute("xformOp:translate").Get()
                    scale = prim.GetAttribute("xformOp:scale").Get()

                    quad = eval(str(quad))
                    # print("quad", quad)

                    obj_info["xformOp:translate"] = [translate[0], translate[1], translate[2]]
                    obj_info["xformOp:orient"] = [quad[0], quad[1], quad[2], quad[3]]
                    obj_info["xformOp:scale"] = [scale[0],scale[1],scale[2]]
                    
                    self.layout["asset"]["sapien"].append(obj_info)
            # print("get mobility info ???")
            # get robot information if don't have
            # if "robot" not in self.layout:
            #     if self.stage.GetPrimAtPath("/World/game/franka"):
            #         # if has robot
            #         self.get_robot_info()

            # get additional furniture collision information if don't have
            # if "furniture_collisions" not in self.layout["asset"]:
            #     self.get_furniture_info()
            
        print("get mobility info", self.layout)

    def save_asset_info(self):
        """
        Save asset at data_path
        """
        print("saveing file at " + str(self.layout["id"]) + ".json")
        with open(os.path.join(self.data_path, "house", str(self.layout["id"]) + ".json"), "w") as output_file:
            json.dump(self.layout, output_file, sort_keys=True, indent=4)

    def _setup_physics_material(self, path):
        """
        Set up physic material for prim at Path
        """
        # def _setup_physics_material(self, path: Sdf.Path):
        from pxr import UsdGeom, UsdLux, Gf, Vt, UsdPhysics, PhysxSchema, Usd, UsdShade, Sdf
        from omni.physx.scripts import physicsUtils

        stage = omni.usd.get_context().get_stage()
        _material_static_friction = 1.0
        _material_dynamic_friction = 1.0
        _material_restitution = 0.0
        _physicsMaterialPath = None

        if _physicsMaterialPath is None:
            _physicsMaterialPath = stage.GetDefaultPrim().GetPath().AppendChild("physicsMaterial")
            UsdShade.Material.Define(stage, _physicsMaterialPath)
            material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(_physicsMaterialPath))
            material.CreateStaticFrictionAttr().Set(_material_static_friction)
            material.CreateDynamicFrictionAttr().Set(_material_dynamic_friction)
            material.CreateRestitutionAttr().Set(_material_restitution)

        collisionAPI = UsdPhysics.CollisionAPI.Get(stage, path)
        prim = stage.GetPrimAtPath(path)
        if not collisionAPI:
            collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
        # apply material
        # physicsUtils.add_physics_material_to_prim(stage, prim, _physicsMaterialPath)
    
    def load_asset_info(self, house_id, object_id = None):
        """
        load asset from data path

        """
        
        room_layout_json = os.path.join(self.data_path, "house", str(house_id) + ".json")
        print("hosue id", str(house_id), "data path: wtf", room_layout_json)

        if not os.path.exists(room_layout_json):
            raise Exception( "The json file at path {} provided wasn't found".format(room_layout_json) )
        
        # load json
        self.layout = json.load(open(room_layout_json))

        # get currect stage and layer
        self.stage =  omni.usd.get_context().get_stage()
        self.layer = self.stage.GetRootLayer()

        # load house info
        house_path = os.path.join(self.house_info_path, self.layout["asset"]["room_name"].replace("\\","/"))
        # print('self.layout["asset"]["room_name"]',self.layout["asset"]["room_name"])
        print("house_path: ", house_path)

        omni.kit.commands.execute(
            "CreateSublayer",
            layer_identifier=self.layer.identifier,
            sublayer_position=0,
            new_layer_path=house_path,
            transfer_root_content=False,
            create_or_insert=False,
            layer_name="",
        )

        # set up furniture root
        default_prim_path_str = self.stage.GetDefaultPrim().GetPath().pathString
        ## this is necessary because for standalone this might not be /World
        if not default_prim_path_str:
            default_prim_path_str = "/World"
        self.xform_game_path = default_prim_path_str + "/game" # omni.usd.get_stage_next_free_path(self.stage, "/World/game", True)

        if not self.stage.GetPrimAtPath(self.xform_game_path):
            xform_game = pxr.UsdGeom.Xform.Define(self.stage, self.xform_game_path)

            xform_game.AddTranslateOp().Set(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
            xform_game.AddOrientOp().Set(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            xform_game.AddScaleOp().Set(pxr.Gf.Vec3f(1.0, 1.0, 1.0))
        
        # # Everything has to have collision
        # furni_parent = self.stage.GetPrimAtPath("/World/furniture")
        # for prim in furni_parent.GetChildren():
        #     setCollider(prim, "convexDecomposition")
        # floor_prim = self.stage.GetPrimAtPath("/World/floors")
        # setCollider(floor_prim, "convexDecomposition")

        # add collision infomation
        if "furniture_collisions" in self.layout["asset"]:
            for furni_path in self.layout["asset"]["furniture_collisions"]:
                prim = self.stage.GetPrimAtPath(furni_path)
                setCollider(prim, "convexDecomposition")
                print("try to set collider: ", furni_path)
                setRigidBody(prim, "convexDecomposition", False)
                physicsAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
                physicsAPI.CreateRigidBodyEnabledAttr(False)
                # physicsAPI.CreateDisableGravityAttr(True)

                print("set rigid body: ", furni_path)

        # load furniture info
        for obj in self.layout["asset"]["sapien"]:
            
            # filter object only necessary for currect task
            if object_id != None:
                if obj['obj_id'] != object_id:
                    continue 

            # get asset path 
            if "asset_path" in obj and obj["asset_path"] is not None:
                if "omniverse:" in obj["asset_path"]:
                    # cloud obj
                    obj_usd_path = obj["asset_path"]
                else:
                    # custom object
                    obj_usd_path = os.path.join(self.sapien_asset_path, "../custom", obj["asset_path"])
            else:
                # sapien object
                obj_usd_path = os.path.join(self.sapien_asset_path, obj["obj_type"], str(obj["obj_id"]), "mobility.usd")
            print("obj_usd_path", obj_usd_path)

            # load data
            mobility_prim_path = xform_game.GetPath().pathString + "/mobility"
            prim = self.stage.GetPrimAtPath(mobility_prim_path)
            if not prim.IsValid():
                prim = self.stage.DefinePrim(mobility_prim_path)

            success_bool = prim.GetReferences().AddReference(obj_usd_path)


            if not success_bool:
                raise Exception("The usd file at path {} provided wasn't found".format(obj_usd_path))


            # set xform
            # obj_xform = pxr.UsdGeom.Xformable.Get(self.stage, prim.GetPath())
            # translate_component = obj_xform.GetOrderedXformOps()[0]
            # orient_component = obj_xform.GetOrderedXformOps()[1]
            # scale_component = obj_xform.GetOrderedXformOps()[2]

            translate = obj["xformOp:translate"]
            # translate_component.Set(tuple(translate))

            orient = eval(obj["xformOp:orient"]) if isinstance(obj["xformOp:orient"], str) else obj["xformOp:orient"]
            rotation = pxr.Gf.Quatd(orient[0], orient[1], orient[2], orient[3])
            # orient_component.Set(rotation)

            scale = obj["xformOp:scale"]
            # scale_component.Set(tuple(scale))

            xform = pxr.Gf.Matrix4d().SetScale(scale) * pxr.Gf.Matrix4d().SetRotate(rotation) * pxr.Gf.Matrix4d().SetTranslate(translate)
            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=prim.GetPath(),
                new_transform_matrix=xform,
            )

            ## or 
            # xform_geom.AddTranslateOp().Set(position)
            # xform_geom.AddOrientOp().Set(orientation)
            # xform_geom.AddScaleOp().Set(scale)

            # set collision & rigidbody
            should_add_rigidbody = False
            for collision_type in RIGIDBODY_OBJ_TYPES:
                if collision_type in obj["obj_type"]:
                    should_add_rigidbody = True
                    break
            
            if should_add_rigidbody:
                setRigidBody(prim, "convexDecomposition", False)

            
            # set up physcial materials
            # self._setup_physics_material(prim.GetPath())
            
            # rename path
            # TODO: set up name rules
            old_prim_name = prim.GetPath().pathString
            new_prim_path = prim.GetPath().GetParentPath().AppendChild("mobility_" + obj["obj_type"] + "_" +  str(obj["obj_id"]))
            new_prim_name = omni.usd.get_stage_next_free_path(self.stage, new_prim_path.pathString, False)
            carb.log_info("rename:" + old_prim_name + ";" + new_prim_name)
            rename_prim(old_prim_name, new_prim_name)

        default_prim_path_str = self.stage.GetDefaultPrim().GetPath().pathString
        ## this is necessary because for standalone this might not be /World
        if not default_prim_path_str:
            default_prim_path_str = "/World"
            
        #set up physics scene
        # from omni.physx.scripts import utils
        _gravityMagnitude = 100.0  # IN CM/s2 - use a lower gravity to avoid fluid compression at 60 FPS
        _gravityDirection = Gf.Vec3f(0.0, -1.0, 0.0)
        _solver = "TGS"
        _gpuMaxNumPartitions = 4

        physicsScenePath = os.path.join(default_prim_path_str, "physicsScene")
        scene = UsdPhysics.Scene.Define(self.stage, physicsScenePath)
        scene.CreateGravityDirectionAttr().Set(_gravityDirection)
        scene.CreateGravityMagnitudeAttr().Set(_gravityMagnitude)
        set_physics_scene_asyncsimrender(scene.GetPrim())
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI.CreateSolverTypeAttr(_solver)
        physxAPI.CreateGpuMaxNumPartitionsAttr(_gpuMaxNumPartitions)
    
    def add_distraction_objects(self):
        pass
    
      
