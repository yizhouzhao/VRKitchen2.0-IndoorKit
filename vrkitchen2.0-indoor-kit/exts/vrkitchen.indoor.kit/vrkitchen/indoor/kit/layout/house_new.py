from cgitb import enable
import os
import json
from typing import Container
import numpy as np
import asyncio

import omni
import pxr
import carb

from omni.physx.scripts import physicsUtils
from omni.physx.scripts.utils import setCollider, setRigidBody, setStaticCollider
from omni.usd import get_world_transform_matrix, get_local_transform_matrix

from ..param import DATA_PATH_NEW, ASSET_PATH, HOUSE_INFO_PATH, IS_IN_ISAAC_SIM, RIGIDBODY_OBJ_TYPES, GAME_OBJ_NAMES, \
    IS_IN_CREAT, CONTAINER_NAMES, OTHER_OBJ_NAMES

from .utils import rename_prim, rotationXYZ_to_quaternion, freeze_prim
from .modify import modify_game_obj_prim

if IS_IN_CREAT:
    import omni.kit.viewport_widgets_manager as wm
    from ..ui.hud import LabelWidget
else:
    from omni.isaac.core.utils.prims import get_all_matching_child_prims, get_prim_type_name
    from omni.isaac.core.utils.semantics import add_update_semantics

from .utils import NpEncoder

class House():
    def __init__(self, 
            task_type:str, 
            task_id:int, 
            robot_id:int = 0, 
            mission_id:int = 0,
            house_id:int = 0,
            anchor_id:int=0, 
            annotator="",
            ):
        self.task_type = task_type
        self.task_id = str(task_id)
        self.data_path = DATA_PATH_NEW
        self.robot_id = str(robot_id)
        self.anchor_id = str(anchor_id)
        self.mission_id = str(mission_id)
        self.house_id = str(house_id)
        self.annotator = str(annotator)

        # task saving dicts/lists
        self.object_info = []
        self.robot_info = {}
        self.make_task_saving_folder()

        # house saving dict
        self.house_appearance = {}
        self.house_task_anchor = {}

        self.object_prims = []

        
    def make_task_saving_folder(self):
        """
        check task saving folder
        """
        task_type_folder = os.path.join(self.data_path, self.annotator, "task",  self.task_type)
        if not os.path.exists(task_type_folder):
            os.makedirs(task_type_folder)
        task_folder = os.path.join(self.data_path, self.annotator,  "task", self.task_type, str(self.task_id))
        if not os.path.exists(task_folder):
            os.makedirs(task_folder)
    
    def record_obj_info(self):
        """
        record game object information and save
        """
        # scene
        self.stage = omni.usd.get_context().get_stage() 
         # Get sapien asset name
        #prims = [self.stage.GetDefaultPrim()]
        game_prim = self.stage.GetPrimAtPath("/World/game")
        if not game_prim:
            raise Exception("Please move object and robot under /World/game")
        #print("prims", prims)
        for prim in game_prim.GetChildren():
            # print("prim ", prim.GetPath())
            # if prim is game obj, record information
            is_game_obj = False
            for game_name in GAME_OBJ_NAMES + CONTAINER_NAMES + OTHER_OBJ_NAMES:
                if game_name in prim.GetPath().pathString.lower():
                    is_game_obj = True
                    break
            
            if is_game_obj:
                reference, _ = omni.usd.get_composed_references_from_prim(prim)[0]
                print("mobility reference: ", reference.assetPath)
                relative_path = omni.client.make_relative_url(ASSET_PATH, reference.assetPath)
                relative_path = relative_path.replace("\\\\","/").replace("\\","/")

                # get obj type from paths
                path_splits = relative_path.split("/")
                # print("path_splits", path_splits)
                # asset_path = "/".join(path_splits[2:])
                obj_info = {
                        "asset_path": relative_path,
                        "obj_type": path_splits[-3],    
                        "obj_id": path_splits[-2],
                        "materials":[],
                    }
                
                mat = get_world_transform_matrix(prim) 
                
                if prim.HasAttribute("xformOp:orient"):
                    quad = prim.GetAttribute("xformOp:orient").Get()
                else:
                    rotateXYZ = prim.GetAttribute("xformOp:rotateXYZ").Get()
                    quad = rotationXYZ_to_quaternion(rotateXYZ)

                # quad = prim.GetAttribute("xformOp:orient").Get() # eval(str(mat.ExtractRotationQuat())) #eval(str(mat.ExtractRotation().GetQuat()))
                quad = eval(str(quad))
                translate = mat.ExtractTranslation()
                scale = prim.GetAttribute("xformOp:scale").Get()
                #print("translate", translate)
                #print("quad", prim.GetPath(), quad)
                obj_info["translate"] = [translate[0], translate[1], translate[2]]
                obj_info["orient"] = [quad[0], quad[1], quad[2], quad[3]]
                obj_info["scale"] = [scale[0],scale[1],scale[2]]

                print("obj_info", obj_info)
                # task_identity = obj_info["obj_type"] + obj_info["obj_id"]
                self.object_info.append(obj_info)
 
            
        # IMPORTANT: if the object is unbalanced scale, freeze object by
        # To enter this condition is very strict: open/close door, wrong proportion of scale 
        # 1. Create a new xform
        # 2. Move the object under the unit xform
        # 3. Save the obj as another usd variance
        game_obj_info = self.object_info[0]
        game_obj_scale = game_obj_info["scale"]
        if self.task_type in ["open_door", "close_door"]:
            need_freeze = abs(game_obj_scale[0] / game_obj_scale[1]) > 1.2 or \
                            abs(game_obj_scale[0] / game_obj_scale[1]) < 0.8 or \
                                abs(game_obj_scale[1] / game_obj_scale[2]) > 1.2 or \
                                    abs(game_obj_scale[1] / game_obj_scale[2]) < 0.8 or \
                                        abs(game_obj_scale[0] / game_obj_scale[2]) > 1.2 or \
                                            abs(game_obj_scale[0] / game_obj_scale[2]) < 0.8 

            if need_freeze:
                    carb.log_warn("Found non-unit scale object, freezing transfrom...")
                    original_usd_path = os.path.join(ASSET_PATH, game_obj_info["asset_path"])
                    
                    var_usd_path = original_usd_path.replace("mobility", 
                        f"mobility_{self.annotator}_{self.task_type}_{self.task_id}_{self.robot_id}_{self.mission_id}_{self.house_id}_{self.anchor_id}")

                    import shutil
                    shutil.copyfile(original_usd_path, var_usd_path)
                    
                    omni.usd.get_context().close_stage()
                    omni.usd.get_context().open_stage(var_usd_path)
                    stage = omni.usd.get_context().get_stage()
                    default_prim = stage.GetDefaultPrim()
                    # default_prim.GetAttribute("xformOp:scale").Set(pxr.Gf.Vec3f(1, 2, 1))
                    new_prim = freeze_prim(default_prim, game_obj_scale)
                    pxr.UsdPhysics.ArticulationRootAPI.Apply(new_prim)
                    stage.SetDefaultPrim(new_prim)
                    omni.usd.get_context().save_stage()
                    # time.sleep(1.0)
                    # omni.usd.get_context().close_stage()
                    
                    relative_path = omni.client.make_relative_url(ASSET_PATH, var_usd_path)
                    relative_path.replace("\\", "/")
                    game_obj_info["asset_path"] = relative_path 
                    new_size = (game_obj_scale[0] * game_obj_scale[1] *  game_obj_scale[2]) ** (1/3)
                    game_obj_info["scale"] = [1 / new_size , 1 / new_size , 1 / new_size]
        
        # save obj info
        if len(self.object_info) > 0:
            if self.house_id != "-1" and self.anchor_id != "-1":
                obj_identifier = f"{self.house_id} {self.anchor_id}"
                task_obj_path = os.path.join(self.data_path, self.annotator,"task", self.task_type, self.task_id, "objects_with_rooms.json")
                objects_with_rooms = {} if not os.path.exists(task_obj_path) else json.load(open(task_obj_path))
                objects_with_rooms[obj_identifier] = self.object_info
                with open(task_obj_path, "w") as f:
                    json.dump(objects_with_rooms, f, indent=4, cls=NpEncoder)
            else:
                task_obj_path = os.path.join(self.data_path, self.annotator,"task", self.task_type, self.task_id, "objects.json")
                with open(task_obj_path, "w") as f:
                    json.dump(self.object_info, f, indent=4, cls=NpEncoder)

            carb.log_info(f"current objects info saving at: {task_obj_path}")
            
    def load_obj_info(self, relative = False):
        """
        Load objects for the task
        if relative: put obj at the original position
        """
        # scene
        self.stage = omni.usd.get_context().get_stage() 
        # set up game root
        default_prim_path_str = "/World"
        self.xform_game_path = default_prim_path_str + "/game" # omni.usd.get_stage_next_free_path(self.stage, "/World/game", True)
   
    
        # check if in house
        self.object_info = None
        if self.house_id != "-1" and self.anchor_id != "-1":
            obj_identifier = f"{self.house_id} {self.anchor_id}"
            task_obj_path = os.path.join(self.data_path, self.annotator,"task", self.task_type, self.task_id, "objects_with_rooms.json")
            objects_with_rooms = {} if not os.path.exists(task_obj_path) else json.load(open(task_obj_path))
            if obj_identifier in objects_with_rooms:
                self.object_info = objects_with_rooms[obj_identifier]
        
        if self.object_info is None:
            task_obj_path = os.path.join(self.data_path, self.annotator, "task", self.task_type, self.task_id, "objects.json")
            if not os.path.exists(task_obj_path):
                raise Exception( "The json file at path {} provided wasn't found".format(task_obj_path) )
            
            # load object info
            self.object_info = json.load(open(task_obj_path))

        
        for obj_idx, obj in enumerate(self.object_info):
    
            # load object usd
            obj_usd_path = os.path.join(ASSET_PATH, obj["asset_path"])
            translate = obj["translate"]
            orient = obj["orient"]
            rotation = pxr.Gf.Quatd(orient[0], orient[1], orient[2], orient[3])
            scale = obj["scale"]

            # move game xform to the first object
            # set up parent 
            if obj_idx == 0:
                xform_game = self.stage.GetPrimAtPath(self.xform_game_path)
                if not xform_game:
                    xform_game = pxr.UsdGeom.Xform.Define(self.stage, self.xform_game_path)
                self.game_translate = translate if not relative else [0,0,0]
                game_xform = pxr.Gf.Matrix4d().SetScale([1,1,1]) *  \
                    pxr.Gf.Matrix4d().SetRotate(pxr.Gf.Quatf(1.0,0.0,0.0,0.0)) * pxr.Gf.Matrix4d().SetTranslate(self.game_translate)

                omni.kit.commands.execute(
                    "TransformPrimCommand",
                    path=self.xform_game_path,
                    new_transform_matrix=game_xform,
                )

                # xform_game.AddTranslateOp().Set(pxr.Gf.Vec3f(*translate))
                # xform_game.AddOrientOp().Set()
                # xform_game.AddScaleOp().Set(pxr.Gf.Vec3f(1.0, 1.0, 1.0))

            # move obj to the correct place
            mobility_prim_path = xform_game.GetPath().pathString + "/mobility"
            prim = self.stage.GetPrimAtPath(mobility_prim_path)
            if not prim.IsValid():
                prim = self.stage.DefinePrim(mobility_prim_path)

            success_bool = prim.GetReferences().AddReference(obj_usd_path)
            # print("get prim children", prim.GetChildren())


            if not success_bool:
                raise Exception("The usd file at path {} provided wasn't found".format(obj_usd_path))
            
            # relative translate
            if obj_idx == 0: # main object
                rel_translate = [0,0,0]
            else:
                rel_translate = [self.game_translate[i] + obj["translate"][i] for i in range(3)]
            xform = pxr.Gf.Matrix4d().SetScale(scale) * pxr.Gf.Matrix4d().SetRotate(rotation) * pxr.Gf.Matrix4d().SetTranslate(rel_translate)
           
            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=prim.GetPath(),
                new_transform_matrix=xform,
            )
            if obj["obj_type"].lower() in GAME_OBJ_NAMES or obj_idx == 0: # main object
                obj_prefix = "mobility_"
            elif obj["obj_type"].lower() in CONTAINER_NAMES:
                obj_prefix = "container_"
            else:
                obj_prefix = "other_"
                
            # if IS_IN_ISAAC_SIM:
            #     add_update_semantics(prim, obj["obj_type"])
            # TODO: set up name rules
            old_prim_name = prim.GetPath().pathString
            new_prim_path = prim.GetPath().GetParentPath().AppendChild(obj_prefix + obj["obj_type"] + "_" +  str(obj["obj_id"]))
            new_prim_name = omni.usd.get_stage_next_free_path(self.stage, new_prim_path.pathString, False)
            # carb.log_info("rename:" + old_prim_name + ";" + new_prim_name ";" + prim.GetPath().pathString)
            rename_prim(old_prim_name, new_prim_name)
            
            target_obj_prim = self.stage.GetPrimAtPath(new_prim_name)
            modify_game_obj_prim(target_obj_prim)  
            

            print("modify prim name: ", new_prim_name)
            self.object_prims.append(new_prim_name)
            
        

    def record_robot_info(self, robot_prim_path = "/World/game/franka"):
        """
        Record robots infomation, and save it RELATIVE position from the main game obj
        :params:
            robot_prim_path: default robot path
        """
        self.stage = omni.usd.get_context().get_stage() 
         # Get sapien asset name
        #prims = [self.stage.GetDefaultPrim()]
        game_prim = self.stage.GetPrimAtPath("/World/game")
        if not game_prim:
            raise Exception("Please move object and robot under /World/game")
        
        #for game_prim in prims:
        for prim in game_prim.GetChildren():
            # print("prim ", prim.GetPath())
            # if prim is game obj, record information
            is_game_obj = False
            for game_name in GAME_OBJ_NAMES:
                if game_name in prim.GetPath().pathString:
                    is_game_obj = True
                    break
            
            if is_game_obj:
                mat = omni.usd.utils.get_world_transform_matrix(prim) 
                game_translate = mat.ExtractTranslation()
                break
        
        if not game_translate:
            raise Exception("Before recording robot, there must be a game object")

        # then, find robot and calcuate relative postion
        """
        Get robot information at robot_prim_path
        """
   
        robot_prim = self.stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim or not pxr.UsdGeom.Xform.Get(self.stage, robot_prim_path):
            raise Exception(f"Must have a robot with XForm at path {robot_prim_path}")
        
        # get robot world transform
        if IS_IN_ISAAC_SIM:
            from omni.isaac.core.prims import XFormPrim
            pos, rot = XFormPrim(robot_prim_path).get_local_pose()
            translate = np.array(pos)
            quad = np.array(rot) 
        else:
            mat = get_local_transform_matrix(robot_prim)
            translate = mat.ExtractTranslation()
            quad = eval(str(mat.ExtractRotation().GetQuat()))

        rob_info = {
            "type":"franka",
            "translate": [round(translate[0], 3), round(translate[1],3), round(translate[2], 3)],
            "orient": [round(quad[0], 3), round(quad[1], 3), round(quad[2], 3), round(quad[3], 3)],
        }
        
        
        
        if self.house_id != "-1" and self.anchor_id != "-1":
            task_robot_path = os.path.join(self.data_path, self.annotator, "task", self.task_type, self.task_id, "robots_with_rooms.json")
            robot_identifier = f"{self.robot_id} {self.house_id} {self.anchor_id} {self.mission_id}"
            objects_with_rooms = {} if not os.path.exists(task_robot_path) else json.load(open(task_robot_path))
            objects_with_rooms[robot_identifier] = rob_info
            with open(task_robot_path, "w") as f:
                json.dump(objects_with_rooms, f, indent=4, cls=NpEncoder)
        else:
            task_robot_path = os.path.join(self.data_path, self.annotator, "task", self.task_type, self.task_id, "robots.json")
            if os.path.exists(task_robot_path):
                self.robot_info = json.load(open(task_robot_path))
            robot_identifier = str(self.robot_id)
            self.robot_info[robot_identifier] = rob_info
            with open(task_robot_path, "w") as f:
                json.dump(self.robot_info, f, indent=4, cls=NpEncoder)
        
        carb.log_info(f"Saving robot json file at {task_robot_path}")

    def load_robot_info(self):
        """
        Load robot for currect task
        """
        # if append house and anchor info
        rot_info = None
        if self.house_id != "-1" and self.anchor_id != "-1":
            task_robot_path = os.path.join(self.data_path, self.annotator, "task", self.task_type, self.task_id, "robots_with_rooms.json")
            robot_identifier = f"{self.robot_id} {self.house_id} {self.anchor_id}"
            robot_identifier = f"{self.robot_id} {self.house_id} {self.anchor_id} {self.mission_id}"
            objects_with_rooms = {} if not os.path.exists(task_robot_path) else json.load(open(task_robot_path))
            if robot_identifier in objects_with_rooms:
                rot_info = objects_with_rooms[robot_identifier]
                
        
        if rot_info is None:
            task_robot_path = os.path.join(self.data_path, self.annotator, "task", self.task_type, self.task_id, "robots.json")
            if not os.path.exists(task_robot_path):
                raise Exception( "The json file at path {} provided wasn't found".format(task_robot_path) )

            # load json information
            self.robot_info = json.load(open(task_robot_path))

            # assert self.robot_id in self.robot_info, \
            #     f"Please record robot id variation first {self.task_type}, task_id {self.task_id}, robot_id {self.robot_id}"
            if self.robot_id in self.robot_info:
                rot_info = self.robot_info[self.robot_id]
            else:
                return None, None
        return rot_info["translate"], rot_info["orient"]

    def record_house_info(self):
        """
        Record house information
        ::params:
            anchor_id: postion of the game root
        """
        # scene
        self.stage = omni.usd.get_context().get_stage() 
        relative_path = None # house/layer asset relative path
        # Get room name 
        room_path = self.stage.GetRootLayer().realPath
        # print("room_path: ", room_path)
        if room_path:
            relative_path = omni.client.make_relative_url(HOUSE_INFO_PATH, room_path)
            relative_path = relative_path.replace("\\\\", "/").replace("\\", "/")
            # print("room_name: ", relative_path)
            # self.layout["asset"]["room_name"] = relative_path
        else:
            self.layer = self.stage.GetRootLayer()
            # print("layer: ", )
            for ref in self.layer.GetExternalReferences():
                if "layout" in str(ref):
                    #PathUtils.compute_relative_path(self.house_info_path,str(ref))
                    relative_path = omni.client.make_relative_url(HOUSE_INFO_PATH, str(ref))
                    relative_path = relative_path.replace("\\\\", "/").replace("\\", "/")
                    # print("relative_path", relative_path)
                    # self.layout["asset"]["room_name"] = relative_path
                    break
        
        # make house saving folder
        assert relative_path is not None

        house_id = relative_path.split("/")[-2]
        house_folder = os.path.join(self.data_path, self.annotator,"house", house_id)
        if not os.path.exists(house_folder):
            os.makedirs(house_folder)
        
        # # make appearance
        # appearance_json_path = os.path.join(house_folder, "appearance.json")
        # if os.path.exists(appearance_json_path):
        #     self.house_appearance = json.load(open(appearance_json_path))
        
        # self.house_appearance["asset_path"] = relative_path
        # with open(appearance_json_path, "w") as f:
        #     json.dump(self.house_appearance, f, indent=4)
        #     carb.log_info(f"Saving hosue appearce json file at {appearance_json_path}")
        
        # find game, task, anchor information
        default_prim_path_str = "/World" #self.stage.GetDefaultPrim().GetPath().pathString
        game_prim = self.stage.GetPrimAtPath(default_prim_path_str + "/game")

        # if game information exists
        if game_prim:

            # load anchor
            anchor_json_path = os.path.join(house_folder, "anchor.json")
            if os.path.exists(anchor_json_path):
                self.house_task_anchor = json.load(open(anchor_json_path))

            # get game transform
            mat = omni.usd.utils.get_world_transform_matrix(game_prim) 
            quad = eval(str(mat.ExtractRotation().GetQuat()))
            translate = mat.ExtractTranslation()
            translate =  [i for i in translate]

            anchor_info = {
                "task_type": self.task_type, 
                "task_id": self.task_id,
                "robot_id": self.robot_id, 
                "anchor_id": self.anchor_id, 
                "game_location": {
                        "translate": translate,
                        "orient":quad,
                    }
            }

            anchor_info["additional_collisions"] = [] # self.get_furniture_collisions()

            # print("anchor_info", anchor_info)
            anchor_identifier = self.task_type + " " + self.task_id + " " + self.robot_id + " " + self.anchor_id
            self.house_task_anchor[anchor_identifier] = anchor_info
            with open(anchor_json_path, "w") as f:
                json.dump(self.house_task_anchor, f, indent=4, cls=NpEncoder)
                carb.log_info(f"Saving anchor json file at {anchor_json_path}")

    def load_house_info(self, enable_collision=True):
        """
        load house infomation from house_id, and anchor_id
        """
        print("loading house")
        # scene
        self.stage = omni.usd.get_context().get_stage() 
        # self.layer = self.stage.GetRootLayer()
        house_path = os.path.join(HOUSE_INFO_PATH, self.house_id, "layout.usd")
        
        # omni.kit.commands.execute(
        #     "CreateSublayer",
        #     layer_identifier=self.layer.identifier,
        #     sublayer_position=0,
        #     new_layer_path=house_path,
        #     transfer_root_content=False,
        #     create_or_insert=False,
        #     layer_name="house",
        # )

        # Check anchor exists, if not, then only the scene
        house_folder = os.path.join(self.data_path, self.annotator, "house", self.house_id)
        anchor_json_path = os.path.join(house_folder, "anchor.json")
        if not os.path.exists(anchor_json_path):
            carb.log_warn("No anchor file found, record anchor information first")
            return False
        
        # print("anchor_json_path: ", anchor_json_path)
        try:
            self.house_task_anchor = json.load(open(anchor_json_path))
        except:
            carb.log_error("anchro_json path not correct: " + str(anchor_json_path))
            return False
        anchor_identifier_prefix = self.task_type + " " + self.task_id # + " " + self.robot_id + " " + self.anchor_id
        has_anchor = False
        for key in self.house_task_anchor:
            if key.startswith(anchor_identifier_prefix):
                has_anchor = True
                anchor_identifier = key
                break

        if not has_anchor:
            carb.log_warn(f"No anchor id: {self.anchor_id}, please record anchor at {anchor_json_path}")
            return False


        # move obj to the correct place
        house_prim_path = "/World/layout"
        house_prim = self.stage.GetPrimAtPath(house_prim_path)
        if not house_prim.IsValid():
            house_prim = self.stage.DefinePrim(house_prim_path)

        success_bool = house_prim.GetReferences().AddReference(house_path)

        if not success_bool:
            raise Exception(f"The house is not load at {house_path}")

        # static collider
        # print("set collisiton")
        # furniture_prim = self.stage.GetPrimAtPath(house_prim_path + "/furniture/furniture_87879")
        # setStaticCollider(furniture_prim, approximationShape="convexDecomposition")
        
        furniture_prim = self.stage.GetPrimAtPath(house_prim_path + "/furniture")
        if furniture_prim:
            setStaticCollider(furniture_prim, approximationShape="convexHull")
        else:
            return False
        # if not self.task_type in ["tap_water", "transfer_water", "pour_water"] and enable_collision:
        #     room_struct_prim = self.stage.GetPrimAtPath(house_prim_path + "/roomStruct")
        #     setStaticCollider(room_struct_prim, approximationShape="none")

        # check task/task_type/robot
        anchor_info = self.house_task_anchor[anchor_identifier]
        # if anchor_info["task_type"] != self.task_type or \
        #     anchor_info["task_id"] != self.task_id or \
        #          anchor_info["robot_id"] != self.robot_id:
        #          raise Exception("Anchor information at {} does not match UI inputs".format(anchor_json_path))

        # find game, task, anchor information
        default_prim_path_str = "/World"
        game_prim = self.stage.GetPrimAtPath(default_prim_path_str + "/game")
        # if game information exists
        if not game_prim:
            carb.log_error(f"must have game obj at path {default_prim_path_str} + /game ")
            return False
        
        print("anchor_info", anchor_info)
        orient = anchor_info["game_location"]["orient"]
        translate = anchor_info["game_location"]["translate"]
        rotation = pxr.Gf.Quatd(orient[0], orient[1], orient[2], orient[3])
        game_xform = pxr.Gf.Matrix4d().SetScale([1,1,1]) *  \
                pxr.Gf.Matrix4d().SetRotate(rotation) * pxr.Gf.Matrix4d().SetTranslate(translate)

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=default_prim_path_str + "/game",
            new_transform_matrix=game_xform,
        )

        # set up additional collision
        # for furni_path in anchor_info["additional_collisions"]:
        #     prim = self.stage.GetPrimAtPath(furni_path)
            
        #     # set rigidbody and disable it, only leave with collision
        #     setRigidBody(prim, "convexDecomposition", False)
        #     prim.GetAttribute("physics:rigidBodyEnabled").Set(False)
        #     print("try to set collider: ", furni_path)

       
        ## add ground
        ground_prim = self.stage.GetPrimAtPath(default_prim_path_str + '/groundPlane')
        if not ground_prim:
            physicsUtils.add_ground_plane(self.stage,  '/groundPlane', "Y", 1000.0, 
                pxr.Gf.Vec3f(0.0, 0.0, 0), pxr.Gf.Vec3f(0.2))
            ground_prim = self.stage.GetPrimAtPath(default_prim_path_str + '/groundPlane')
        # prim_list = list(self.stage.TraverseAll())
        # prim_list = [ item for item in prim_list if 'groundPlane' in item.GetPath().pathString and item.GetTypeName() == 'Mesh' ]
        # for prim in prim_list:
        ground_prim.GetAttribute('visibility').Set('invisible')
        # if ground_prim: 
        #     omni.kit.commands.execute("DeletePrims", paths=[ground_prim.GetPath()])
        # ground_prim = self.stage.GetPrimAtPath("/World/groundPlane")
        # if ground_prim:
        #     omni.kit.commands.execute("DeletePrims", paths=[ground_prim.GetPath()])

        # gui = self.stage.GetPrimAtPath("/World/GUI")
        # if gui:
        #     omni.kit.commands.execute("DeletePrims", paths=[gui.GetPath()])

        return True

    #----------------------------------------utils---------------------------------------------
    def get_furniture_collisions(self):
        """
        Get furniture information especially for collision from current scene
        """
        # scene # furniture parent
        self.stage = omni.usd.get_context().get_stage() 
        additional_collisions = []
        furni_parent = self.stage.GetPrimAtPath("/World/furniture")

        # if has furniture
        if furni_parent:
            for prim in furni_parent.GetChildren():
                
                if prim.HasAPI(pxr.UsdPhysics.RigidBodyAPI) or prim.HasAPI(pxr.UsdPhysics.CollisionAPI):
                    #  prim.GetAttribute("physics:rigidBodyEnabled").Set(False)
                    print("collision prim name", prim.GetPath(), prim.GetAttribute("physics:rigidBodyEnabled").Get())
                    # robot_prim.GetAttribute("xformOp:orient").Get()
                    additional_collisions.append(prim.GetPath().pathString)
        
        return additional_collisions

    
    def regularizing_game_robot_obj_location(self):
        """
        Regulariting game/robot/obj locations: put /World/game translate as the obj location
        """
        carb.log_info("Regularizing game/robot/obj locations")
        # move game to main object
        stage = omni.usd.get_context().get_stage()
        game_prim = stage.GetPrimAtPath("/World/game")
        if game_prim:
            for obj_prim in game_prim.GetChildren():
                if "mobility" in obj_prim.GetPath().pathString:
                    pos = pxr.UsdGeom.Xformable(obj_prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
                    # rot = pos = pxr.UsdGeom.Xformable(obj_prim).ComputeLocalToWorldTransform(0).ExtractRotation().GetQuat()
                    # print("pos", pos, "rot", rot)

                    pos = [i for i in pos]
                    game_xform = pxr.Gf.Matrix4d().SetScale([1,1,1]) *  \
                        pxr.Gf.Matrix4d().SetRotate(pxr.Gf.Quatf(1.0,0.0,0.0,0.0)) * pxr.Gf.Matrix4d().SetTranslate(pos)
                    
                    omni.kit.commands.execute(
                                "TransformPrimCommand",
                                path=game_prim.GetPath().pathString,
                                new_transform_matrix=game_xform,
                            )

                    obj_prim.GetAttribute("xformOp:translate").Set(pxr.Gf.Vec3f(0.0, 0.0, 0.0))

                    # also transfer the location of the robot
                    robot_prim = stage.GetPrimAtPath("/World/game/franka")
                    if robot_prim:
                        robot_translate = robot_prim.GetAttribute("xformOp:translate").Get()
                        new_robot_translate = [robot_translate[i] - pos[i] for i in range(3)]
                        robot_prim.GetAttribute("xformOp:translate").Set(pxr.Gf.Vec3f(*new_robot_translate))
                
                    break
    
    def house_anchor_id_suggestion(self):
        """
        Get house ids that are possible for current task_type/task_id/anchor
        """
        suggested_house_ids = []
        suggested_anchor_ids = []

        anchor_identifier_prefix = self.task_type + " " + self.task_id + " " + self.robot_id
        house_root = os.path.join(self.data_path, self.annotator, "house")
        print("os.listdir(house_root)", house_root)
        for house_name in os.listdir(house_root):
            anchor_json_path = os.path.join(house_root, house_name, "anchor.json")
            if not os.path.exists(anchor_json_path):
                carb.log_warn("please add anchor.json to current task")
                return ""
            with open(anchor_json_path, "r") as f:
                anchor_info = json.load(f)
                for identifier in anchor_info.keys():
                    if identifier.startswith(anchor_identifier_prefix):
                        suggested_house_ids.append(house_name)
                        anchod_id = identifier.split()[-1]
                        suggested_anchor_ids.append(anchod_id)

        return [str((i,j)) for i,j in zip(suggested_house_ids, suggested_anchor_ids)]

    def build_HUD(self):

        self.stage = omni.usd.get_context().get_stage()
        gui_path = self.stage.GetDefaultPrim().GetPath().pathString + "/GUI"
        gui = self.stage.GetPrimAtPath(gui_path)
        if not gui:
            gui = pxr.UsdGeom.Xform.Define(self.stage, gui_path)
            gui_location = pxr.Gf.Vec3f(0, 100, 100)
            gui.AddTranslateOp().Set(gui_location)
            self.wiget_id = wm.add_widget(gui_path, LabelWidget(f"House id: {self.house_id}"), wm.WidgetAlignment.TOP)


    