# auto task generating

import os
import json
import numpy as np
import asyncio

import omni
import pxr
import carb

from omni.physx.scripts import physicsUtils

from ..param import IS_IN_ISAAC_SIM, DATA_PATH_NEW, CUSTOM_ASSET_PATH, ROBOT_PATH, SAPIEN_ASSET_PATH, IS_IN_CREAT, \
    GAME_OBJ_NAMES, CONTAINER_NAMES, OTHER_OBJ_NAMES, HOUSE_INFO_PATH
from ..task_check import BaseChecker, JointChecker, GraspChecker, OrientChecker, ContainerChecker
from .meta import AUTOTASK_META

if IS_IN_CREAT:
    import omni.kit.viewport_widgets_manager as wm
    from ..ui.hud import LabelWidget

class AutoTasker():
    TASK_DESCRIPTION = ""
    TASK_ID = ""
    def __init__(self, 
            task_type:str, 
            task_id:int, 
            robot_id:int = 0, 
            mission_id:int = 0,
            house_id:int = 0,
            anchor_id:int = 0,
            meta_id : int = 0, # to retrieve which config from meta data
            annotator : int = 0,
            ) -> None:
        
        self.task_type = task_type
        self.task_id = str(task_id)
        
        self.robot_id = str(robot_id)
        self.mission_id = str(mission_id)

        self.house_id = str(house_id)
        self.anchor_id = str(anchor_id)

        self.meta_id = mission_id # meta_id
        self.data_path = DATA_PATH_NEW
        
        # scene
        self.stage = omni.usd.get_context().get_stage()

        ##
        self.annotator = annotator

        # get objects
        self.probe_obj_folder()
    
    def probe_obj_folder(self):
        """
        check task  folder
        """
        task_type_folder = os.path.join(self.data_path, self.annotator, "task", self.task_type)
        if not os.path.exists(task_type_folder):
            os.makedirs(task_type_folder)
        task_folder = os.path.join(self.data_path, self.annotator, "task", self.task_type, str(self.task_id))
        if not os.path.exists(task_folder):
            os.makedirs(task_folder)

        """
        Get furniture
        """
        if self.task_type in ["open_drawer", "open_cabinet", "close_drawer", "close_cabinet"]:
            self.obj_type = "StorageFurniture"
            self.obj_folder = os.path.join(SAPIEN_ASSET_PATH, self.obj_type)
        elif self.task_type in ["pickup_object", "reorient_object"]:
            self.obj_type = "Bottle"
            self.obj_folder = os.path.join(CUSTOM_ASSET_PATH, self.obj_type)
        elif self.task_type in ["put_object_into_box", "take_object_out_box"]: 
            self.obj_type = "Box"
            self.obj_folder = os.path.join(SAPIEN_ASSET_PATH, self.obj_type)
        elif self.task_type in ["open_door", "close_door"]:
            self.obj_type = "Door"
            self.obj_folder = os.path.join(SAPIEN_ASSET_PATH, self.obj_type)
        elif self.task_type in ["pour_water", "transfer_water"]:
            self.obj_type = "Cup"
            self.obj_folder = os.path.join(CUSTOM_ASSET_PATH, self.obj_type) 
        elif self.task_type in ["tap_water"]:
            self.obj_type = "Faucet"
            self.obj_folder = os.path.join(SAPIEN_ASSET_PATH, self.obj_type) 
        else:
            raise Exception(f"current task type not supported: {self.task_type}")
        
        objs = [ item for item in os.listdir(self.obj_folder) if item.isnumeric() ]

        self.obj_list = sorted( objs, key=lambda x: int(x))
        self.obj_id = self.obj_list[int(self.task_id)]
        self.target_obj_path = "/mobility_" + self.obj_type + "_" + str(self.obj_id) 
        
         
    
    def reconfig(self, obj_index):
        """
        Reconfig obj from object index
        """
        self.obj_index = obj_index
        self.obj_id = self.obj_list[int(obj_index)]
        self.target_obj_path = "/mobility_" + self.obj_type + "_" + str(self.obj_id) 
        print("AUTOTASK_META[self.task_type][self.meta_id]", AUTOTASK_META[self.task_type][self.meta_id]) 
        AutoTasker.TASK_DESCRIPTION = AUTOTASK_META[self.task_type][self.meta_id]["goal"]["description"]

        print("AutoTasker.TASK_DESCRIPTION", AutoTasker.TASK_DESCRIPTION) 

    def add_obj(self):
        """
        Add object to the scene
        """
        self.stage = omni.usd.get_context().get_stage()
        # set up game root
        default_prim_path_str = self.stage.GetDefaultPrim().GetPath().pathString
        ## this is necessary because for standalone this might not be /World
        if not default_prim_path_str:
            default_prim_path_str = "/World"
        self.xform_game_path = default_prim_path_str + "/game" # omni.usd.get_stage_next_free_path(self.stage, "/World/game", True)
        # move obj to the correct place
        xform_game = self.stage.GetPrimAtPath(self.xform_game_path)
        if not xform_game:
            xform_game = pxr.UsdGeom.Xform.Define(self.stage, self.xform_game_path)

        # set game xform 
        game_xform = pxr.Gf.Matrix4d().SetScale([1, 1, 1]) *  \
            pxr.Gf.Matrix4d().SetRotate(pxr.Gf.Quatf(1.0,0.0,0.0,0.0)) * pxr.Gf.Matrix4d().SetTranslate([0,0,0])

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=self.xform_game_path,
            new_transform_matrix=game_xform,
        )
        
        # set obj prim path
        mobility_prim_path = xform_game.GetPath().pathString + self.target_obj_path 
        print("mobility_prim_path", mobility_prim_path)
        prim = self.stage.GetPrimAtPath(mobility_prim_path)
        if not prim.IsValid():
            prim = self.stage.DefinePrim(mobility_prim_path)
        
        if self.task_type in ["pour_water", "transfer_water"]:
            obj_usd_path = os.path.join(self.obj_folder, self.obj_id, "cup.usd")
        else:
            obj_usd_path = os.path.join(self.obj_folder, self.obj_id, "mobility.usd")

        # import obj
        success_bool = prim.GetReferences().AddReference(obj_usd_path)
        
        if not success_bool:
            raise Exception(f"Cannot import obj usd at path {obj_usd_path}")

        # set up scale
        if self.task_type in ["open_door", "close_door"]:
            from .utils import calculate_door_size
            scale = calculate_door_size(prim)
        else:   
            scale  = [AUTOTASK_META[self.task_type][self.meta_id]["size"]]*3

        if prim.HasAttribute("xformOp:scale"):
            prim.GetAttribute("xformOp:scale").Set(pxr.Gf.Vec3f(scale))
        else:
            obj_xform = pxr.Gf.Matrix4d().SetScale(scale)
            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=prim.GetPath().pathString,
                new_transform_matrix=obj_xform,
            )  

        # set up orient
        #if self.task_type  "reorient_object":
        orient  = AUTOTASK_META[self.task_type][self.meta_id]["orient"]
        print("orient: ", orient)
        mat = pxr.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        obj_xform = pxr.Gf.Matrix4d().SetRotate(pxr.Gf.Quatf(*orient))
        new_xform = obj_xform * mat
        print("new_xform", new_xform, "\n scale:", scale)
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath().pathString,
            new_transform_matrix=mat,
        ) 

        # other imports
        if self.task_type in ["put_object_into_box", "transfer_water", "tap_water"]:
            self.add_auxilary_object()

        # unbind material
        if self.task_type in ["transfer_water", "pour_water"]:
            print("unbind material")
            omni.kit.commands.execute(
                'BindMaterial',
                prim_path=prim.GetPath().pathString + "/cupShape",
                material_path=None,
                strength=pxr.UsdShade.Tokens.strongerThanDescendants
                )

    def add_auxilary_object(self):
        """
        Add object to the scene
        """
        self.stage = omni.usd.get_context().get_stage()
        # set up game root
        default_prim_path_str = self.stage.GetDefaultPrim().GetPath().pathString
        ## this is necessary because for standalone this might not be /World
        if not default_prim_path_str:
            default_prim_path_str = "/World"
        self.xform_game_path = default_prim_path_str + "/game" # omni.usd.get_stage_next_free_path(self.stage, "/World/game", True)
        # move obj to the correct place
        xform_game = self.stage.GetPrimAtPath(self.xform_game_path)
        if not xform_game:
            raise Exception(f"must have /World/game prim")

        if self.task_type == "put_object_into_box":
            aux_folder = os.path.join(CUSTOM_ASSET_PATH, "standalone")
            aux_folder_objs = os.listdir(aux_folder)
            aux_obj_name = aux_folder_objs[self.obj_index + 12]
            
            aux_prim_path = xform_game.GetPath().pathString + "/mobility_standalone_" +  aux_obj_name
            obj_usd_path = os.path.join(aux_folder, aux_obj_name, "mobility.usd")
            position = [-20,0,0]

        else:
            aux_folder = os.path.join(CUSTOM_ASSET_PATH, "Cup")
            aux_folder_objs = sorted(os.listdir(aux_folder), key=lambda x:int(x))
            aux_obj_name = str(int(self.task_id))
            
            aux_prim_path = xform_game.GetPath().pathString + "/container_Cup_" +  aux_obj_name
            obj_usd_path = os.path.join(aux_folder, aux_obj_name, "cup.usd")
            position = [0,0,-20]

        # print("aux_prim_path", aux_prim_path)
        prim = self.stage.GetPrimAtPath(aux_prim_path)
        if not prim.IsValid():
            prim = self.stage.DefinePrim(aux_prim_path)
        
        success_bool = prim.GetReferences().AddReference(obj_usd_path)
        
        if not success_bool:
            raise Exception(f"Cannot import obj usd at path {obj_usd_path}")

        # offset
        if True:
            purposes = [pxr.UsdGeom.Tokens.default_]
            bboxcache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), purposes)
            game_prim = self.stage.GetPrimAtPath(self.xform_game_path)
            bboxes = bboxcache.ComputeWorldBound(game_prim)
            # print("bboxes", bboxes)
            game_bboxes = [bboxes.ComputeAlignedRange().GetMin(),bboxes.ComputeAlignedRange().GetMax()]
        else:
            game_bboxes =  omni.usd.get_context().compute_path_world_bounding_box(self.xform_game_path)
        position[1] += game_bboxes[0][1] # the same y
        position[0] += game_bboxes[0][0] # offset x
        position[2] += game_bboxes[0][2] # offset x

        # set up scale 
        obj_xform = pxr.Gf.Matrix4d().SetScale([1,1,1]).SetRotate(pxr.Gf.Quatf(1,0,0,0)).SetTranslate(position)
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath().pathString,
            new_transform_matrix=obj_xform,
        ) 

        # unbind material
        if self.task_type in ["transfer_water", "pour_water"]:
            print("unbind material")
            omni.kit.commands.execute(
                'BindMaterial',
                prim_path=prim.GetPath().pathString + "/cupShape",
                material_path=None,
                strength=pxr.UsdShade.Tokens.strongerThanDescendants
                )

    def add_robot(self):
        """
        Add robot to the scene:
        1. load robot
        2. calculate position
        """
        self.stage = omni.usd.get_context().get_stage()
        franka_path = os.path.join(ROBOT_PATH, "franka/franka.usd")
        
        self.xform_game_path  = "/World/game"
        

        # position, rotation
        position = [i for i in AUTOTASK_META[self.task_type][self.meta_id]["robot_offset"]]
        rotation = [i for i in AUTOTASK_META[self.task_type][self.meta_id]["robot_orient"]]
        
        # offset
        if True: ##IS_IN_ISAAC_SIM:
            purposes = [pxr.UsdGeom.Tokens.default_]
            bboxcache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), purposes)
            prim = self.stage.GetPrimAtPath(self.xform_game_path)
            bboxes = bboxcache.ComputeWorldBound(prim)
            # print("bboxes", bboxes)
            game_bboxes = [bboxes.ComputeAlignedRange().GetMin(),bboxes.ComputeAlignedRange().GetMax()]
        else:
            game_bboxes =  omni.usd.get_context().compute_path_world_bounding_box(self.xform_game_path)
        
        print("game_bboxes", game_bboxes)
        position[1] += game_bboxes[0][1]

        # print("game_bboxes", game_bboxes, position)
        if position[0] != 0 :
            position[0] += game_bboxes[0][0]
        if position[2] != 0 :
            position[2] += game_bboxes[0][2] 
        
        # load robot
        robot_prim = self.stage.GetPrimAtPath(self.xform_game_path + "/franka")
        if not robot_prim.IsValid():
            robot_prim = self.stage.DefinePrim(self.xform_game_path + "/franka")
        
        print("add robot at position: ", position)
        success_bool = robot_prim.GetReferences().AddReference(franka_path)
        if not success_bool:
            raise Exception("The usd file at path {} provided wasn't found".format(franka_path))
            
        # set robot xform
        robot_xform = pxr.UsdGeom.Xformable.Get(self.stage, robot_prim.GetPath())
        robot_xform.ClearXformOpOrder()

        # print("position $ rotation: ", position[0], position[1], position[2], rotation)
        robot_xform.AddTranslateOp().Set(pxr.Gf.Vec3f(float(position[0]), float(position[1]), float(position[2])))
        robot_xform.AddOrientOp().Set(pxr.Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])))
        robot_xform.AddScaleOp().Set(pxr.Gf.Vec3f(1.0, 1.0, 1.0))

        #selection = omni.usd.get_context().get_selection()
        #selection.clear_selected_prim_paths()
        #selection.set_prim_path_selected(robot_parent_path + "/franka", True, True, True, True)

    def add_house(self):
        """
        Add house from house_d
        """
        print("auto add house??")

        # scene
        self.stage = omni.usd.get_context().get_stage() 
        self.layer = self.stage.GetRootLayer()
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

        # move obj to the correct place
        house_prim_path = "/World/layout"
        house_prim = self.stage.GetPrimAtPath(house_prim_path)
        if not house_prim.IsValid():
            house_prim = self.stage.DefinePrim(house_prim_path)

        success_bool = house_prim.GetReferences().AddReference(house_path)

        if not success_bool:
            raise Exception(f"The house is not load at {house_path}")

        if not self.task_type in ["tap_water", "transfer_water", "pour_water"]:
            from omni.physx.scripts.utils import setStaticCollider
            # static collider
            furniture_prim = self.stage.GetPrimAtPath(house_prim_path + "/furniture")
            setStaticCollider(furniture_prim, approximationShape="none")

            # TODO: check room_struct collider
            room_struct_prim = self.stage.GetPrimAtPath(house_prim_path + "/roomStruct")
            setStaticCollider(room_struct_prim, approximationShape="none")

        # put game onto ground
        game_prim_path = "/World/game"
        game_prim = self.stage.GetPrimAtPath(game_prim_path)
        if game_prim:
            if True: #IS_IN_ISAAC_SIM:
                purposes = [pxr.UsdGeom.Tokens.default_]
                bboxcache = pxr.UsdGeom.BBoxCache(pxr.Usd.TimeCode.Default(), purposes)
                
                bboxes = bboxcache.ComputeWorldBound(game_prim)
                # print("bboxes", bboxes)
                y = bboxes.ComputeAlignedRange().GetMin()[1]
            else:
                # prim_path = stage.GetDefaultPrim().GetPath().pathString
                usd_context = omni.usd.get_context()
                bboxes = usd_context.compute_path_world_bounding_box(game_prim_path)
                y = bboxes[0][1]
            
            game_xform = pxr.Gf.Matrix4d().SetScale([1, 1, 1]) *  \
            pxr.Gf.Matrix4d().SetRotate(pxr.Gf.Quatf(1.0,0.0,0.0,0.0)) * pxr.Gf.Matrix4d().SetTranslate([0,-y,0])

            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=game_prim_path,
                new_transform_matrix=game_xform,
            )

            # add ground
            ground_prim = self.stage.GetPrimAtPath("/World/groundPlane")
            if not ground_prim:
                physicsUtils.add_ground_plane(self.stage, "/World/groundPlane", "Y", 1000.0, 
                    pxr.Gf.Vec3f(0.0, 0.0, 0), pxr.Gf.Vec3f(0.2))
                ground_prim = self.stage.GetPrimAtPath("/World/groundPlane")
            
            # prim_list = list(self.stage.TraverseAll())
            # prim_list = [ item for item in prim_list if 'groundPlane' in item.GetPath().pathString and item.GetTypeName() == 'Mesh' ]
            # for prim in prim_list:
            ground_prim.GetAttribute('visibility').Set('invisible')



    def add_task(self):
        """
        Add task to current scene 
        """
        self.stage = omni.usd.get_context().get_stage()

        self.task_checker = BaseChecker(self.task_type, self.task_id, self.robot_id, self.mission_id, annotator = "Yizhou", run_time = False)
        # if self.task_type in ["open_drawer", "open_cabinet", "open_door", "close_door"]:
        #     self.task_checker = JointChecker(self.task_type, self.task_id, self.robot_id, self.mission_id)
        # elif self.task_type == "pickup_object":
        #     self.task_checker = GraspChecker(self.task_type, self.task_id, self.robot_id, self.mission_id)
        # elif self.task_type == "reorient_object":
        #     self.task_checker = OrientChecker(self.task_type, self.task_id, self.robot_id, self.mission_id)
        # elif self.task_type in ["put_object_into_box"]:
        #     self.task_checker = ContainerChecker(self.task_type, self.task_id, self.robot_id, self.mission_id)
        # else:
        #     raise Exception(f"Current task type {self.task_type} not supported")
        # modify task from template 
        # print(AUTOTASK_META[self.task_type][self.meta_index]["task_template"])
        self.task_checker.current_mission  = AUTOTASK_META[self.task_type][self.meta_id]
        condition = self.task_checker.current_mission["goal"]["condition"]

        # get target 
        target_prim = None
        for prim in self.stage.GetPrimAtPath("/World/game").GetChildren():
            for game_name in GAME_OBJ_NAMES:
                if game_name in prim.GetPath().pathString:
                    target_prim = prim
                    break
        
        condition["target"] = target_prim.GetPath().pathString.split("/")[-1]

        # other condition
        if self.task_type in ["open_drawer", "open_cabinet", "open_door", "close_door", "close_drawer", "close_cabinet"]:
            selection = omni.usd.get_context().get_selection()
            
            assert len(selection.get_selected_prim_paths()) == 1, "Please select one joint!"

            joint_path = selection.get_selected_prim_paths()[0]
            joint_name = joint_path.split("/")[-1]
            # print("joint_name:", joint_name)
            self.task_checker.current_mission["goal"]
            condition["joint"] = joint_name
        elif self.task_type in ["put_object_into_box", "transfer_water", "take_object_out_box", "tap_water"]:
            container_prim = None
            for prim in self.stage.GetPrimAtPath("/World/game").GetChildren():
                for game_name in CONTAINER_NAMES:
                    if game_name in prim.GetPath().pathString.lower():
                        container_prim = prim
                        break
            
            if not container_prim:
                raise Exception(f"Container prim must exist at under /World/game")
            
            condition["container"] = container_prim.GetPath().pathString.split("/")[-1]

        # save mission
        self.task_checker.current_mission["goal"]["description"] = AutoTasker.TASK_DESCRIPTION
        print("current_mission", self.task_checker.current_mission)
        self.task_checker.current_mission["goal"]["condition"] = condition
        self.task_checker.save_mission()

    @classmethod
    def new_scene(cls):
        async def open_new_scene():
            await omni.usd.get_context().new_stage_async()
            await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(open_new_scene())
    
    def build_HUD(self):
        if IS_IN_CREAT or IS_IN_ISAAC_SIM:
            gui_path = self.stage.GetDefaultPrim().GetPath().pathString + "/GUI"
            gui = self.stage.GetPrimAtPath(gui_path)
            if not gui:
                gui = pxr.UsdGeom.Xform.Define(self.stage, gui_path)
                gui_location = pxr.Gf.Vec3f(0, 50, 0)
                gui.AddTranslateOp().Set(gui_location)

                self.wiget_id = wm.add_widget(gui_path, LabelWidget(f"Object id: {self.obj_id}"), wm.WidgetAlignment.TOP)

