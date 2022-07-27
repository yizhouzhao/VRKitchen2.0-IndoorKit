############# omniverse import ##################
import omni.ext
import omni.ui as ui
import carb
import pxr


############# python import ##################
import asyncio
import os
import time
import random
import math
import json
import numpy as np

############# VRKitchen import ##################
from .param import *
from .layout.house import House
from .layout.randomizer import Randomizer
from .layout.utils import rotationXYZ_to_quaternion

from .layout.house_new import House as HouseNew
from .task_check import GraspChecker, JointChecker, OrientChecker, ContainerChecker, WaterChecker, TapWaterChecker, BaseChecker
from .autotask.auto import AutoTasker
from .autotask.auto_label import AutoLabeler

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[vrkitchen.indoor.kit] VRKitchen2.0-Indoor-Kit startup")

        # set rendering settings:
        carb.settings.get_settings().set_bool("/rtx/ecoMode/enabled", True)
        FPS = 60.0
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int( FPS))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(FPS))

        # models
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.franka = None
        self.auto_labeler = AutoLabeler(None)
        self.task_type = None

        # build windows
        self.build_setup_layout_window()
        self.build_labeling_window() 
    
    ################################################################################################
    ######################################## Build omni ui window ##################################
    ################################################################################################

    def build_setup_layout_window(self):
        """
        Build a window to control/debug layout 
        """
        self._window = ui.Window("VRKitchen2.0-Indoor-Kit", width=600)
        with self._window.frame:
            with ui.VStack():
                ui.Label("\n Set up Layout", style = {"font_size": 20, "margin": 10}, height = 30, alignment=ui.Alignment.CENTER)
                self.task_desc_ui = omni.ui.StringField(height=20, style={ "margin_height": 2 })
                with ui.HStack(height=30):
                    # set up tasks 
                    self.task_types = TASK_TYPES
                    ui.Label(" Task type: ", width = 30, style={ "margin": 2 })
                    default_task_index = self.task_types.index("pickup_object")
                    self.task_type_ui = ui.ComboBox(default_task_index, width = 120, *self.task_types, style={ "margin": 8 })

                    ui.Label(" Task id: ", width=20)
                    self.task_id_ui = omni.ui.IntField(height=20, width = 60, style={ "margin": 8 })    

                    ui.Button(" + ", clicked_fn=self.auto_next_task, width = 20, style={ "margin_height": 8})
                    # ui.Button("+ object id", clicked_fn=self.auto_next_obj_only, style={ "margin": 8})

                with ui.HStack(height=30):
                    ui.Label("Object ", width=20)
                    self.object_id_ui = omni.ui.IntField(height=20, width = 25, style={ "margin_height": 8 , "margin_width": 4})
                    self.object_id_ui.model.set_value(0)
                    ui.Button("+", width = 20, style={"margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.object_id_ui.model.set_value(self.object_id_ui.model.get_value_as_int() + 1))
                    ui.Button("-", width = 20, style={ "margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.object_id_ui.model.set_value(self.object_id_ui.model.get_value_as_int() - 1))

                    ui.Label("  Anchor:", width=20, visible = False)
                    self.anchor_id_ui = omni.ui.IntField(height=20, width = 25, style={ "margin_height": 8 , "margin_width": 4},  visible = False)
                    self.anchor_id_ui.model.set_value(0)
                    ui.Button("+", width = 20, style={"margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.anchor_id_ui.model.set_value(self.anchor_id_ui.model.get_value_as_int() + 1),  visible = False)
                    ui.Button("-", width = 20, style={ "margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.anchor_id_ui.model.set_value(self.anchor_id_ui.model.get_value_as_int() - 1),  visible = False)
   

                    ui.Label("  Robot:", width=20)
                    self.robot_id_ui = omni.ui.IntField(height=20, width = 25, style={ "margin_height": 8 , "margin_width": 4})
                    ui.Button("+", width = 20, style={"margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.robot_id_ui.model.set_value(self.robot_id_ui.model.get_value_as_int() + 1))
                    ui.Button("-", width = 20, style={ "margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.robot_id_ui.model.set_value(self.robot_id_ui.model.get_value_as_int() - 1))
   
                    ui.Label("Mission ", width=20, visible = False)
                    self.mission_id_ui = omni.ui.IntField(height=20, width = 40, style={ "margin": 8 })
                    
                    ui.Label(" | ", width=10)

                    ui.Label("House:", width=20)
                    self.house_id_ui = omni.ui.IntField(height=20, width = 25, style={ "margin_height": 8, "margin_width": 4})
                    self.house_id_ui.model.set_value(0)
                    ui.Button("+", width = 20, style={"margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.house_id_ui.model.set_value(self.house_id_ui.model.get_value_as_int() + 1))
                    ui.Button("-", width = 20, style={ "margin_height": 8, "font_size": 12},
                        clicked_fn=lambda: self.house_id_ui.model.set_value(self.house_id_ui.model.get_value_as_int() - 1))
   
                 

                with ui.HStack(height=20):
                    ui.Button("Add obj", clicked_fn=self.auto_add_obj, style={ "margin": 4})
                    ui.Button("Add robot", clicked_fn=self.auto_add_robot, style={ "margin": 4})
                    ui.Button("Add mission", clicked_fn=self.auto_add_mission, style={ "margin": 4}, visible = False)
                    ui.Label(" |", width=10)
                    ui.Button("Add house", clicked_fn=self.auto_add_house, style={ "margin": 4})

                with ui.HStack(height=20):
                    ui.Button("Record obj", clicked_fn=self.record_obj_new, style={ "margin": 4})
                    ui.Button("Record robot", clicked_fn=self.record_robot_new, style={ "margin": 4})
                    ui.Label(" |", width=10)
                    ui.Button("Record house", clicked_fn=self.record_house_new, style={ "margin": 4})
                
                with ui.HStack(height=20):
                    ui.Button("Load obj", clicked_fn=self.load_obj_new, style={ "margin": 4})
                    ui.Button("Load robot", clicked_fn=self.load_robot_new, style={ "margin": 4})
                    # ui.Button("Load mission", clicked_fn=self.load_mission, style={ "margin": 4})      
                    ui.Label(" |", width=10)
                    ui.Button("Load house", clicked_fn=self.load_house_new, style={ "margin": 4})

                ui.Label("\n Scene utility", style = {"font_size": 20, "margin": 10}, height = 30, alignment=ui.Alignment.CENTER)
                with ui.HStack(height=30):
                    ui.Button("Add Ground", clicked_fn=self.auto_add_ground, style={ "margin": 2})
                with ui.HStack(height=30):
                    ui.Button("Randomize house material", clicked_fn=self.randomize_material, style={ "margin": 2}) 
                    ui.Button("Randomize sky", clicked_fn=self.randomize_sky, style={ "margin": 2}) 
                    ui.Button("Randomize light", clicked_fn=self.randomize_light, style={ "margin": 2}) 
        
    
    ################################################################################################
    ######################################## Auto task labeling ####################################
    ################################################################################################              

    def init_auto_tasker(self): 
        """
        Initialize auto task labeling tool
        """
        task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
        task_type = self.task_types[task_index]
        task_id = self.task_id_ui.model.get_value_as_int()
        robot_id = self.robot_id_ui.model.get_value_as_int()
        anchor_id = self.anchor_id_ui.model.get_value_as_int()
        mission_id = self.mission_id_ui.model.get_value_as_int()
        house_id = self.house_id_ui.model.get_value_as_int()
        # meta_id = self.meta_id_ui.model.get_value_as_int() 
        
        # FIXME: add annotator
        # annotator_index = self.annotator_ui.model.get_item_value_model().get_value_as_int()
        annotator = "Yizhou" # self.annotators[annotator_index]
        
        self.auto_tasker = AutoTasker(task_type, task_id, robot_id, mission_id, house_id, anchor_id, annotator=annotator)
        AutoTasker.TASK_DESCRIPTION = self.task_desc_ui.model.get_value_as_string()

    def auto_next_obj_only(self):
        """
        retrieve the next object index for current task
        """
        # new scene
        AutoTasker.new_scene() 
        global OBJ_INDEX
        OBJ_INDEX = self.object_id_ui.model.get_value_as_int()
        OBJ_INDEX += 1
        self.object_id_ui.model.set_value(OBJ_INDEX)

        self.init_auto_tasker()
        self.auto_tasker.reconfig(OBJ_INDEX)

        self.task_desc_ui.model.set_value(AutoTasker.TASK_DESCRIPTION)
        
    def auto_next_task(self):
        """
        next task
        """
        task_id = self.task_id_ui.model.get_value_as_int()
        self.task_id_ui.model.set_value(task_id + 1)
        AutoTasker.new_scene() 
        self.init_auto_tasker()
        self.auto_tasker.reconfig(0)
        self.task_desc_ui.model.set_value(AutoTasker.TASK_DESCRIPTION)

    
    def auto_next_task(self):
        """
        next task
        """
        task_id = self.task_id_ui.model.get_value_as_int()
        self.task_id_ui.model.set_value(task_id + 1)
        AutoTasker.new_scene() 
        self.init_auto_tasker()
        self.auto_tasker.reconfig(0)
        self.task_desc_ui.model.set_value(AutoTasker.TASK_DESCRIPTION)

    def auto_add_obj(self):    
        self.init_auto_tasker()
        self.auto_tasker.add_obj() 
        self.auto_tasker.build_HUD()
        
    def auto_add_robot(self):
        self.init_auto_tasker()
        self.auto_tasker.add_robot()
    
    def auto_add_house(self):
        self.init_auto_tasker()
        self.auto_tasker.add_house()
    
    def auto_add_mission(self):
        self.init_auto_tasker()
        self.auto_tasker.add_task()

    def on_shutdown(self):
        print("[vrkitchen.indoor.kit] VRKitchen2.0-Indoor-Kit shutdown")


    ################################################################################################
    ######################################## Modify Scene ##########################################
    ################################################################################################              

    def auto_add_ground(self):
        """
        Add ground to the scene
        """
        from .layout.modify import add_ground_plane
        add_ground_plane()
        self.task_desc_ui.model.set_value("Add ground to scene (not visible)")

    def randomize_material(self):
        """
        Randomize house materials
        """
        self.randomizer = Randomizer()
        self.randomizer.randomize_house()
        self.task_desc_ui.model.set_value("Random floor/wall material")

    def randomize_sky(self):
        """
        Randomize house materials
        """
        self.randomizer = Randomizer()
        self.randomizer.randomize_sky()
        self.task_desc_ui.model.set_value("Random sky")
    
    def randomize_light(self):
        """
        Randomize house materials
        """
        self.randomizer = Randomizer()
        self.randomizer.randomize_light()
        self.task_desc_ui.model.set_value("Random light")

    
    ################################################################################################
    ######################################## Load / Record #########################################
    ################################################################################################              

    def init_new_house(self):
        """
        Initiate HouseNew for recording/loading task info
        """
        task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
        task_type = self.task_types[task_index]
        task_id = self.task_id_ui.model.get_value_as_int()
        robot_id = self.robot_id_ui.model.get_value_as_int()
        anchor_id = self.anchor_id_ui.model.get_value_as_int()
        mission_id = self.mission_id_ui.model.get_value_as_int()
        house_id = self.house_id_ui.model.get_value_as_int()

        annotator_index = self.annotator_ui.model.get_item_value_model().get_value_as_int()
        annotator = self.annotators[annotator_index]

        self.house = HouseNew(task_type, task_id, robot_id, mission_id, house_id, anchor_id, annotator)
        self.house.build_HUD()
        # print("robot", self.house.robot_id) 

    def record_obj_new(self):
        """
        New pipeline to record game objects
        """
        self.init_new_house()
        self.house.record_obj_info()


    def record_robot_new(self):
        """
        New pipeline to record game robots
        """
        self.init_new_house()
        self.house.record_robot_info()

        if BaseChecker.SUCCESS_UI:
            BaseChecker.SUCCESS_UI.model.set_value("robot id (robot variation) recorded")
    
    def record_house_new(self):
        self.init_new_house()
        self.house.record_house_info()

        if BaseChecker.SUCCESS_UI:
            BaseChecker.SUCCESS_UI.model.set_value("house-anchor recorded")

    def load_obj_new(self):
        """
        New pipeline to load game objs
        """
        stage = omni.usd.get_context().get_stage()
        default_prim_path = stage.GetDefaultPrim().GetPath()
        if default_prim_path.pathString == '':
            # default_prim_path = pxr.Sdf.Path('/World')
            root = pxr.UsdGeom.Xform.Define(stage, "/World").GetPrim()
            stage.SetDefaultPrim(root)
            default_prim_path = stage.GetDefaultPrim().GetPath()

        self.init_new_house()
        self.house.load_obj_info(relative=True)

        task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
        task_type = self.task_types[task_index]
        
        # fix linear joint scale
        if task_type in ["open_drawer","open_cabinet", "open_door", \
            "close_drawer", "close_cabinet", "close_door", "tap_water"]:

            if task_type in ["open_door", "close_door"]:
                self.fix_linear_joint(fix_driver=True, damping_cofficient=1000)
            elif task_type in ["tap_water"]:
                self.fix_linear_joint(fix_driver=True, damping_cofficient=100)
            else:
                self.fix_linear_joint(fix_driver=True, damping_cofficient=10)
        if task_type in ["pour_water", "transfer_water", "tap_water"]:
            self.add_liquid_to_cup(task_type)

    def load_robot_new(self):
        """
        New pipeline to load robots objs
        """
        self.is_initial_setup = False
        self.init_new_house()
        self.setup_robot(new_method=True)
    
    def load_house_new(self):
        self.init_new_house()
        self.load_house_successful = self.house.load_house_info()

        # if load house successfully, randomize sky, floor, and wall
        if self.load_house_successful:
            from .layout.randomizer import Randomizer
            
            if not hasattr(self, "house_randomizer"):
                self.house_randomizer = Randomizer(None)
            
            self.house_randomizer.randomize_house(randomize_floor=True, randomize_wall=True)
            # if IS_IN_CREAT:
            #     self.house_randomizer.randomize_sky()

    ################################################################################################
    ######################################## Second window #########################################
    ################################################################################################              

    
    def build_labeling_window(self):
        """
        Build another window for labeling only
        """
        from .autotask.auto_suggest import AutoSuggest
        self.auto_suggest = AutoSuggest()

        self._window2 = ui.Window("VRKitchen Labeling", width=500, dockPreference=ui.DockPreference.LEFT_BOTTOM)
        with self._window2.frame:
            with ui.VStack():
                # ui.Label("Labeling Helper", height=20, style = {"font_size": 20, "margin": 10}, alignment=ui.Alignment.CENTER)
                with ui.HStack(height = 20):
                    # task type
                    self.suggest_task_types = ["None"] + TASK_TYPES
                    ui.Label("Task type: ", width=20, style={"font_size": 12, "color": "PeachPuff"})
                    self.auto_suggest.suggest_task_type_ui = ui.ComboBox(0, width = 140, *self.suggest_task_types, 
                        style={ "margin_height": 2, "font_size": 12, "color": "PeachPuff"})
                    self.annotators = ANNOTATORS
                    ui.Label("  Annotator: ", width = 30, style={ "font_size": 12 , "color": "PowderBlue"})
                    annotator_index = ANNOTATORS.index("Yizhou")
                    self.annotator_ui = ui.ComboBox(annotator_index, width = 100, *self.annotators, style={ "margin_height": 2, "font_size": 12, "color": "PowderBlue" })
                    self.auto_suggest.annotator_ui = self.annotator_ui
                    # self.suggestion_task_type_ui.model.add_item_changed_fn(functools.partial(task_type_suggestion_call_back))
                    self.auto_suggest.suggest_task_type_ui.model.add_item_changed_fn(self._on_suggestion_task_type_change)
                       
                with ui.HStack(height = 20):
                    BaseChecker.SUCCESS_UI = ui.StringField(style={ "margin": 2, "font_size": 12, "color": "red"}) #  read_only  = True 
                
                with ui.HStack(height = 4):
                    ui.Label("-"*120)
                
                with ui.HStack(height = 20):
                    self.csv_row_id = -1
                    self.csv_row_id_model = ui.SimpleIntModel(self.csv_row_id) 
                    ui.Label("Csv row: ", width=20, style={"font_size": 12, "color": "LightYellow"})
                    self.auto_label_csv_row_id_ui = ui.IntField(model = self.csv_row_id_model, width = 30, style={ "margin": 2, "font_size": 12, "color": "LightYellow"})
                    ui.Button(" Next ", clicked_fn=self.auto_label_load_next, style={ "margin_height": 2, "font_size": 12, "color": "LightYellow"})
                    self.franka_easy_control_button = ui.Button("EE Rotation Control", clicked_fn=self._on_franka_easy_control_button_clicked, style={ "margin_height": 2, "font_size": 12, "color": "Aqua"})
                    
                    # ui.Button("", width = 40, clicked_fn=self.load_obj_new, style={ "margin_height": 2, "font_size": 12, "color": "darkorange"})

                    self.csv_row_id_model.add_end_edit_fn(lambda model: self._on_csv_row_id_changed(model))

                
                with ui.HStack(height = 20):
                    # task id
                    self.auto_suggest.suggest_task_id_model = ui.SimpleIntModel(-1) 
                    suggest_task_id_model = self.auto_suggest.suggest_task_id_model 
                    ui.Label("Task: ", width=20, style={"font_size": 12, "color": "darkorange"})
                    self.auto_suggest.suggest_task_id_ui = ui.IntField(model = suggest_task_id_model, width = 20, style={ "margin": 2, "font_size": 12, "color": "darkorange"})
                    self.auto_suggest.suggest_task_id_text_ui = ui.StringField(style={ "margin": 2, "font_size": 12, "color": "darkorange"}, visible = False)
                    ui.Button("+", width = 10, style={"margin_height": 2, "font_size": 12, "color": "darkorange", },
                        clicked_fn=lambda: suggest_task_id_model.set_value(suggest_task_id_model.get_value_as_int() + 1))
                    ui.Button("-", width = 10, style={ "margin_height": 2, "font_size": 12, "color": "darkorange"},
                        clicked_fn=lambda: suggest_task_id_model.set_value(suggest_task_id_model.get_value_as_int() - 1))
                    
                    suggest_task_id_model.add_value_changed_fn(lambda model: self._on_suggestion_task_id_changed(model))
                            
                #with ui.HStack(height = 30):
                    # robot id
                    self.auto_suggest.suggest_robot_id_model = ui.SimpleIntModel(-1) 
                    suggest_robot_id_model = self.auto_suggest.suggest_robot_id_model 
                    ui.Label("  Robot: ", width=20, style={"font_size": 12, "color": "cyan"})
                    self.auto_suggest.suggest_robot_id_ui = ui.IntField(model = suggest_robot_id_model, width = 20, style={ "margin": 2, "font_size": 12, "color": "cyan"})
                    self.auto_suggest.suggest_robot_id_text_ui = ui.StringField(style={ "margin": 2, "font_size": 12, "color": "cyan"}, visible = False)
                    ui.Button("+", width = 10, style={"margin_height": 2, "font_size": 12, "color": "cyan", },
                        clicked_fn=lambda: suggest_robot_id_model.set_value(suggest_robot_id_model.get_value_as_int() + 1))
                    ui.Button("-", width = 10, style={ "margin_height": 2, "font_size": 12, "color": "cyan"},
                        clicked_fn=lambda: suggest_robot_id_model.set_value(suggest_robot_id_model.get_value_as_int() - 1))
                    
                    suggest_robot_id_model.add_value_changed_fn(lambda model: self._on_suggestion_robot_id_changed(model))
                     

                # with ui.HStack(height = 20):
                    # mission id
                    self.auto_suggest.suggest_mission_id_model = ui.SimpleIntModel(-1) 
                    suggest_mission_id_model = self.auto_suggest.suggest_mission_id_model 
                    ui.Label("  Mission: ", width=20, style={"font_size": 12, "color": "lightgreen"})
                    self.auto_suggest.suggest_mission_id_ui = ui.IntField(model = suggest_mission_id_model, width = 20, style={ "margin": 2, "font_size": 12, "color": "lightgreen"})
                    self.auto_suggest.suggest_mission_id_text_ui = ui.StringField(style={ "margin": 2, "font_size": 12, "color": "lightgreen"}, visible = False)
                    ui.Button("+", width = 10, style={"margin_height": 2, "font_size": 12, "color": "lightgreen", },
                        clicked_fn=lambda: suggest_mission_id_model.set_value(suggest_mission_id_model.get_value_as_int() + 1))
                    ui.Button("-", width = 10, style={ "margin_height": 2, "font_size": 12, "color": "lightgreen"},
                        clicked_fn=lambda: suggest_mission_id_model.set_value(suggest_mission_id_model.get_value_as_int() - 1))
                    
                    suggest_mission_id_model.add_value_changed_fn(lambda model: self._on_suggestion_mission_id_changed(model))
                   
                #with ui.HStack(height = 20):
                    # house id
                    self.auto_suggest.suggest_house_id_model = ui.SimpleIntModel(-1) 
                    suggest_house_id_model = self.auto_suggest.suggest_house_id_model 
                    ui.Label("House: ", width=20, style={"font_size": 12, "color": "SkyBlue"})
                    self.auto_suggest.suggest_house_id_ui = ui.IntField(model = suggest_house_id_model, width = 20, style={ "margin": 2, "font_size": 12, "color": "SkyBlue"})
                    self.auto_suggest.suggest_house_id_text_ui = ui.StringField(style={ "margin": 2, "font_size": 12, "color": "SkyBlue"}, visible = False)
                    ui.Button("+", width = 10, style={"margin_height": 2, "font_size": 12, "color": "SkyBlue", },
                        clicked_fn=lambda: suggest_house_id_model.set_value(suggest_house_id_model.get_value_as_int() + 1))
                    ui.Button("-", width = 10, style={ "margin_height": 2, "font_size": 12, "color": "SkyBlue"},
                        clicked_fn=lambda: suggest_house_id_model.set_value(suggest_house_id_model.get_value_as_int() - 1))
                    
                    suggest_house_id_model.add_value_changed_fn(lambda model: self._on_suggestion_house_id_changed(model))
      
                #with ui.HStack(height = 30):
                    # anchor id
                    self.auto_suggest.suggest_anchor_id_model = ui.SimpleIntModel(-1) 
                    suggest_anchor_id_model = self.auto_suggest.suggest_anchor_id_model 
                    ui.Label("  Anchor: ", width=20, style={"font_size": 12, "color": "HotPink"})
                    self.auto_suggest.suggest_anchor_id_ui = ui.IntField(model = suggest_anchor_id_model, width = 20, style={ "margin": 2, "font_size": 12, "color": "HotPink"})
                    self.auto_suggest.suggest_anchor_id_text_ui = ui.StringField(style={ "margin": 2, "font_size": 12, "color": "HotPink"}, visible = False)
                    ui.Button("+", width = 10, style={"margin_height": 2, "font_size": 12, "color": "HotPink", },
                        clicked_fn=lambda: suggest_anchor_id_model.set_value(suggest_anchor_id_model.get_value_as_int() + 1))
                    ui.Button("-", width = 10, style={ "margin_height": 2, "font_size": 12, "color": "HotPink"},
                        clicked_fn=lambda: suggest_anchor_id_model.set_value(suggest_anchor_id_model.get_value_as_int() - 1))
                    
                    suggest_anchor_id_model.add_value_changed_fn(lambda model: self._on_suggestion_anchor_id_changed(model))

                with ui.HStack(height = 4):
                    ui.Label("-"*120)
                
                with ui.HStack(height = 10, visible = False):
                    ui.Button("Load task", width = 40, clicked_fn=self.load_obj_new, style={ "margin_height": 2, "font_size": 12, "color": "darkorange"})
                    ui.Button("Load robot", width = 40, clicked_fn=self.load_robot_new, style={ "margin_height": 2, "font_size": 12, "color": "cyan"})
                    # ui.Button("Load mission", width = 40, clicked_fn=self.load_task, style={ "margin_height": 2, "font_size": 12, "color": "lightgreen"})
                    ui.Button("Add house", clicked_fn=self.auto_add_house, style={ "margin": 4, "font_size": 12, "color": "Violet"})

                with ui.HStack(height = 10):
                    ui.Button("Load house + anchor", clicked_fn=self.load_house_new, style={ "margin": 4}, visible = False)
                    ui.Button("Record house + anchor", clicked_fn=self.record_house_new, style={ "margin": 4})
                    ui.Label(" |", width=10)
                    ui.Button("Record robot", clicked_fn=self.record_robot_new, style={ "margin": 4})
                    ui.Label(" |", width=10)
                    ui.Button("Record object", width = 40, clicked_fn=self.record_obj_new, style={ "margin": 4})
                    

                with ui.HStack(height = 4):
                    ui.Label("-"*120) 

                # with ui.HStack(height=20):
                #     ui.Label("Trial: ", width = 30, style={ "margin": 2, "color": "Wheat"})
                #     self.trial_id_ui = omni.ui.IntField(height=20, width = 20, style={ "margin": 2 , "color": "Wheat"}) 
                #     ui.Button("Load mission & Record", clicked_fn=self.load_mission_and_start_record, style={ "margin": 2, "color": "lightgreen"})
                #     ui.Button("Stop", width = 30, clicked_fn=self.stop_record, style={ "margin": 2, "color": "yellow"})
                #     ui.Button("Load mission & Replay", clicked_fn=self.load_mission_and_replay_traj, style={ "margin": 2, "color": "firebrick"})

                self.auto_suggest.info_ui = omni.ui.StringField(style={ "margin": 8, "font_size": 12, "color": "Yellow"}, multiline = True, read_only  = True, )
                
                with ui.HStack(height=30, visible = False):
                    ui.Button("Reset", style={"margin_height": 2, "font_size": 12 },
                            clicked_fn=self._on_click_reset)  

    def _on_click_reset(self):
        self.auto_suggest.reset_ui()

    def _on_suggestion_task_type_change(self, model, item):
        # task_index = self.auto_suggest.suggest_task_type_ui.model.get_item_value_model().get_value_as_int()
        # task_type = TASK_TYPES[task_index - 1]
        # print("task_type", task_type)
        # self.auto_suggest.suggest_task()
        self.auto_suggest.read_ui()
        self.task_type_ui.model.get_item_value_model().set_value(self.auto_suggest.task_type_index - 1)

        # update csv_row_id
        self.auto_label_csv_row_id_ui.model.set_value(-1)
        task_type_index = self.auto_suggest.suggest_task_type_ui.model.get_item_value_model().get_value_as_int()
        if task_type_index == 0: 
            carb.log_warn("Please select a task type")
            return

        task_type = TASK_TYPES[task_type_index - 1]

        if (not hasattr(self, "auto_labeler")) or (self.auto_labeler.task_type != task_type):
            self.auto_labeler.set_task_type(task_type)
            time.sleep(0.5)

    def _on_suggestion_task_id_changed(self, model):
        self.auto_suggest.read_ui()
        # self.auto_suggest.suggest_robot()
        self.task_id_ui.model.set_value(self.auto_suggest.task_id)

    def _on_suggestion_robot_id_changed(self, model):
        self.auto_suggest.read_ui()
        # self.auto_suggest.suggest_mission()
        self.robot_id_ui.model.set_value(self.auto_suggest.robot_id)
    
    # def _on_suggestion_trial_number_changed(self, model):
    #     num = self.auto_suggest.suggest_trial_num()
    #     self.trial_id_next_ui.model.set_value(num)

    
    def _on_suggestion_mission_id_changed(self, model):
        #self.auto_suggest.read_ui()
        self.auto_suggest.suggest_goal()
        self.mission_id_ui.model.set_value(self.auto_suggest.mission_id)
    
    def _on_suggestion_house_id_changed(self, model):
        self.auto_suggest.read_ui()
        # self.auto_suggest.suggest_houseID()
        self.house_id_ui.model.set_value(self.auto_suggest.house_id)

    def _on_suggestion_anchor_id_changed(self, model):
        self.auto_suggest.read_ui()
        # self.auto_suggest.suggest_anchor_id()
        self.anchor_id_ui.model.set_value(self.auto_suggest.anchor_id) 
    
    def _on_csv_row_id_changed(self, model):
        # record original value
        self.auto_suggest.read_ui()
        original_task_id = self.auto_suggest.task_id
        original_house_id = self.auto_suggest.house_id
        original_robot_id = self.auto_suggest.robot_id
        original_trial_id = self.trial_id_ui.model.get_value_as_int()

        if not hasattr(self, "auto_labeler"):
            carb.log_error("Please select a task type")
            return

        if self.auto_label_csv_row_id_ui.model.get_value_as_int() < 2:
            carb.log_warn("Please enter a valid csv row id")
            self.csv_row_id = self.auto_label_csv_row_id_ui.model.get_value_as_int()
            return 

        self.auto_labeler.current_id = self.auto_label_csv_row_id_ui.model.get_value_as_int() - 2
        task_id, robot_id, mission_id, house_id, trial_id = self.auto_labeler.load_row()
        print("task_id, robot_id, mission_id, house_id, trial_id", task_id, robot_id, mission_id, house_id, trial_id)
        self.auto_suggest.suggest_task_id_model.set_value(task_id)
        self.auto_suggest.suggest_robot_id_model.set_value(robot_id)
        self.auto_suggest.suggest_mission_id_model.set_value(mission_id)
        self.auto_suggest.suggest_house_id_model.set_value(house_id)
        self.auto_suggest.suggest_anchor_id_model.set_value(0)
        self.trial_id_ui.model.set_value(trial_id)

        new_csv_row_id = self.auto_label_csv_row_id_ui.model.get_value_as_int()

        # need scene renewal
        need_renew = False
        
        # if only the trial id changes, don't need renew scene, else, reload scene
        if new_csv_row_id >= 2 and (original_task_id != task_id or original_house_id != house_id): 
            need_renew = True

        # if reset sample id
        if new_csv_row_id >= 2 and new_csv_row_id != self.csv_row_id + 1:
            need_renew = True
    
        # update row id
        self.csv_row_id = new_csv_row_id

        # add some hints
        if robot_id != original_robot_id:
            BaseChecker.SUCCESS_UI.model.set_value("Remember to move '/World/game/franka' and record robot id (robot variation) if needed")
            select_prim_path = "/World/game/franka" 
            selection = omni.usd.get_context().get_selection()
            selection.clear_selected_prim_paths()
            selection.set_prim_path_selected(select_prim_path, True, True, True, True)
        else:
            if original_trial_id != trial_id:
                BaseChecker.SUCCESS_UI.model.set_value("Trial id changed!")
            else:
                BaseChecker.SUCCESS_UI.model.set_value("")

        if need_renew:
            # asyncio.ensure_future(self.load_new_task())
            omni.usd.get_context().close_stage()
            omni.usd.get_context().new_stage()
            self.load_obj_new()
            self.load_robot_new()
            self.load_house_new()
            if not self.load_house_successful:
                self.auto_add_house()
            
            select_prim_path = "/World/game" 
            selection = omni.usd.get_context().get_selection()
            selection.clear_selected_prim_paths()
            selection.set_prim_path_selected(select_prim_path, True, True, True, True)

            ui_text = f"Successfully loaded (with anchor)" if self.load_house_successful else "Move '/World/game' and record anchor."
            BaseChecker.SUCCESS_UI.model.set_value(ui_text)



    
    async def load_new_task(self):
        # update ui 
        BaseChecker.SUCCESS_UI.model.set_value("Loading...............")

        # new scene
        await omni.usd.get_context().close_stage_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        await asyncio.sleep(0.5)
        # load task, robot, house
        self.load_obj_new()
        await asyncio.sleep(0.5)
        self.load_robot_new()
        await asyncio.sleep(1)
        self.load_house_new()
        if not self.load_house_successful:
            time.sleep(0.5)
            self.auto_add_house()

        await asyncio.sleep(1)
        select_prim_path = "/World/game" 
        selection = omni.usd.get_context().get_selection()
        time.sleep(0.5)
        if selection and selection is not None:
            selection.clear_selected_prim_paths()
            time.sleep(0.5)
            selection.set_prim_path_selected(select_prim_path, True, True, True, True)

        ui_text = f"Successfully loaded (with anchor)" if self.load_house_successful else "Move '/World/game' and record anchor."
        BaseChecker.SUCCESS_UI.model.set_value(ui_text)

    def auto_label_load_next(self):
        """
        get auto labeler and load next ids
        """
        # task_type_index = self.auto_suggest.suggest_task_type_ui.model.get_item_value_model().get_value_as_int()
        # assert task_type_index > 0, "Please select a task type"
        # task_type = TASK_TYPES[task_type_index - 1]

        # if (not hasattr(self, "auto_labeler")) or (self.auto_labeler.task_type != task_type):
        #     self.auto_labeler = AutoLabeler(task_type)
        #     need_renew = True
        
        # self.auto_labeler.current_id = self.auto_label_csv_row_id_ui.model.get_value_as_int() - 2

        if (not hasattr(self, "auto_labeler")):
            raise Exception("Please select a task type")

        self.auto_labeler.next()
        self.auto_label_csv_row_id_ui.model.set_value(self.auto_labeler.current_id + 2)
        self._on_csv_row_id_changed(None)

    ###################################################################################
    ################################ Robot       ######################################
    ###################################################################################
    
    def setup_robot(self, new_method = False): 
        """
        Set up robot in the currect example
        """

        # get the game xform as the parent for the robot
        self.stage = omni.usd.get_context().get_stage()
        #game_xform = self.stage.GetPrimAtPath("/World/game")
        robot_parent_path = "/World/game"
        has_game_xform = True
        if not self.stage.GetPrimAtPath(robot_parent_path):
            has_game_xform = False
            xform_game = pxr.UsdGeom.Xform.Define(self.stage, robot_parent_path)
            xform_game.AddTranslateOp().Set(pxr.Gf.Vec3f(0.0, 0.0, 0.0))
            xform_game.AddOrientOp().Set(pxr.Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            xform_game.AddScaleOp().Set(pxr.Gf.Vec3f(1.0, 1.0, 1.0))

        # retreive timeline
        # _timeline = omni.timeline.get_timeline_interface()
        # _timeline.play() # default not playing 

        if not new_method:
            # old method
            # load json info from example
            task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
            task_type = self.task_types[task_index]
            task_id = self.task_id_ui.model.get_value_as_int()
            house_id = self.house_id_ui.model.get_value_as_int()
            object_id = self.object_id_ui.model.get_value_as_int()
            task_json = os.path.join(DATA_PATH, "tasks", task_type, str(house_id), str(object_id), str(task_id) + ".json")
            print("task json: ", task_json)

            has_robot_info = False
            if os.path.exists(task_json):
                # raise Exception( "The json file at path {} provided wasn't found".format(room_layout_json) )
                layout = json.load(open(task_json))

                if "robot" in layout:
                    position = layout["robot"]["position"]
                    rotation = layout["robot"]["rotation"]
                    has_robot_info = True
            
            # if there is no robot information / or no game_xform
            if not has_robot_info or not has_game_xform:
                carb.log_warn("Don't know the location/rotation for the robot")
                position = [0,0,0]
                rotation = [-0.5,0.5,0.5,0.5]
        
        # new robot loading method
        else:
            #from .layout.house_new import HouseNew
            self.init_new_house()
            position, rotation = self.house.load_robot_info()
                
        # print("position, rotation ", np.array(position), np.array(rotation))
        if (not self.is_initial_setup) and IS_IN_ISAAC_SIM:
            # target_path = "/World/game/mobility_Bottle_3618"
            target_path = None
            for target_prim in self.stage.GetPrimAtPath("/World/game").GetChildren():
                if "mobility" in target_prim.GetPath().pathString:
                    target_path = target_prim.GetPath().pathString
            
            if target_path is None:
                raise Exception("Must have a game object with mobility in the scene")
            
            # self.franka = FrankabotKeyboard()
            self.franka = FrankabotGamePad(target_path, position=np.array(position), rotation=np.array(rotation), parent_path=robot_parent_path)
        else:
            franka_path = os.path.join(ROBOT_PATH, "franka/franka.usd")
            
            # load robot
            robot_prim = self.stage.GetPrimAtPath(robot_parent_path + "/franka")
            if not robot_prim.IsValid():
                robot_prim = self.stage.DefinePrim(robot_parent_path + "/franka")

            success_bool = robot_prim.GetReferences().AddReference(franka_path)
            if not success_bool:
                raise Exception("The usd file at path {} provided wasn't found".format(franka_path))

            # set robot xform
            # robot_xform = pxr.UsdGeom.Xformable.Get(self.stage, robot_prim.GetPath())

            # print("position $ rotation: ", position[0], position[1], position[2], rotation)
            robot_xform_mat = pxr.Gf.Matrix4d().SetScale([1,1,1]) *  \
                    pxr.Gf.Matrix4d().SetRotate(pxr.Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
                    pxr.Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=robot_prim.GetPath().pathString,
                new_transform_matrix=robot_xform_mat,
            )

            # robot_xform.AddTranslateOp().Set(pxr.Gf.Vec3f(float(position[0]), float(position[1]), float(position[2])))
            # robot_xform.AddOrientOp().Set(pxr.Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3])))
            # robot_xform.AddScaleOp().Set(pxr.Gf.Vec3f(1.0, 1.0, 1.0))

            # selection = omni.usd.get_context().get_selection()
            # selection.clear_selected_prim_paths()
            # selection.set_prim_path_selected(robot_parent_path + "/franka", True, True, True, True)

        from pxr import PhysxSchema, UsdPhysics
        physicsScenePath = "/World/physicsScene"
        scene = UsdPhysics.Scene.Get(self.stage, physicsScenePath)
        if not scene:
            scene = UsdPhysics.Scene.Define(self.stage, physicsScenePath)
            self._gravityDirection = pxr.Gf.Vec3f(0.0, -1.0, 0.0)
            self._gravityMagnitude = 981
            scene.CreateGravityDirectionAttr().Set(self._gravityDirection)
            scene.CreateGravityMagnitudeAttr().Set(self._gravityMagnitude)

        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.CreateEnableCCDAttr().Set(True)
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(120)
        physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(True)
        physxSceneAPI.CreateEnableEnhancedDeterminismAttr().Set(True)

        physxSceneAPI.CreateEnableStabilizationAttr().Set(True)
    
    def _on_franka_easy_control_button_clicked(self):
        if IS_IN_ISAAC_SIM:
            if self.franka_easy_control_button.text == "EE Rotation Control":
                self.franka_easy_control_button.text = "Joint Control"
                FrankabotGamePad.ANTI_HUMAN_CONTROL = False
            else:
                self.franka_easy_control_button.text = "EE Rotation Control"
                FrankabotGamePad.ANTI_HUMAN_CONTROL = True

    ###################################################################################
    ################################ Liquid       ######################################
    ###################################################################################
   

    def init_fluid_helper(self):
        from .layout.fluid.cup_setup import CupFluidHelper
        # cup_id = 0 # self.cup_id_ui.model.get_value_as_int()
        # r = self.r_ui.model.get_value_as_float()
        # g = self.g_ui.model.get_value_as_float()
        # b = self.b_ui.model.get_value_as_float()
 
        self.cup_fluid_helper = CupFluidHelper()

    def set_up_fluid_helper(self): 
        # Fluid System setup
        self.init_fluid_helper()
        self.cup_fluid_helper.create()
    
    def add_liquid_to_cup(self, task_type):
        self.init_fluid_helper()
        self.stage = omni.usd.get_context().get_stage()
        game_prim = self.stage.GetPrimAtPath("/World/game")

        enable_physics = True
        if task_type == 'tap_water':
            enable_physics = False
        for prim in game_prim.GetChildren():
            if "mobility_" in prim.GetPath().pathString and task_type in ["pour_water", "transfer_water"]:
                self.cup_fluid_helper.modify_cup_scene(prim, add_liquid = True, set_physics = enable_physics)
            elif "container_" in prim.GetPath().pathString:
                self.cup_fluid_helper.modify_cup_scene(prim, add_liquid = False, set_physics = enable_physics)


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
            print(rotateXYZ)
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

    def fix_linear_joint(self, fix_driver = True, damping_cofficient = 1 ):
        self.stage = omni.usd.get_context().get_stage()
        prim_list = self.stage.TraverseAll()
        for prim in prim_list:
            if "joint_" in str(prim.GetPath()):
                
                if fix_driver:
                    # find linear drive
                    joint_driver = pxr.UsdPhysics.DriveAPI.Get(prim, "linear")
                    if joint_driver:
                        joint_driver.CreateDampingAttr(damping_cofficient)

                    # find linear drive
                    joint_driver = pxr.UsdPhysics.DriveAPI.Get(prim, "angular")
                    if joint_driver:
                        joint_driver.CreateDampingAttr(damping_cofficient)
                
                # find linear joint upperlimit
                joint = pxr.UsdPhysics.PrismaticJoint.Get(self.stage, prim.GetPath())	
                if joint:
                    upper_limit = joint.GetUpperLimitAttr().Get() #GetAttribute("xformOp:translate").Get()
                    print(prim.GetPath(), "upper_limit", upper_limit)
                    mobility_prim = prim.GetParent().GetParent()
                    mobility_xform = pxr.UsdGeom.Xformable.Get(self.stage, mobility_prim.GetPath())
                    scale_factor = mobility_xform.GetOrderedXformOps()[2].Get()[0]
                    print("scale_factor", scale_factor)
                    joint.CreateUpperLimitAttr(upper_limit * scale_factor / 100)