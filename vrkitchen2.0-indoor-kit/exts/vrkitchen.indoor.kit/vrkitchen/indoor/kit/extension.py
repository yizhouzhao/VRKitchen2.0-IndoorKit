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
    
    ################################################################################################
    ######################################## Build omni ui window ##################################
    ################################################################################################

    def build_setup_layout_window(self):
        """
        Build a window to control/debug layout 
        """
        self._window = ui.Window("VRKitchen2.0-Indoor-Kit", width=300)
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

                    ui.Button("+ task id", clicked_fn=self.auto_next_task, style={ "margin": 8})
                    ui.Button("+ object id", clicked_fn=self.auto_next_obj_only, style={ "margin": 8})

                with ui.HStack(height=30):
                    ui.Label("House ", width=20)
                    self.house_id_ui = omni.ui.IntField(height=20, width = 40, style={ "margin": 8 })
                    self.house_id_ui.model.set_value(-1)

                    ui.Label("Anchor ", width=20)
                    self.anchor_id_ui = omni.ui.IntField(height=20, width = 40, style={ "margin": 8 })
                    self.anchor_id_ui.model.set_value(-1)

                    ui.Label("Robot ", width=20)
                    self.robot_id_ui = omni.ui.IntField(height=20, width = 40, style={ "margin": 8 })
                    
                    ui.Label("Mission ", width=20)
                    self.mission_id_ui = omni.ui.IntField(height=20, width = 40, style={ "margin": 8 })
                    
                    ui.Label(" | ", width=10)
                    ui.Label("Object ", width=20)
                    self.object_id_ui = omni.ui.IntField(height=20, width = 40, style={ "margin": 8 })
                    self.object_id_ui.model.set_value(-1)

                with ui.HStack(height=30):
                    ui.Button("Add obj", clicked_fn=self.auto_add_obj, style={ "margin": 4})
                    ui.Button("Add robot", clicked_fn=self.auto_add_robot, style={ "margin": 4})
                    ui.Button("Add mission", clicked_fn=self.auto_add_mission, style={ "margin": 4})
                    ui.Label(" |", width=10)
                    ui.Button("Add house", clicked_fn=self.auto_add_house, style={ "margin": 4})

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