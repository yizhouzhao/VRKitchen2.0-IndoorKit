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
# from .layout.house import House
from .layout.randomizer import Randomizer
from .layout.utils import add_semantics

from .layout.house_new import House as HouseNew
from .autotask.auto import AutoTasker
# from .autotask.auto_label import AutoLabeler
from .render.helper import CustomSyntheticDataHelper 


###################### ui import ################
from .ui.indoorkit_ui_widget import TaskTypeComboboxWidget, CustomRecordGroup, CustomControlGroup, CustomBoolWidget, CustomSliderWidget, \
    CustomSkySelectionGroup, CustomIdNotice, CustomPathButtonWidget, CustomRenderTypeSelectionGroup

from omni.kit.window.popup_dialog import MessageDialog

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
        # carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(FPS))

        # stage and timeline
        self.stage = omni.usd.get_context().get_stage()
        pxr.UsdGeom.SetStageUpAxis(self.stage, pxr.UsdGeom.Tokens.y)
        self.timeline = omni.timeline.get_timeline_interface()

        # robot
        self.franka = None
        # self.auto_labeler = AutoLabeler(None)
        self.task_type = None

        # set up render
        self.use_isosurface = False # use isosurface
        self.render_folder = RENDER_ROOT
        self.render_helper = CustomSyntheticDataHelper()

        # build windows
        self.build_setup_layout_window()

    
    ################################################################################################
    ######################################## Build omni ui window ##################################
    ################################################################################################

    def build_setup_layout_window(self):
        """
        Build a window to control/debug layout 
        """
        from  .ui.style import julia_modeler_style
        self._window = ui.Window("VRKitchen2.0-Indoor-Kit", width=390)

        with self._window.frame:
            self._window.frame.style = julia_modeler_style
            with ui.ScrollingFrame():
                with ui.VStack(height=0):
                    self.task_desc_ui = ui.StringField(height=20, style={ "margin_height": 2})
                    self.task_desc_ui.model.set_value(" Welcome to VRKitchen2.0 Indoor Kit!")
                    ui.Spacer(height=10)
                    ui.Line(style_type_name_override="HeaderLine")

                    self.task_layout_collapse_ui =  ui.CollapsableFrame("TASK LAYOUT", build_header_fn=self._build_custom_frame_header) 
                    # self.task_layout_collapse_ui.set_collapsed_changed_fn(lambda x:self.on_task_layout_ui_collapse(x))
                    with self.task_layout_collapse_ui:
                        with ui.VStack(height=0, spacing=0):
                            ui.Line(style_type_name_override="HeaderLine")
                            ui.Spacer(height = 12)
                            with ui.HStack(height=30):
                                # set up tasks  
                                self.task_types = TASK_TYPES
                                # ui.Label(" Task type: ", width = 30, style={ "margin": 2 , "color": "cornflowerblue", "font_size":18})
                                # default_task_index = self.task_types.index("pickup_object")
                                # self.task_type_ui = ui.ComboBox(default_task_index, width = 200, *self.task_types, style={ "margin": 8, "color": "cornflowerblue", "font_size":18})
 
                                self.task_type_ui = TaskTypeComboboxWidget(label="Task type:\t", options=self.task_types, on_restore_fn=self.fill_task_info)
                                
                                
                                # ui.Button(" + ", clicked_fn=self.auto_next_task, width = 20, style={ "margin_height": 8})
                                # ui.Button("+ object id", clicked_fn=self.auto_next_obj_only, style={ "margin": 8})

                                self.annotators = ANNOTATORS
                                ui.Label("  Annotator: ", width = 30, style={ "font_size": 12 , "color": "PowderBlue"}, visible = False)
                                annotator_index = ANNOTATORS.index("MyLuckyUser")
                                self.annotator_ui = ui.ComboBox(annotator_index, width = 100, *self.annotators, style={ "margin_height": 8, "font_size": 12, "color": "PowderBlue" }, visible=False)
                                # self.auto_suggest.annotator_ui = self.annotator_ui

                            with ui.HStack(height=30):
                                with ui.HStack():
                                    ui.Label("\tObject id: ", width=30, style={"color": "DarkSalmon"})
                                    self.task_id_ui = omni.ui.IntField(width = 30, name = "choose_id", style={ "color": "DarkSalmon"})    

                                    ui.Button("+", width = 30, style={"margin_height": 8,  "color": "DarkSalmon", "border_color": 1, "border_width": 1},
                                        clicked_fn=lambda: self.task_id_ui.model.set_value(min(self.task_id_ui.model.get_value_as_int() + 1, 19)))
                                    ui.Button("-", width = 30, style={ "margin_height": 8, "color": "DarkSalmon", "border_color": 1, "border_width": 1},
                                        clicked_fn=lambda: self.task_id_ui.model.set_value(max(self.task_id_ui.model.get_value_as_int() - 1, 0 )))
                                    
                                    ui.Button("Add object", name = "add_button", clicked_fn=self.auto_add_obj, style={ "color": "DarkSalmon"})
                            

                                ui.Label("  Object ", width=20, visible = False)
                                self.object_id_ui = omni.ui.IntField(height=20, width = 25, style={ "margin_height": 8 , "margin_width": 4},  visible = False)
                                self.object_id_ui.model.set_value(0)
                                ui.Button("+", width = 20, style={"margin_height": 8, "font_size": 12},
                                    clicked_fn=lambda: self.object_id_ui.model.set_value(self.object_id_ui.model.get_value_as_int() + 1),  visible = False)
                                ui.Button("-", width = 20, style={ "margin_height": 8, "font_size": 12},
                                    clicked_fn=lambda: self.object_id_ui.model.set_value(self.object_id_ui.model.get_value_as_int() - 1),  visible = False)

                                ui.Label("  Anchor:", width=20, visible = False)
                                self.anchor_id_ui = omni.ui.IntField(height=20, width = 25, style={ "margin_height": 8 , "margin_width": 4},  visible = False)
                                self.anchor_id_ui.model.set_value(0)
                                ui.Button("+", width = 20, style={"margin_height": 8, "font_size": 12},
                                    clicked_fn=lambda: self.anchor_id_ui.model.set_value(self.anchor_id_ui.model.get_value_as_int() + 1),  visible = False)
                                ui.Button("-", width = 20, style={ "margin_height": 8, "font_size": 12},
                                    clicked_fn=lambda: self.anchor_id_ui.model.set_value(self.anchor_id_ui.model.get_value_as_int() - 1),  visible = False)
            

                                ui.Label("  Robot:", width=20,  visible = False)
                                self.robot_id_ui = omni.ui.IntField(height=20, width = 25, style={ "margin_height": 8 , "margin_width": 4},  visible = False)
                                ui.Button("+", width = 20, style={"margin_height": 8, "font_size": 12},
                                    clicked_fn=lambda: self.robot_id_ui.model.set_value(self.robot_id_ui.model.get_value_as_int() + 1),  visible = False)
                                ui.Button("-", width = 20, style={ "margin_height": 8, "font_size": 12},
                                    clicked_fn=lambda: self.robot_id_ui.model.set_value(self.robot_id_ui.model.get_value_as_int() - 1),  visible = False)
            
                                ui.Label("Mission ", width=20, visible = False)
                                self.mission_id_ui = omni.ui.IntField(height=20, width = 40, style={ "margin": 8 }, visible = False)
                                
                            
                            with ui.HStack(): 
                                ui.Label("\tHouse id: ", width = 30, style = { "color": "Plum", "font_size": 14})
                                self.house_id_ui = omni.ui.IntField(width = 30, name = "choose_id", style={"color": "Plum"})
                                self.house_id_ui.model.set_value(0)
                                ui.Button("+", width = 30, style={"margin_height": 8, "font_size": 14,  "color": "Plum", "border_color": 1, "border_width": 1},
                                    clicked_fn=lambda: self.house_id_ui.model.set_value(min(self.house_id_ui.model.get_value_as_int() + 1, 19)))
                                ui.Button("-", width = 30, style={ "margin_height": 8, "font_size": 14,  "color": "Plum", "border_color": 1, "border_width": 1},
                                    clicked_fn=lambda: self.house_id_ui.model.set_value(max(self.house_id_ui.model.get_value_as_int() - 1, 0)))
                                ui.Button("Add house", name = "add_button", clicked_fn=self.auto_add_house, style={ "color": "Plum"})


                            with ui.HStack(height=20, visible = False): 
                                ui.Button("Add robot", clicked_fn=self.auto_add_robot, style={ "margin": 4})
                                ui.Button("Add mission", clicked_fn=self.auto_add_mission, style={ "margin": 4})
                                # ui.Label(" |", width=10)
                              
                            with ui.HStack(height=20, visible = False):
                                ui.Button("Record object", name = "record_button", clicked_fn=self.record_obj_new, style={ "margin": 4})
                                ui.Button("Record robot", name = "record_button", clicked_fn=self.record_robot_new, style={ "margin": 4})
                                ui.Label(" |", width=10)
                                ui.Button("Record house", name = "record_button", clicked_fn=self.record_house_new, style={ "margin": 4})
                            
                            with ui.HStack(height=20): 
                                ui.Button("Record scene", height = 40, name = "record_button", clicked_fn=self.record_scene, style={ "margin": 4})
                             

                            with ui.HStack(height=20, visible = False):
                                ui.Button("Load object", clicked_fn=self.load_obj_new, style={ "margin": 4})
                                ui.Button("Load robot", clicked_fn=self.load_robot_new, style={ "margin": 4})
                                # ui.Button("Load mission", clicked_fn=self.load_mission, style={ "margin": 4})      
                                ui.Label(" |", width=10)
                                ui.Button("Load house", clicked_fn=self.load_house_new, style={ "margin": 4})

                    ui.Spacer(height = 10)
                    ui.Line(style_type_name_override="HeaderLine")
                    with ui.CollapsableFrame("SCENE UTILITY"):
                        with ui.VStack(height=0, spacing=4):
                            ui.Line(style_type_name_override="HeaderLine")

                            # open a new stage
                            ui.Button("New scene", height = 40, name = "load_button", clicked_fn=lambda : omni.kit.window.file.new(), style={ "margin": 4}, tooltip = "open a new empty stage")
                            
                            # load recorded scene
                            ui.Button("Load scene", height = 40, name = "load_button", clicked_fn=self.load_scene, style={ "margin": 4})
             
                            # ground plan
                            CustomBoolWidget(label="Visible ground:", default_value=False, on_checked_fn = self.auto_add_ground)
                            
                            # light intensity
                            CustomSliderWidget(min=0, max=3000, label="Light intensity:", default_val=1000, on_slide_fn = self.change_light_intensity)
                            
                            # sky selection
                            CustomSkySelectionGroup(on_select_fn=self.randomize_sky)
                            
                            # house material
                            CustomBoolWidget(label="Random house material:", default_value=False, on_checked_fn = self.randomize_material)

                            # water isosurface
                            CustomBoolWidget(label="Enable isosurface:", default_value=False, on_checked_fn = self.enable_isosurface)

                    # PLAY group
                    ui.Spacer(height = 10)
                    ui.Line(style_type_name_override="HeaderLine")
                    with ui.CollapsableFrame("PLAY"):
                        with ui.VStack(height=0, spacing=0):
                            ui.Line(style_type_name_override="HeaderLine") 
                            ui.Spacer(height = 12)
                            
                            # play and record
                            record_group = CustomRecordGroup(
                                on_click_record_fn=self.start_record,
                                on_click_stop_fn=self.stop_record,
                                on_click_replay_fn=self.replay_record,
                                )

                            # robot control
                            control_group = CustomControlGroup()
                            record_group.control_group = control_group                            
                           
                            with ui.CollapsableFrame("Render"):
                                with ui.VStack(height=0, spacing=0):
                                    CustomRenderTypeSelectionGroup(on_select_fn=self.set_render_type)
                                    ui.Button("Capture image", height = 40, name = "tool_button", clicked_fn=self.render_an_image, style={ "margin": 4}, tooltip = "Capture current screenshot")
                    
                    # PATH group
                    ui.Spacer(height = 10)
                    ui.Line(style_type_name_override="HeaderLine")
                    with ui.CollapsableFrame("PATH", collapsed = True):
                        with ui.VStack(height=0, spacing=0):
                            ui.Line(style_type_name_override="HeaderLine") 
                            ui.Spacer(height = 12)

                            CustomPathButtonWidget(label="Task folder:", path=DATA_PATH_NEW)
                            CustomPathButtonWidget(label="Record folder:", path=SAVE_ROOT)
                            CustomPathButtonWidget(label="Render folder:", path=self.render_folder)

    ################################################################################################
    ######################################## Auto task labeling ####################################
    ################################################################################################   

    def fill_task_info(self, reset = False):
        """
        Automatically (randomly fill task type, housing id, and object id)
        :: params:
            reset: if true, set all to zeros
        """
   
        task_type_id = np.random.randint(len(self.task_types)) if not reset else 0
        object_id = np.random.randint(20) if not reset else 0 # task id
        house_id = np.random.randint(20) if not reset else 0 # house id

        self.task_type_ui.model.get_item_value_model().set_value(task_type_id)
        self.task_id_ui.model.set_value(object_id)
        self.house_id_ui.model.set_value(house_id)
    

    def init_auto_tasker(self): 
        """
        Initialize auto task labeling tool
        """
        
        # update stage
        self.stage = omni.usd.get_context().get_stage()
        pxr.UsdGeom.SetStageUpAxis(self.stage, pxr.UsdGeom.Tokens.y)

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
        annotator = "MyLuckyUser" # self.annotators[annotator_index]
        
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

        if self.stage.GetPrimAtPath("/World/game"):
            dialog = MessageDialog(
            title="Add Object",
            message=f"Already have `/World/game` in the scene. Please start a new stage.",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide()
            )
            dialog.show()
            return

        self.auto_tasker.add_obj() 
        # self.auto_tasker.build_HUD()

        if self.stage.GetPrimAtPath("/World/game"):
            self.task_desc_ui.model.set_value("Task object added!")

        self.auto_add_robot()
        
    def auto_add_robot(self):
        self.init_auto_tasker()
        self.auto_tasker.add_robot()

        franka_prim = self.stage.GetPrimAtPath("/World/game/franka")
        if franka_prim:
            self.task_desc_ui.model.set_value("Feel free to move the robot, \nthen you can `Add house`")
            selection = omni.usd.get_context().get_selection()
            selection.clear_selected_prim_paths()
            selection.set_prim_path_selected(franka_prim.GetPath().pathString, True, True, True, True)

            viewport = omni.kit.viewport_legacy.get_viewport_interface()
            if viewport:
                viewport.get_viewport_window().focus_on_selected()


    def auto_add_house(self):
        self.init_auto_tasker()

        if self.stage.GetPrimAtPath("/World/layout"):
            dialog = MessageDialog(
            title="Add house",
            message=f"Already have `/World/layout` in the scene. Please start a new stage.",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide()
            )
            dialog.show()
            return

        self.auto_tasker.add_house()

        layout_prim = self.stage.GetPrimAtPath("/World/layout")
        if layout_prim:
            self.task_desc_ui.model.set_value("House added! Feel feel to move the /World/game and record scene.")
            selection = omni.usd.get_context().get_selection()
            selection.clear_selected_prim_paths()
            selection.set_prim_path_selected("/World/game", True, True, True, True)

            floor_prim = self.stage.GetPrimAtPath("/World/layout/floor")
            
    
    def auto_add_mission(self):
        self.init_auto_tasker()
        self.auto_tasker.add_task()

    ################################################################################################
    ######################################## Modify Scene ##########################################
    ################################################################################################              

    def auto_add_ground(self, visible = False):
        """
        Add ground to the scene
        """
        self.stage = omni.usd.get_context().get_stage()
        if not self.stage.GetPrimAtPath("/World/game"):
            carb.log_error("Please add /World/game first!")
            self.task_desc_ui.model.set_value(f"Please `Add Object`")
            return 

        from .layout.modify import add_ground_plane
        add_ground_plane(visiable=visible)
        self.task_desc_ui.model.set_value(f"Add ground to scene (visible : {visible})")

        selection = omni.usd.get_context().get_selection()
        selection.clear_selected_prim_paths()
        selection.set_prim_path_selected("/World/groundPlane", True, True, True, True)


    def randomize_material(self, rand = True):
        """
        Randomize house materials
        """
        self.stage = omni.usd.get_context().get_stage()
        if not self.stage.GetPrimAtPath("/World/layout"):
            carb.log_error("Please add /World/layout (load scene) first!")
            self.task_desc_ui.model.set_value(f"Please `Load Scene`")
            return 

        self.randomizer = Randomizer()
        self.randomizer.randomize_house(rand = rand)
        self.task_desc_ui.model.set_value("Added floor/wall material")

    def randomize_sky(self, sky_type = None):
        """
        Randomize house materials
        """
        self.randomizer = Randomizer()
        self.randomizer.randomize_sky(sky_type = sky_type)
        self.task_desc_ui.model.set_value("Sky added.")
    
    def randomize_light(self):
        """
        Randomize house materials
        """
        self.randomizer = Randomizer()
        self.randomizer.randomize_light()
        self.task_desc_ui.model.set_value("Random light")

    def change_light_intensity(self, intensity):
        """
        Change default light intensity
        """
        self.stage = omni.usd.get_context().get_stage()
        light_prim = self.stage.GetPrimAtPath("/World/defaultLight")

        if not light_prim:
            # Create basic DistantLight
            omni.kit.commands.execute(
                "CreatePrim",
                prim_path="/World/defaultLight",
                prim_type="DistantLight",
                select_new_prim=False,
                attributes={pxr.UsdLux.Tokens.angle: 1.0, pxr.UsdLux.Tokens.intensity: 1000},
                create_default_xform=True,
            )

            light_prim = self.stage.GetPrimAtPath("/World/defaultLight")

        light_prim.GetAttribute("intensity").Set(float(intensity))

    def enable_isosurface(self, enable = False):
        """
        enable isosurface for water scene
        """
        self.use_isosurface = enable
        dialog = MessageDialog(
            title="Isosurface",
            message=f"Enabled iso surface: {self.use_isosurface} \n Please a [New Scene] and [Load Scene] for water task again.",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide()
        )
        dialog.show()
    
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
        # self.house.build_HUD()
        # print("robot", self.house.robot_id) 
    
    def record_scene(self):
        """
        Record obj + robot + house
        """
        self.init_new_house()
        self.house.record_obj_info()
        self.house.record_robot_info()
        self.house.record_house_info()

        self.task_desc_ui.model.set_value("Scene recorded! Please start a new empty scene [Load scene] \n Note: you don't have to save the current stage.")

        dialog = MessageDialog(
            title="Scene Recorded",
            message=f"Scene recorded! \nPlease start a [New scene] and then [Load scene] \nNote: you don't have to save the current stage.",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide()
        )
        dialog.show()

    def record_obj_new(self):
        """
        New pipeline to record game objects
        """
        self.init_new_house()
        self.house.record_obj_info()
        self.task_desc_ui.model.set_value("object location recorded!")


    def record_robot_new(self):
        """
        New pipeline to record game robots
        """
        self.init_new_house()
        self.house.record_robot_info()

        # if BaseChecker.SUCCESS_UI:
        #     BaseChecker.SUCCESS_UI.model.set_value("robot id (robot variation) recorded")

        self.task_desc_ui.model.set_value("robot location recorded!")
    
    def record_house_new(self):
        self.init_new_house()
        self.house.record_house_info()

        # if BaseChecker.SUCCESS_UI:
        #     BaseChecker.SUCCESS_UI.model.set_value("house-anchor recorded")

        self.task_desc_ui.model.set_value("game location in house recorded!")
    
    def load_scene(self):
        """
        Load obj + robot + house
        """
        self.stage = omni.usd.get_context().get_stage()
        pxr.UsdGeom.SetStageUpAxis(self.stage, pxr.UsdGeom.Tokens.y)

        if self.stage.GetPrimAtPath("/World/game"):
            dialog = MessageDialog(
            title="Load scene",
            message=f"Already have `/World/game` in the scene. Please start a new stage.",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide()
            )
            dialog.show()
            return

        dialog = MessageDialog(
            title="Loading scene ......",
            message=f"Please wait ......",
            disable_cancel_button=True,
            ok_handler=lambda dialog: dialog.hide()
        )
        dialog.show()

        self.load_obj_new()
        self.load_robot_new()
        self.load_house_new()

        # focus on game
        selection = omni.usd.get_context().get_selection()
        selection.clear_selected_prim_paths()
        selection.set_prim_path_selected("/World/game", True, True, True, True)

        viewport = omni.kit.viewport_legacy.get_viewport_interface()
        if viewport:
            viewport.get_viewport_window().focus_on_selected()

        selection.clear_selected_prim_paths()

        dialog.hide()

        dialog2 = MessageDialog(
            title="Loading scene ......",
            message=f"Loading scene complete!",
            disable_cancel_button=True,
            ok_handler=lambda dialog2: dialog2.hide()
        )
        dialog2.show()
        
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
            self.add_liquid_to_cup(task_type, self.use_isosurface)

    def load_robot_new(self):
        """
        New pipeline to load robots objs
        """
        self.is_initial_setup = False
        self.init_new_house()
        self.setup_robot(new_method=True)

        franka_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/game/franka")
        if franka_prim:
            add_semantics(franka_prim, "franka")
    
    def load_house_new(self):
        self.stage = omni.usd.get_context().get_stage()
        self.init_new_house()
        self.load_house_successful = self.house.load_house_info()

        # if load house successfully, randomize sky, floor, and wall
        if self.load_house_successful:
            floor_prim = self.stage.GetPrimAtPath("/World/layout/floor")
            if floor_prim:
                add_semantics(floor_prim, "floor")

            furniture_prim = self.stage.GetPrimAtPath("/World/layout/furniture")
            if furniture_prim:
                add_semantics(furniture_prim, "furniture")

            wall_prim = self.stage.GetPrimAtPath("/World/layout/roomStruct")
            if wall_prim:
                add_semantics(wall_prim, "wall")

            # from .layout.randomizer import Randomizer

            # if not hasattr(self, "house_randomizer"):
            #     self.house_randomizer = Randomizer(None)
            
            # self.house_randomizer.randomize_house(randomize_floor=True, randomize_wall=True)
            # if IS_IN_CREAT:
            # self.house_randomizer.randomize_sky()
            self.randomize_material(rand=True)
            # self.randomize_sky(sky_type="")

            

    ################################################################################################
    ######################################## Second window #########################################
    ################################################################################################              

    # pass
   
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
            task_json = os.path.join(DATA_PATH_ROOT, "tasks", task_type, str(house_id), str(object_id), str(task_id) + ".json")
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
        if False: # (not self.is_initial_setup) and IS_IN_ISAAC_SIM:
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

        # setup physics
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
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(60)
        physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(True)
        physxSceneAPI.CreateEnableEnhancedDeterminismAttr().Set(True)

        physxSceneAPI.CreateEnableStabilizationAttr().Set(True)

    def fix_linear_joint(self, fix_driver = True, damping_cofficient = 1):
        """
        Fix the linear joint limit when scaling an object
        """
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
    

    ###################################################################################
    ################################ Liquid       ######################################
    ###################################################################################
   

    def init_fluid_helper(self, use_isosurface = False):
        from .layout.fluid.cup_setup import CupFluidHelper
        # cup_id = 0 # self.cup_id_ui.model.get_value_as_int()
        # r = self.r_ui.model.get_value_as_float()
        # g = self.g_ui.model.get_value_as_float()
        # b = self.b_ui.model.get_value_as_float()
 
        self.cup_fluid_helper = CupFluidHelper(use_isosurface)

    # def set_up_fluid_helper(self): 
    #     # Fluid System setup
    #     self.init_fluid_helper()
    #     self.cup_fluid_helper.create()
    
    def add_liquid_to_cup(self, task_type, use_isosurface = False):
        self.init_fluid_helper(use_isosurface)
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


    ###################################################################################
    ################################ Play and Record      #############################
    ###################################################################################
    
    def init_franka_tensor(self):
        """
        Init franka tensor controller
        """
        from .param import APP_VERION
        assert APP_VERION >= "2022.0.0", "need Omniverse Isaac-Sim/Create in 2022"

        task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
        task_type = self.task_types[task_index]
        task_id = self.task_id_ui.model.get_value_as_int()
        # robot_id = self.robot_id_ui.model.get_value_as_int()
        # mission_id = self.mission_id_ui.model.get_value_as_int()
        house_id = self.house_id_ui.model.get_value_as_int()
        # anchor_id = self.anchor_id_ui.model.get_value_as_int()
        annotator_index = self.annotator_ui.model.get_item_value_model().get_value_as_int()
        annotator = ANNOTATORS[annotator_index]

        root_dir = '-'.join([str(os.path.join(SAVE_ROOT, annotator, task_type)),str(task_id), str(house_id)])#, \
            #str(robot_id), str(mission_id), str(house_id), str(anchor_id)])

        traj_dir = os.path.join(root_dir, TRAJ_FOLDER)
        # print("traj_dir", traj_dir)

        from .robot_setup.franka_tensor import FrankaTensor
        self.ft = FrankaTensor(save_path=traj_dir)

    def stop_record(self):
        """
        Stop recording button
        """
        if not hasattr(self, "ft"):
            self.timeline.stop()
            carb.log_error( "please load layout and start recording first")
            return

        self.ft.is_record = False
        self.ft.is_replay = False
        self.timeline.stop()

        self.task_desc_ui.model.set_value("Stop.")
    
    def replay_record(self):
        """
        Replay recording button
        """
        self.init_franka_tensor()
        self.ft.is_replay = True
        self.ft.is_record = False

        self.ft.load_record()
       
        self.timeline.play()

        self.task_desc_ui.model.set_value("Start replaying...")

    def start_record(self):
        """
        Play and record
        """
        self.init_franka_tensor()
        self.ft.is_replay = False
        self.ft.is_record = True

        import shutil
        if os.path.exists(self.ft.save_path):
                shutil.rmtree(self.ft.save_path)
        
        os.makedirs(self.ft.save_path, exist_ok=True)

        self.timeline.play()

        self.task_desc_ui.model.set_value("Start recording...")

    def set_render_type(self, render_type):
        """
        Set up rendering type for current camera
        """
        self.render_helper.reset()
        self.render_helper.render_type = render_type
        print("Setting render_type", self.render_helper.render_type)

    def render_an_image(self):
        """
        Render an image to render folder according render type
        """
        task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
        task_type = self.task_types[task_index]
        task_id = self.task_id_ui.model.get_value_as_int()
        house_id = self.house_id_ui.model.get_value_as_int()

        self.render_helper.render_image(self.render_folder, prefix = f"{task_type}_{task_id}_{house_id}")
        self.task_desc_ui.model.set_value("image captured!")
    
    ######################## ui ###############################
       
    def _build_custom_frame_header(self, collapsed, text):
        """
        When task layout ui collapse, show id notified for task, object, and house id
        """
        if collapsed:
            alignment = ui.Alignment.RIGHT_CENTER
            width = 8
            height = 8
        else:
            alignment = ui.Alignment.CENTER_BOTTOM
            width = 8
            height = 8

        with ui.HStack():
            ui.Spacer(width=8)
            with ui.VStack(width=0):
                ui.Spacer()
                ui.Triangle(
                    style = {"Triangle": {"background_color": 0xDDDDDDDD}}, width=width, height=height, alignment=alignment
                )
                ui.Spacer()
            ui.Spacer(width=8)
            ui.Label(text, width = 100)
            if collapsed:
                self.id_note_ui = CustomIdNotice()
                # print("on_task_layout_ui_collapse", task_block_collapsed)
                self.id_note_ui.ui.visible = collapsed

                task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
                task_type = self.task_types[task_index]
                task_id = self.task_id_ui.model.get_value_as_int()
                robot_id = self.robot_id_ui.model.get_value_as_int()
                anchor_id = self.anchor_id_ui.model.get_value_as_int()
                mission_id = self.mission_id_ui.model.get_value_as_int()
                house_id = self.house_id_ui.model.get_value_as_int()

                self.id_note_ui.task_ui.text = task_type
                self.id_note_ui.object_ui.text = f"Object: {task_id}"
                self.id_note_ui.house_ui.text = f"House: {house_id}"

    ############################# shot down #########################

    def on_shutdown(self):
        print("[vrkitchen.indoor.kit] VRKitchen2.0-Indoor-Kit shutdown")

    