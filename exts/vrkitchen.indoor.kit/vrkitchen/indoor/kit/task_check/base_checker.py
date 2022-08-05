from lib2to3.pgen2.token import BACKQUOTE
import os
import json
from pxr import PhysxSchema, UsdPhysics
# task completion checking
import pxr 
import omni
import carb
from omni.physx.scripts import physicsUtils
from ..param import DATA_PATH_NEW
from ..layout.randomizer import Randomizer
class BaseChecker():
    SUCCESS_UI = None
    IS_REPLAY  = False
    def __init__(self, task_type, task_id, robot_id, mission_id, annotator="Steven", run_time = True) -> None:
        """
        ::params:
            :run_time: is run-time task checker or not
        """
        # property
        self.task_type = task_type
        self.task_id = str(task_id)
        self.mission_id = str(mission_id)
        self.robot_id = str(robot_id)
        self.data_path = DATA_PATH_NEW
        self.annotator = annotator

        # keep the old mission identifier temporarily
        self.old_mission_identifier = self.task_type + " " + self.task_id + " " + self.robot_id + " " + self.mission_id
        self.mission_identifier_prefix = self.task_type + " " + self.task_id + " "#+ self.robot_id + " " + self.mission_id
        self.mission_identifier_suffix = self.mission_id

        # scene
        self.stage = omni.usd.get_context().get_stage()
        self.default_prim_path_str = self.stage.GetDefaultPrim().GetPath().pathString
        self.timeline = omni.timeline.get_timeline_interface()

        self.current_mission = self.register_mission()
        self.success_steps = 0
        self.success = False
        self.time = 0
        # tasks
        if run_time:
            self.create_task_callback()

            # log
            self.total_step = 0
            self.print_every = 240
            self.checking_interval = 15

            # get time per second
            physicsScenePath = "/World/physicsScene"
            scene = UsdPhysics.Scene.Get(self.stage, physicsScenePath)
            if not scene:
                carb.log_warn("physics scene not found")

            physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
            self.steps_per_second = physxSceneAPI.GetTimeStepsPerSecondAttr().Get()
    
    def register_mission(self):
        """
        Register mission
        """
        task_folder = os.path.join(self.data_path, self.annotator, "task", self.task_type, str(self.task_id))
        if not os.path.exists(task_folder):
            raise carb.log_warn(f"Task folder not exist at {task_folder}")
        
        self.mission_file_path = os.path.join(task_folder, "missions.json")
        if os.path.exists(self.mission_file_path):
            self.missions = json.load(open(self.mission_file_path))
            carb.log_info(f"Loading missions.json at path {self.mission_file_path}")
        else:
            self.missions = {}
            with open(self.mission_file_path, "w") as f:
                json.dump(self.missions, f, indent = 4)
                carb.log_info(f"Saving missions.json at path {self.mission_file_path}")

        for key, value in self.missions.items():
            if key.startswith(self.mission_identifier_prefix) and key.endswith(self.mission_identifier_suffix): 
                return self.missions[key]
        else:
            return {}
    
    def get_diff(self):
        raise NotImplementedError

    def create_task_callback(self):
        stream = self.timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        # subscribe to Physics updates:
        self._physics_update_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

    def _on_timeline_event(self, e):
        """
        set up timeline event
        """
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.it = 0
            self.time = 0
            self.reset()
    
    def reset(self):
        """
        Reset event
        """
        self._physics_update_subscription = None
        self._timeline_subscription = None
        # self._setup_callbacks()
    
    def _on_success_hold(self):
        try:
            if (self.success_steps - 1) % 240 == 0:
                carb.log_info("hold on")
                BaseChecker.SUCCESS_UI.model.set_value("hold on")
        except:
            pass
    
    def _on_success(self):
        carb.log_info("task sucess")
        self.success = True
        try:
            BaseChecker.SUCCESS_UI.model.set_value("task sucess")
            if self.timeline.is_playing() and not BaseChecker.IS_REPLAY:
                self.timeline.pause()
        except:
            pass
            

    def _on_not_success(self):
        # carb.log_info("task not sucess")
        self.success_steps = 0
        self.success = False
        try:
            BaseChecker.SUCCESS_UI.model.set_value("")
        except:
            pass

    def _on_physics_step(self, dt):
        """
        Physics event
        """
        # print("timestep: ", self.time)
        if self.time == 0:
            stage = omni.usd.get_context().get_stage()
            prim_list = list(stage.TraverseAll())

            prim_list = [ item for item in prim_list if 'Isosurface' in item.GetPath().pathString and item.GetTypeName() == 'Mesh' ]
            from pxr import  Sdf
            water_path = Randomizer(None, 1).get_water_material()
            
            for iso2Prim in prim_list:
                
                # omni.kit.commands.execute(
                #     "CreateAndBindMdlMaterialFromLibrary",
                #     mdl_name='/media/nikepupu/fast/omni_lib/lib_path/isaac_sim-2021.2.1/kit/mdl/core/Base/OmniSurfacePresets.mdl',
                #     mtl_name='OmniSurface_ClearWater',
                #     mtl_created_list=None,
                # )
                # water_path = '/World/Looks/OmniSurface_ClearWater'
                rel = iso2Prim.CreateRelationship("material:binding", False)
                rel.SetTargets([Sdf.Path(water_path)])

                # Randomizer.get_water_material(iso2Prim)
        self.time += 1
        self.start_checking()
        
    
    def start_checking(self):
        if self.success_steps > self.steps_per_second * 2:
            self._on_success()
            

    def save_mission(self):
        """
        save mission
        """
        self.missions[self.old_mission_identifier] = self.current_mission

        with open(self.mission_file_path, "w") as f:
                json.dump(self.missions, f, indent = 4)
                carb.log_info(f"Saving missions.json at path {self.mission_file_path}")
    