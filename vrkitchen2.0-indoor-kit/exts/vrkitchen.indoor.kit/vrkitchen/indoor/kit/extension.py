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

        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.franka = None
        self.auto_labeler = AutoLabeler(None)
        self.task_type = None

        self._window = ui.Window("VRKitchen2.0-Indoor-Kit", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("Some Label")

                def on_click():
                    print("clicked!")

                ui.Button("Click Me", clicked_fn=lambda: on_click())

    def on_shutdown(self):
        print("[vrkitchen.indoor.kit] VRKitchen2.0-Indoor-Kit shutdown")
