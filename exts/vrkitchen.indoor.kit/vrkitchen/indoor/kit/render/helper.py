import math
import time
import typing
import asyncio

import carb
import omni
import numpy as np
from PIL import Image

import omni.syntheticdata as syn

class CustomSyntheticDataHelper:
    def __init__(self):
        # initialize syntheticdata extension
        self.app = omni.kit.app.get_app_interface()
        ext_manager = self.app.get_extension_manager()

        if not ext_manager.is_extension_enabled("omni.syntheticdata"):
            ext_manager.set_extension_enabled("omni.syntheticdata", True)

        self.reset()

    def reset(self):
        # viewport
        viewport = omni.kit.viewport_legacy.get_viewport_interface()
        # viewport_handle = viewport.get_instance("Viewport")
        self.viewport_window = viewport.get_viewport_window(None)

        

     
    def render_image(self, type ="rgb"):
        self.reset()
        # self.viewport_window.set_texture_resolution(*resolution)
        print("vp_window.get_active_camera()",  self.viewport_window.get_active_camera())

        async def render_rgb():
            # Render one frame  
            await omni.kit.app.get_app().next_update_async()
            await syn.sensors.initialize_async(self.viewport_window, [syn._syntheticdata.SensorType.Rgb])
            await syn.sensors.next_sensor_data_async(self.viewport_window.get_id()) 
            data = syn.sensors.get_rgb(self.viewport_window)
            print("get_rgb", data)

        asyncio.ensure_future(render_rgb())

        
        
        
        