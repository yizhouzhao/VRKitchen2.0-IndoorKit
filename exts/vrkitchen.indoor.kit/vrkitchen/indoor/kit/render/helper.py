import math
import time
import typing
import asyncio

import carb
import omni
import numpy as np
from PIL import Image

import os

import omni.syntheticdata as syn
from omni.kit.window.popup_dialog import MessageDialog

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
        self.render_type = "Rgb"
        
        # viewport = omni.kit.viewport_legacy.get_viewport_interface()
        # viewport_handle = viewport.get_instance("Viewport")

        from omni.kit.viewport.utility import get_active_viewport
        self.viewport = get_active_viewport()
        self.viewport_window = omni.kit.viewport.utility.get_viewport_from_window_name() # viewport.get_viewport_window(None) 
        self.timeline = omni.timeline.get_timeline_interface()
        
        
    def render_image(self, export_folder = None, prefix = ""):
        print("rendering image...")
        self.stage = omni.usd.get_context().get_stage()
        # get camera
        # self.viewport_window.set_texture_resolution(*resolution)
        camera_name = self.viewport_window.get_active_camera().pathString.replace("/","")

        # set up export folder
        if export_folder:
            if not os.path.exists(export_folder):
                os.makedirs(export_folder, exist_ok=True)

            time_str = str(int(self.timeline.get_current_time() * self.stage.GetTimeCodesPerSecond()))
            img_save_path = f"{export_folder}/{prefix}_{camera_name}_{self.render_type}_{time_str}.png"
    
        # get render type
        # synthetic_type = syn._syntheticdata.SensorType.Rgb
        # if self.render_type == "Depth":
        #     synthetic_type = syn._syntheticdata.SensorType.DepthLinear
        # elif self.render_type == "Semantic":
        #     synthetic_type = syn._syntheticdata.SensorType.SemanticSegmentation

        # render and export
        async def render_img():
            # Render one frame  
            await omni.kit.app.get_app().next_update_async()
            

            syn.sensors.enable_sensors(
                self.viewport,
                [
                    syn._syntheticdata.SensorType.Rgb, 
                    syn._syntheticdata.SensorType.DepthLinear, 
                    syn._syntheticdata.SensorType.SemanticSegmentation, 
                    syn._syntheticdata.SensorType.InstanceSegmentation
                ],
            )
            # # await syn.sensors.initialize_async(self.viewport_window, [])
            # await syn.sensors.next_sensor_data_async(self.viewport, True) 
            # if self.render_type == "Depth":
            #     from omni.syntheticdata.scripts.visualize import get_depth
            #     data = get_depth(self.viewport_window, mode = "linear") 
            #     # print("img", data.shape)
            #     img = Image.fromarray(data.astype(np.uint8))

            
            if self.render_type == "Depth":
                await syn.sensors.next_sensor_data_async(self.viewport) 
                data = syn.sensors.get_depth_linear(self.viewport)
                print("depthimg", data.shape)
                img = Image.fromarray(data.astype(np.uint8)) 

            elif self.render_type == "Semantic":
                await syn.sensors.next_sensor_data_async(self.viewport) 
                data = syn.sensors.get_instance_segmentation(self.viewport, parsed = True) 
                img = Image.fromarray(data.astype(np.uint8))

            else:
                await syn.sensors.next_sensor_data_async(self.viewport) 
                data = syn.sensors.get_rgb(self.viewport)
                print("img", data.shape, data.dtype)
                img = Image.fromarray(data)

            if export_folder: 
                img.save(img_save_path)
                print("image saved at path: ", img_save_path)

            dialog = MessageDialog(
                title="Image capture",
                message=f"Screenshot captured!",
                disable_cancel_button=True,
                ok_handler=lambda dialog: dialog.hide()
            )
            dialog.show()

        asyncio.ensure_future(render_img())

        
        
        