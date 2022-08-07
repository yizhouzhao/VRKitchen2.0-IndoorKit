# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["CustomRadioCollection"]

from typing import List, Optional

import omni
import omni.ui as ui

from .style import ATTR_LABEL_WIDTH, cl

SPACING = 5


class CustomRadioCollection:
    """A custom collection of radio buttons.  The group_name is on the first
    line, and each label and radio button are on subsequent lines.  This one
    does not inherit from CustomBaseWidget because it doesn't have the same
    Head label, and doesn't have a Revert button at the end.
    """

    def __init__(self,
                 group_name: str,
                 labels: List[str],
                 model: ui.AbstractItemModel = None,
                 default_value: bool = True,
                 **kwargs):
        self.__group_name = group_name
        self.__labels = labels
        self.__default_val = default_value
        self.__images = []
        self.__selection_model = ui.SimpleIntModel(default_value)
        self.__frame = ui.Frame()
        with self.__frame:
            self._build_fn()

    def destroy(self):
        self.__images = []
        self.__selection_model = None
        self.__frame = None

    @property
    def model(self) -> Optional[ui.AbstractValueModel]:
        """The widget's model"""
        if self.__selection_model:
            return self.__selection_model

    @model.setter
    def model(self, value: int):
        """The widget's model"""
        self.__selection_model.set(value)

    def __getattr__(self, attr):
        """
        Pretend it's self.__frame, so we have access to width/height and
        callbacks.
        """
        return getattr(self.__frame, attr)

    def _on_value_changed(self, index: int = 0):
        """Set states of all radio buttons so only one is On."""
        self.__selection_model.set_value(index)
        for i, img in enumerate(self.__images):
            img.checked = i == index
            img.name = "radio_on" if img.checked else "radio_off"

    def _build_fn(self):
        """Main meat of the widget.  Draw the group_name label, label and
        radio button for each row, and set up callbacks to keep them updated.
        """
        with ui.VStack(spacing=SPACING):
            ui.Spacer(height=2)
            ui.Label(self.__group_name.upper(), name="radio_group_name",
                     width=ATTR_LABEL_WIDTH)

            for i, label in enumerate(self.__labels):
                with ui.HStack():
                    ui.Label(label, name="attribute_name",
                             width=ATTR_LABEL_WIDTH)

                    with ui.HStack():
                        with ui.VStack():
                            ui.Spacer(height=2)
                            self.__images.append(
                                ui.Image(
                                    name=("radio_on" if self.__default_val == i else "radio_off"),
                                    fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                    height=16, width=16, checked=self.__default_val
                                )
                            )
                        ui.Spacer()
            ui.Spacer(height=2)

        # Set up a mouse click callback for each radio button image
        for i in range(len(self.__labels)):
            self.__images[i].set_mouse_pressed_fn(
                lambda x, y, b, m, i=i: self._on_value_changed(i))



class CustomRecordGroup:
    STYLE = {
        "Rectangle::image_button": {
            "background_color": 0x0, 
            "border_width": 1.5, 
            "border_radius": 2.0,
            "margin": 4,
            "border_color": cl.btn_border,
            "corner_flag": ui.CornerFlag.RIGHT,
            },
        "Rectangle::image_button:hovered": {
            "background_color": 0xAAB8B8B8,
            "border_width": 0,
            "border_radius": 2.0,
        },
        "Rectangle::image_button:selected": {
            "background_color": 0x0,
            "border_width": 1,
            "border_color": 0xFFC5911A,
            "border_radius": 2.0,
        },
    }

    def __init__(self, width = 60, height = 60,
        on_click_record_fn: callable = None,
        on_click_stop_fn: callable = None,
        on_click_replay_fn: callable = None,
    ):

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_click_record_fn = on_click_record_fn
        self.on_click_stop_fn = on_click_stop_fn
        self.on_click_replay_fn = on_click_replay_fn

        # another ui for control
        self.control_group = None 


        self._selected = False
        with ui.HStack():
            with ui.HStack():
                with ui.ZStack(width=0, height=0, spacing=0): # 
                    with ui.Placer(offset_x=width, offset_y=0):
                        self.play_label = ui.Label("Record", width = 60)

                    with ui.Placer(offset_x=0, offset_y=0):
                        self.rect_play = ui.Rectangle(name="image_button", width=2 * width, height=height, style=CustomRecordGroup.STYLE)
        
                    with ui.Placer(offset_x=5, offset_y=5):
                        self.image_play = ui.Image(
                        name="start_on", width=width - 10, height=height - 10, fill_policy=ui.FillPolicy.STRETCH
                        )  
                
                    self.rect_play.set_mouse_pressed_fn(lambda x, y, btn, a: self._on_mouse_pressed_play(btn))

            with ui.ZStack(width=0, height=0, spacing=0): # 
                with ui.Placer(offset_x=width, offset_y=0):
                    self.stop_label = ui.Label("Stop", width = 60)

                with ui.Placer(offset_x=0, offset_y=0):
                    self.rect_stop = ui.Rectangle(name="image_button", width=2 * width, height=height, style=CustomRecordGroup.STYLE)
    
                with ui.Placer(offset_x=5, offset_y=5):
                    self.image_stop = ui.Image(
                    name="stop_on", width=width - 10, height=height - 10, fill_policy=ui.FillPolicy.STRETCH
                    )  
            
                self.rect_stop.set_mouse_pressed_fn(lambda x, y, btn, a: self._on_mouse_pressed_stop(btn)) # 

            with ui.HStack():
                with ui.ZStack(width=0, height=0, spacing=0):

                    with ui.Placer(offset_x=width, offset_y=0):
                        self.replay_label = ui.Label("Replay", width = 60)

                    with ui.Placer(offset_x=0, offset_y=0):
                        self.rect_replay = ui.Rectangle(name="image_button", width= 2 * width, height=height, style=CustomRecordGroup.STYLE)
        
                    with ui.Placer(offset_x=10, offset_y=10):
                        self.image_replay = ui.Image(
                        name="replay_on", width=width - 20, height=height - 20, fill_policy=ui.FillPolicy.STRETCH
                        )  
                
                    self.rect_replay.set_mouse_pressed_fn(lambda x, y, btn, a: self._on_mouse_pressed_replay(btn))


    def __del__(self):
        # set ui.Image objects to None explicitly to avoid this error:
        # Client omni.ui Failed to acquire interface [omni::kit::renderer::IGpuFoundation v0.2] while unloading all plugins
        self.image_play = None


    def _on_mouse_pressed_play(self, key):
        # 0 is for mouse left button
        if key == 0:
            if self.timeline.is_stopped(): # if stopped, start recording
                self.play_label.text = "Pause"
                self.image_play.name = "pause_on"
                self.on_click_record_fn()
            elif self.timeline.is_playing(): # if is playing, pause
                self.play_label.text = "Continue"
                self.image_play.name = "start_on"
                self.timeline.pause()
            else: # if is paused, just play
                self.play_label.text = "Pause"
                self.image_play.name = "pause_on"
                self.timeline.play()


    
    def _on_mouse_pressed_replay(self, key):
        # 0 is for mouse left button
        if key == 0:
            if self.timeline.is_stopped(): # if stopped, start recording
                self.replay_label.text = "Pause"
                self.image_replay.name = "pause_on"
                self.on_click_replay_fn()
            elif self.timeline.is_playing(): # if is playing, pause
                self.replay_label.text = "Continue"
                self.image_replay.name = "replay_on"
                self.timeline.pause()
            else: # if is paused, just play
                self.replay_label.text = "Pause"
                self.image_replay.name = "pause_on"
                self.timeline.play()

    def _on_mouse_pressed_stop(self, key):
        # print("press stop button", self.timeline.is_playing(), self.timeline.is_stopped())
        # 0 is for mouse left button
        if key == 0:
            self.play_label.text = "Record"
            self.image_play.name = "start_on"

            self.replay_label.text = "Replay"
            self.image_replay.name = "replay_on"
            
            self.on_click_stop_fn()

    @property
    def selected(self):
        return self._selected

    @selected.setter
    def selected(self, value):
        self._selected = value

class CustomControlGroup():
    def __init__(self) -> None:
        self.collapse_frame = ui.CollapsableFrame("\tRobot control")
        self.collapse_frame.collapsed = False
        self.collapse_frame.enabled = True

        # control
        self.W = False
        self.S = False
        self.A = False
        self.D = False
        
        self.UP = False
        self.DOWN = False
        self.LEFT = False
        self.RIGHT = False
        

        # ui
        with self.collapse_frame:
            with ui.VStack(height=0, spacing=0):
                with ui.HStack():
                    ui.Label("position control: ")
                    ui.Button("W", name = "control_button", tooltip = "move end factor forward")
                    ui.Button("S", name = "control_button", tooltip = "move end factor backward")
                    ui.Button("A", name = "control_button", tooltip = "move end factor to left")
                    ui.Button("D", name = "control_button", tooltip = "move end factor to right")

                with ui.HStack():
                    ui.Label("rotation control: ")
                    ui.Button("UP", name = "control_button",  tooltip = "Rotate hand upward")
                    ui.Button("DOWN", name = "control_button", tooltip = "Rotate hand downard")
                    ui.Button("LEFT", name = "control_button", tooltip = "Rotate hand to left")
                    ui.Button("RIGHT", name = "control_button", tooltip = "Rotate hand to right")

                with ui.HStack():
                    ui.Label("gripper control: ")
                    ui.Button("LEFT CTRL", name = "control_button", tooltip = "Close/Open gripper")
            