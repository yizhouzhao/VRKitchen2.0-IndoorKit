from typing import List, Optional

import omni
import omni.ui as ui

from .style import ATTR_LABEL_WIDTH, cl
from ..robot_setup.controller import Controller

SPACING = 5 

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
        self.control_group : CustomControlGroup = None 


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

                if self.control_group:
                    self.control_group.enable()

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

            if self.control_group:
                self.control_group.disable()

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
        
        # ui 
        with self.collapse_frame:
            with ui.VStack(height=0, spacing=0):
                with ui.HStack():
                    ui.Label("position control: ")
                    self.button_w = ui.Button("W", name = "control_button", tooltip = "move end factor forward")
                    self.button_s = ui.Button("S", name = "control_button", tooltip = "move end factor backward")
                    self.button_a = ui.Button("A", name = "control_button", tooltip = "move end factor to left")
                    self.button_d = ui.Button("D", name = "control_button", tooltip = "move end factor to right")

                    self.button_q = ui.Button("Q", name = "control_button", tooltip = "move end factor to down")
                    self.button_e = ui.Button("E", name = "control_button", tooltip = "move end factor to up")

                with ui.HStack():
                    ui.Label("rotation control: ")
                    self.button_up = ui.Button("UP", name = "control_button",  tooltip = "Rotate hand upward")
                    self.button_down = ui.Button("DOWN", name = "control_button", tooltip = "Rotate hand downard")
                    self.button_left = ui.Button("LEFT", name = "control_button", tooltip = "Rotate hand to left")
                    self.button_right = ui.Button("RIGHT", name = "control_button", tooltip = "Rotate hand to right")

                with ui.HStack():
                    ui.Label("gripper control: ")
                    self.button_control = ui.Button("LEFT CTRL", name = "control_button", tooltip = "Close/Open gripper")
            
        self.button_list = [self.button_w, self.button_s, self.button_a, self.button_d, self.button_q, self.button_e,
                            self.button_up, self.button_down, self.button_left, self.button_right,
                            ]
    
        self.button_w.set_clicked_fn(lambda : self._on_button("w"))
        self.button_s.set_clicked_fn(lambda : self._on_button("s"))
        self.button_a.set_clicked_fn(lambda : self._on_button("a"))
        self.button_d.set_clicked_fn(lambda : self._on_button("d"))
        self.button_q.set_clicked_fn(lambda : self._on_button("q"))
        self.button_e.set_clicked_fn(lambda : self._on_button("e"))

        self.button_up.set_clicked_fn(lambda : self._on_button("up", 2))
        self.button_down.set_clicked_fn(lambda : self._on_button("down", 2))
        self.button_left.set_clicked_fn(lambda : self._on_button("left", 2))
        self.button_right.set_clicked_fn(lambda : self._on_button("right", 2))

        self.button_control.set_clicked_fn(lambda: self._on_button_control())

        self.disable()
    
    def enable(self):
        """
        Enable itself by showing the robot controling buttons
        """
        self.collapse_frame.collapsed = False
        self.collapse_frame.enabled = True

        self.enable_buttons()
    
    def disable(self):
        """
        Disable itself by closing the robot controling buttons
        """
        self.collapse_frame.collapsed = True
        # self.collapse_frame.enabled = False


    def disable_buttons(self):
        for button in self.button_list:
            button.name = "control_button_disabled"
            # button.enabled = False
            Controller.reset_movement()
    
    def enable_buttons(self):
        for button in self.button_list:
            button.enabled = True
            button.name = "control_button"
            Controller.reset_movement()
    

    def _on_button(self, attr_name:str, style = 1):
        attr = getattr(Controller, attr_name)
        # print("attr", attr_name, attr)
        button = getattr(self, f"button_{attr_name}")
        if attr:
            setattr(Controller, attr_name, False)
            button.name = "control_button"
            self.enable_buttons()
        else:
            self.disable_buttons()
            setattr(Controller, attr_name, True)
            button.enabled = True
            button.name = f"control_button_pressed{style}"

    def _on_button_control(self):
        if Controller.left_control:
            Controller.left_control = False
            self.button_control.text = "LEFT CTRL"
            self.button_control.name = "control_button"
        else:
            Controller.left_control = True
            self.button_control.text = "Gripper closed"
            self.button_control.name = "control_button_pressed3"